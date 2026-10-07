/**
 * @file SDM.h
 * @brief 分段微分模型 (Segmented-Differential Model) — 力位混合控制
 *
 * 对外接口只暴露 sdm_init 和 sdm_step。
 * SDM 模块内部持有所有状态 (参数、k_ratio、控制状态)，
 * 外部 (CR.c) 只需设置期望值、调用 sdm_step、把结果发给电机。
 *
 * @date 2026-06-19
 * @author blinest
 */

#ifndef __SDM_H
#define __SDM_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "Sensor/Sensor.h"
#include "CR.h"   /* SDM_SEGMENTS/SDM_WIRES/SDM_WIRES_PER_SEG 由 CR.h 定义 */

/**
 * @brief 运行时设置臂体安装方向 (竖直↔水平动态切换)
 *
 * 影响: 重力在各丝上的投影分布 (f_grav) 与重力补偿方向。
 * 传入世界坐标系下的单位向量; 非法(NULL/零向量)将被忽略。
 *   [0,0,1] 竖直向上 | [1,0,0] 水平沿X | [0,1,0] 水平沿Y
 *
 * @param dir 安装方向 (长度3, 内部会归一化)
 */
void sdm_set_mount(const float dir[3]);

/**
 * @brief 读取当前安装方向 (未设置时返回 [0,0,1])
 * @param dir 输出 (长度3, 单位向量)
 */
void sdm_get_mount(float dir[3]);

/**
 * @brief 计算重力在各丝上引起的张力 (仅重力项, 与弯曲无关)
 *
 * 用于: 从实测张力中扣除重力分量, 使「接触/干扰」判据不被水平安装下的
 *       重力分布污染 (水平安装时该分布为 0:1:2:0:2:1, 最大 1.27N)。
 *
 * @param theta   各段弯曲角 (rad, [SDM_SEGMENTS])
 * @param phi     各段弯曲方向角 (rad, [SDM_SEGMENTS])
 * @param g_out   输出: 6 根丝重力张力 (N, 均 ≥0)
 */
void sdm_gravity_force(const float theta[SDM_SEGMENTS],
                       const float phi[SDM_SEGMENTS],
                       float g_out[SDM_WIRES]);

/**
 * @brief 设置重力补偿增益 (前馈, 用驱动丝张力抵消臂体+尖端重力)
 * @param gain 0=关闭; 1=按模型理论值全额补偿; 可在 0~1 间标定
 */
void sdm_set_grav_comp_gain(float gain);

/** 读取当前重力补偿增益 */
float sdm_get_grav_comp_gain(void);

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 数学常量 ==================== */
#define SDM_PI       3.14159265358979323846f
#define SDM_EPS      1e-9f

/* ==================== 力控模式 ====================
 *   0 = 关闭       : 纯运动学, k_ratio 恒 1
 *   1 = 比例式力平衡 (默认): 只约束「段内张力份额」, 不使用任何绝对张力
 *   2 = 绝对力误差  : legacy, 依赖模型绝对张力 (现场标定不准时误差大)
 *
 * 现场实测: 张力绝对量受安装/干扰影响大、模型绝对力误差大, 但比值稳定 ⇒ 默认 1。
 */
#ifndef SDM_FORCE_MODE
#define SDM_FORCE_MODE   1
#endif

/* 判定「该丝被拉紧」的基准位移阈值 (mm) */
#define SDM_ACTIVE_DL_MM     1e-3f
/* 段内实测张力和低于此值时不介入比值控制 (N) */
#define SDM_RATIO_MIN_SUM_N  0.5f
/* k_ratio 输出限幅 */
#define SDM_KRATIO_MIN       0.6f
#define SDM_KRATIO_MAX       1.4f

/* ==================== 对外 API ==================== */

/**
 * @brief 计算 PCC 基准丝位移 (逆运动学), 供力控/感知共用
 * @param theta      各段弯曲角 (rad, [SDM_SEGMENTS])
 * @param phi        各段弯曲方向角 (rad, [SDM_SEGMENTS])
 * @param R          驱动丝分布半径 (m)
 * @param deltaL_out 输出: 6 根丝基准位移 (m, [SDM_WIRES]), 负值=被拉紧
 */
void sdm_pcc_deltaL(const float theta[SDM_SEGMENTS],
                    const float phi[SDM_SEGMENTS],
                    float R,
                    float deltaL_out[SDM_WIRES]);

/**
 * @brief 计算某段 3 根丝的期望张力份额 (比值, 无量纲)
 *
 *   被拉紧的丝 (ΔL_base < 0) 按拉动量 w ∝ -ΔL ∝ |cos(φ+α_j)| 分配, 松弛丝份额 0。
 *
 * @param seg            段号 0/1
 * @param phi_seg        该段弯曲方向角 (rad) —— 当前实现由 deltaL_base 隐含, 保留形参
 * @param deltaL_base    6 根丝 PCC 基准位移 (m)
 * @param share_des_out  输出: 3 根丝期望份额, 已在 3 根丝上归一化 (和=1), 顺序同 s_seg_to_wires
 * @return 参与分配的「被拉丝」根数 (1~3);
 *         0 = 整段无被拉丝(近直臂) → 填均分, 控制侧应跳过
 */
int sdm_segment_shares(int seg,
                       float phi_seg,
                       const float deltaL_base[SDM_WIRES],
                       float share_des_out[SDM_WIRES_PER_SEG]);

/**
 * @brief 初始化 SDM 模块。
 * @param cable_radius       每段驱动丝半径数组 (m, [2])
 * @param bending_stiffness  弯曲刚度 (N·m/rad)
 * @param force_peak_limit   力峰值上限 (N)
 * @param force_recovery     恢复阈值 [0~1]
 * @param tip_mass           尖端质量 (kg), 用于重力补偿
 * @param mount_dir          臂体初始方向 (世界坐标系, 单位向量, [3])
 *                           水平安装传 [1,0,0] 或 [0,1,0]，竖直传 [0,0,1]
 *                           NULL 则默认竖直向上
 */
void sdm_init(const float cable_radius[SDM_SEGMENTS],
              float bending_stiffness,
              float force_peak_limit,
              float force_recovery,
              float tip_mass,
              const float mount_dir[3]);

/**
 * @brief 一步 SDM 完整控制。
 *
 * 内部流程:
 *   1. 力安全 → theta_safe = theta_desired × safety
 *   2. PCC 逆运动学 → 基准丝长 ΔL_base
 *   3. 从 deltaL_actual 反解真实臂体几何 (θ_actual, φ_actual)
 *   4. 动力学模型基于真实几何计算每根丝目标拉力
 *   5. k_ratio = f(F_real, F_target, ΔL_base)
 *   6. ΔL_out[i] = ΔL_base[i] × k_ratio[i]
 *
 * @param forces         实时 6 路肌腱力 (N)
 * @param theta_desired  各段期望弯曲角 (rad, [2])
 * @param phi_desired    各段期望弯曲方向角 (rad, [2])
 * @param deltaL_actual  实际 6 根丝位移反馈 (mm), NULL 则用期望值近似
 * @param K_force        力控增益 (建议 0.01~0.1)
 * @param R              驱动丝半径 (m)
 * @param deltaL_out     输出: 6 根丝最终位移量 (mm)
 */
void sdm_step(const float forces[SENSOR_NUM],
              const float theta_desired[SDM_SEGMENTS],
              const float phi_desired[SDM_SEGMENTS],
              const float deltaL_actual[SDM_WIRES],
              float K_force,
              float R,
              float deltaL_out[SDM_WIRES]);

/**
 * @brief 计算给定臂体几何下的每根丝目标拉力 (供感知/诊断复用)
 *
 * 与 sdm_step 内部所用的简化动力学模型完全一致:
 *   f_target_i = 几何项(PCC, ∝ θ) + 重力项(随累积姿态变化)
 *
 * @param theta  各段弯曲角 (rad, [SDM_SEGMENTS])
 * @param phi    各段弯曲方向角 (rad, [SDM_SEGMENTS])
 * @param R      驱动丝半径 (m)
 * @param f_out  输出: 6 根丝目标拉力 (N, [SDM_WIRES])
 */
void sdm_force_target(const float theta[SDM_SEGMENTS],
                      const float phi[SDM_SEGMENTS],
                      float R,
                      float f_out[SDM_WIRES]);

#ifdef __cplusplus
}
#endif

#endif /* __SDM_H */
