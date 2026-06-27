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

#ifdef __cplusplus
extern "C" {
#endif

/* 驱动丝/段几何常量 (SDM 模块定义，CR.h 等外部模块共用) */
#ifndef SDM_SEGMENTS
#define SDM_SEGMENTS        2
#endif
#ifndef SDM_WIRES
#define SDM_WIRES           6
#endif
#ifndef SDM_WIRES_PER_SEG
#define SDM_WIRES_PER_SEG   3
#endif

#define SDM_PI       3.14159265358979323846f
#define SDM_EPS      1e-9f

/* ==================== 对外 API ==================== */

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
 * @brief 获取力峰值上限 (N)
 */
float sdm_get_force_peak_limit(void);

#ifdef __cplusplus
}
#endif

#endif /* __SDM_H */
