/**
 * @file ForceBalance.h
 * @brief 上电自动「通道平衡」标定 (差分法, 免疫重力与预紧)
 *
 * 问题:
 *   水平安装时, 仅重力就会让 6 路张力比会发生变化，并不是理想的1:1:1:1:1:1
 *   [预紧 2N 时偏差 24%, 0.5N 时偏差 56%]。
 *   若直接按 τ_i 标定, 会把「重力分布」错当成「通道增益误差」固化进 g_i。
 *
 * 对策 —— 差分测量:
 *   同一姿态下测两次, 两次之差天然消掉「预紧力」与「重力」:
 *       τ_pre,i  = g_i · (P_i + G_i)                (拉之前)
 *       τ_post,i = g_i · (P_i + G_i + Δ_i)          (6 路同向等量拉之后)
 *       Δτ_i     = τ_post,i - τ_pre,i = g_i · Δ_i
 *   若拉动量对称 ⇒ Δ_i 各丝相同 ⇒  g_i ∝ mean(Δτ)/Δτ_i  (不含 P、G)
 *
 *   前提: 拉动前后**姿态不变**。6 路等量同向是轴向加载, 不产生弯曲 ⇒ 成立。
 *
 * 流程:
 *   IDLE → PRE_SETTLE(判稳+取均值) → POST_MOVE(下发加载+确认电机已响应)
 *        → POST_SETTLE(判稳+取均值) → APPLY → DONE/FAIL
 *
 *   两个 SETTLE 阶段都不用固定等待时间, 而是直接看实测张力, 并且
 *   「判稳」与「取均值」合并: 6 路读数在窗口内变化量持续 < FB_SETTLE_TOL_N
 *   即判稳定, 此时窗口内的样本均值就是本次采样值。
 *
 *   POST_MOVE 是 PRE 与 POST 之间的位移过渡策略: 必须等到电机确实动作
 *   (张力相对 pre 出现可见变化) 且超过最短等待时间, 才允许开始判稳 ——
 *   否则指令下发到电机出力之间的空档会被误判为"已稳定"。
 *
 * 失败保护: Δτ 过小、符号不一致(方向错)、离散度过大 → 放弃, 保留 g=1.0, 电机归零。
 *
 * ⚠ 本流程会驱动电机。首次使用前必须机械确认 FB_PULL_MM 的「量值」与「方向」。
 *
 * @date 2026-10-03
 */
#ifndef __FORCE_BALANCE_H
#define __FORCE_BALANCE_H

#include <stdint.h>
#include <stdbool.h>
#include "Sensor/Sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 配置 (上电前确认) ==================== */
#define FB_CAL_ENABLE        1
/* 标定加载量 (mm): 相对零位的绝对目标位置。>0 = 电机正方向。
 * ⚠ 正负必须按实机确认(哪边是"向后"); 过大可能过载或屈曲。
 * 建议取能让各丝张力上升 ≥ 数 N 的最小值 —— Δτ 越大, 信噪比越高。 */
#define FB_PULL_MM           2.0f
/* ↑ 该位移通过 motor_sync_control() 下发, 但**不会使能驱动器**。
 *   标定前必须确保电机已使能 (上位机发 0xAA 0x01), 否则电机不动,
 *   POST_MOVE 等不到响应 → 超时 → Δτ≈0 → 判 FB_ERR_NO_RESPONSE。 */

/* --- 稳定判据 = 采样窗口 (二合一) ---
 * 在「锚定参考」下, 6 路最大偏离 < FB_SETTLE_TOL_N 并连续保持
 * FB_SETTLE_STABLE_MS ⇒ 判为稳定, 同时把这段稳定窗口内的样本取均值作为本次采样值。
 * 参考锚定在窗口起点 ⇒ 判据同时限制窗口内的累计漂移。
 * 偏离超限则重锚参考并清空累计, 重新计时。
 *
 * 注: FB_SETTLE_STABLE_MS 既是「稳定时长」也是「平均窗口长度」,
 *     调大 = 更稳更慢, 调小 = 更快更粗。 */
#define FB_SETTLE_TOL_N      0.10f   /* 稳定阈值 (N): 窗口内最大变化量 */
#define FB_SETTLE_STABLE_MS  300     /* 稳定时长 = 平均窗口长度 (ms) */
#define FB_SETTLE_POLL_MS     20     /* 采样/检测间隔 (ms) */
#define FB_SETTLE_TIMEOUT_MS 4000    /* 兜底超时: 超时强制放行(用已有样本), 防卡死 */

/* --- 加载位移策略 (PRE → POST 之间的过渡) ---
 * 下发加载指令后**不能立刻开始判稳**: 电机从收到指令到真正出力有延迟
 * (CAN 下发 + 加速段), 这段时间张力不变, 会被误判为"已稳定",
 * 导致 τ_post ≈ τ_pre ⇒ Δτ≈0 ⇒ 标定必然失败。
 *
 * 因此 POST_MOVE 阶段**必须确认 6 路电机全部到达指令位置并停稳**才放行:
 *   ① |current_pos - 下发前位置| >= |FB_PULL_MM| - FB_POS_TOL_MM
 *   ② |current_vel| <= FB_VEL_TOL_MM_S
 * 超时则向上位机发送「标定超时帧」(0xBB 0x04, 含未到位电机掩码) 并判失败。 */
#define FB_POS_POLL_MS       100     /* 期间读取电机位置的周期 (ms) */
#define FB_POS_TOL_MM        0.05f   /* 到位容差 (mm) */
#define FB_VEL_TOL_MM_S      0.50f   /* 判定已停稳的速度上限 (mm/s) */
#define FB_MOVE_TIMEOUT_MS  3000     /* 到位超时 (ms) */

#define FB_MIN_DELTA_N       0.5f    /* 单通道最小 Δτ; 低于判失败 */
#define FB_SPREAD_LIMIT      4.0f    /* Δτ max/min 超此值判异常, 放弃标定 */
#define FB_GAIN_MIN          0.5f    /* 单次修正限幅 */
#define FB_GAIN_MAX          2.0f

/* ==================== 对外 API ==================== */

/** 标定状态 */
typedef enum {
    FB_ST_IDLE  = 0,   /* 未运行 */
    FB_ST_BUSY  = 1,   /* 进行中 */
    FB_ST_OK    = 2,   /* 成功 */
    FB_ST_FAIL  = 3,   /* 失败 */
} FbStatus_t;

/** 失败原因 (供上位机定位) */
typedef enum {
    FB_ERR_NONE       = 0,
    FB_ERR_DISABLED   = 1,   /* 编译期关闭 (FB_CAL_ENABLE=0) */
    FB_ERR_NO_RESPONSE= 2,   /* 下发后张力无响应: 电机未使能/未上电/CAN 不通 */
    FB_ERR_DELTA_SMALL= 3,   /* Δτ 过小: 加载量不足或信号太弱 */
    FB_ERR_DELTA_NEG  = 4,   /* Δτ 出现负值: 加载方向反了 */
    FB_ERR_SPREAD     = 5,   /* 离散度过大: 疑某路卡死/断丝 */
    FB_ERR_MOVE_TIMEOUT = 6, /* 电机未在超时内到达指令位置 (附未到位掩码) */
    FB_ERR_SETTLE_TIMEOUT = 7, /* 信号始终未稳定: 噪声过大/持续漂移/传感器异常 */
} FbErr_t;

/** 超时发生的阶段 (超时帧 [3] 字段) */
typedef enum {
    FB_STAGE_NONE        = 0,
    FB_STAGE_PRE_SETTLE  = 1,
    FB_STAGE_POST_MOVE   = 2,
    FB_STAGE_POST_SETTLE = 3,
} FbStage_t;

/** 最近一次超时发生时「未到位」的电机掩码 (bit i = 电机 i), 0=无 */
uint8_t force_balance_timeout_mask(void);

/** 标定状态 */
uint8_t force_balance_status(void);

/** 标定失败原因 (成功或未运行时为 FB_ERR_NONE) */
uint8_t force_balance_error(void);

/** 启动标定序列 (幂等; 仅首次调用生效) */
void force_balance_start(void);

/** 周期推进标定状态机 (每 10~50ms 调一次) */
void force_balance_tick(uint32_t now_ms);

/** 标定进行中 (此时应暂停感知判据) */
bool force_balance_is_busy(void);

/** 标定已结束(成功或失败) */
bool force_balance_is_done(void);

/** 标定成功 */
bool force_balance_is_ok(void);

/** 最近一次标定测得的 Δτ (N), 供上位机诊断 (全 0 表示未标定) */
const float *force_balance_delta(void);

/** 当前 6 路平衡系数 (只读, 未标定时为 1.0) */
const float *force_balance_gains(void);

#ifdef __cplusplus
}
#endif

#endif /* __FORCE_BALANCE_H */
