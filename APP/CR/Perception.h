/**
 * @file Perception.h
 * @brief 六路肌腱张力感知模块 (比值型)
 *
 * 输入: 6 路压力传感器张力 τ_i (N)  —— global_sensor[i].x
 * 输出: 三层感知信息
 *   L1 绝对残差   : τ_meas - τ_model  —— **仅作诊断**, 不作判据
 *                   (现场: 绝对量受安装/干扰影响大、模型绝对力误差大)
 *   L2 接触感知   : 段内「张力份额」失衡矢量 → 接触有无/所在段/粗定位/力估计
 *   L3 肌腱健康   : 逐路「实测份额/期望份额」比值 → 松弛/断丝/过张/过载
 *
 * 核心思想 (比值型):
 *   段内份额  r_j = τ_j / Στ,  期望份额  w*_j ∝ |cos(φ+α_j)| (仅统计被拉丝)
 *   份额误差  e_j = r_j - w*_j   (无量纲)
 *   截面不平衡矢量 = Σ e_j·[cosα_j, sinα_j]
 *
 *   份额只涉及比值 ⇒ 对传感器增益误差/零点漂移、模型绝对力尺度误差、
 *   温度引起的整体灵敏度漂移全部免疫。仅依赖「通道一致性」。
 *
 * @date 2026-10-03
 */
#ifndef __PERCEPTION_H
#define __PERCEPTION_H

#include <stdint.h>
#include <stdbool.h>
#include "CR.h"              /* SDM_SEGMENTS / SDM_WIRES */
#include "Sensor/Sensor.h"   /* SENSOR_NUM */

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 标定阈值 (按实测调整) ==================== */
/* 接触判据: 截面份额不平衡矢量模长, 无量纲 (理论 0 ~ 1)
 * 标定方法: 空载自由运动时统计该指标的 σ, 阈值取 ≈4σ */
/** 期望份额公式说明 (见 SDM.c sdm_segment_shares) */
#define PERCEPT_CONTACT_THRESH     0.15f
/* 份额不平衡 → 接触力估计的换算系数 (N), 用已知砝码标定 */
#define PERCEPT_CONTACT_GAIN_N    20.0f
/* 健康判据: 实测份额/期望份额 (无量纲, 期望=1) */
#define PERCEPT_ACTIVE_SHARE       0.10f   /* 期望份额低于此值的丝不参与健康判定 */
#define PERCEPT_BROKEN_RATIO       0.30f   /* 份额比 ≤ 此值 判为松弛/断丝 */
#define PERCEPT_DEAD_N              0.5f   /* 实测张力绝对“死区” (N): 区分断丝/松弛 */
#define PERCEPT_OVERTENSION_RATIO  1.80f   /* 份额比 ≥ 此值 判为过张 */
#define PERCEPT_OVERLOAD_N        100.0f   /* 单路张力绝对上限, 与 SDM force_peak_limit 一致 (N) */
#define PERCEPT_MIN_SEG_SUM_N       0.5f   /* 段内超额张力和低于此值则不判定该段 (N) */

/* 逐通道无载基线(预紧力)自适应跟踪:
 *   低于基线 → 立即跟随(捕捉真实下限)
 *   高于基线 → 按此比例缓慢上抬(跟踪温漂/零漂)
 * 采样周期 ~15ms, 0.002/次 ≈ 时间常数 7.5s */
#define PERCEPT_TAU0_RELEASE        0.002f

/* ==================== 数据类型 ==================== */

/** 单路肌腱健康状态 */
typedef enum
{
    PERCEPT_HEALTH_OK           = 0,
    PERCEPT_HEALTH_SLACK        = 1,  /* 松弛/打滑: 份额明显低于期望 */
    PERCEPT_HEALTH_BROKEN       = 2,  /* 断丝/掉线: 期望有份额但实测≈0 */
    PERCEPT_HEALTH_OVERTENSION  = 3,  /* 过张: 份额明显高于期望 */
    PERCEPT_HEALTH_OVERLOAD     = 4,  /* 过载: 超过绝对力上限 */
} PerceptHealth_t;

/** 接触感知结果 */
typedef struct
{
    bool    detected;        /* 是否检测到外部接触/扰动 */
    uint8_t seg;             /* 主导段 (0/1) */
    float   s_norm;          /* 归一化弧长估计 0..1 (粗糙) */
    float   F_seg[2];        /* 主导段截面份额不平衡分量 (无量纲), [0]=x, [1]=y */
    float   F_mag;           /* 接触力估计 (N), 由 F_seg 模长 × 标定系数 → 粗略 */
} PerceptContact_t;

/** 完整感知状态 */
typedef struct
{
    /* --- 判据量 (比值型, 无量纲) --- */
    float   share_meas[SDM_WIRES];    /* 实测段内份额 r_i */
    float   share_des[SDM_WIRES];     /* 期望段内份额 w*_i */
    float   share_err[SDM_WIRES];     /* 份额误差 e_i = r - w* */
    float   disturbance;              /* 主导段不平衡矢量模长 (无量纲) */

    /* --- 诊断量 (绝对, 不作为判据) --- */
    float   force[SDM_WIRES];         /* 实测张力快照 (N) */
    float   f_target[SDM_WIRES];      /* 模型目标张力 (N) */
    float   f_grav[SDM_WIRES];        /* 模型重力分量 (N), 判据前已扣除 */
    float   residual[SDM_WIRES];      /* τ_meas - τ_model (N) */
    float   residual_norm;            /* 残差模长 (N) */
    float   force_mean[SDM_SEGMENTS]; /* 每段平均张力 (N) */

    /* --- 结论 --- */
    PerceptHealth_t health[SDM_WIRES];
    bool    health_any_bad;
    PerceptContact_t contact;
} PerceptState;

/* ==================== 对外 API ==================== */

/**
 * @brief 初始化感知模块 (清零状态与基线)
 */
void perception_init(void);

/**
 * @brief 用给定的 6 路张力执行一次感知更新
 * @param forces 6 路张力 (N, [SDM_WIRES]), 丝序与 SDM 一致
 */
void perception_update(const float forces[SDM_WIRES]);

/**
 * @brief 从 global_sensor[] 取快照并执行感知更新
 *
 * 在临界区内复制 6 个 float, 避免与 DataParserTask 的写入撕裂。
 */
void perception_update_from_global(void);

/**
 * @brief 获取最近一次感知结果 (只读)
 */
const PerceptState *perception_get(void);

/**
 * @brief 采集当前状态作为零点
 *
 * 1) 把当前 6 路张力记为「无载基线」(预紧力), 相当于一次去皮;
 *    之后所有判据都使用「超额张力 τ - τ0」, 因此对通道零点偏置免疫。
 * 2) 同时记录绝对残差基线, 仅用于让诊断量 residual 显示更直观。
 *
 * 请在臂体自由、无接触时调用一次(也可省略: τ0 会自适应跟踪)。
 */
void perception_capture_baseline(void);

#ifdef __cplusplus
}
#endif

#endif /* __PERCEPTION_H */
