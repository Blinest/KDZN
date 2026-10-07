/**
 * @file ForceBalance.c
 * @brief 上电自动通道平衡标定 (差分法) 实现
 *
 * 差分原理见 ForceBalance.h。
 *
 * SETTLE 阶段 = 判稳 + 取均值 (二合一):
 *   参考锚定在「本轮稳定窗口起点」, 逐步累积窗口内样本;
 *   只要窗口内最大偏离 < FB_SETTLE_TOL_N 且窗口时长 >= FB_SETTLE_STABLE_MS,
 *   即判稳定, 并直接返回窗口均值 —— 不再额外做固定窗口平均。
 *   偏离超限则重锚参考、清空累计、重新计时。
 *   FB_SETTLE_TIMEOUT_MS 兜底, 用已积累的样本放行, 不会卡死。
 *
 * @date 2026-10-03
 */

#include "ForceBalance.h"
#include "Sensor/ForceFilter.h"
#include "Motor/Motor.h"
#include "Communicate/cmd_packer.h"
#include "usart.h"

#include <math.h>
#include <string.h>

typedef enum {
    FB_IDLE = 0,
    FB_PRE_SETTLE,    /* 判稳+取均值: 拉之前 (含预紧+重力) */
    FB_POST_MOVE,     /* 下发加载位移, 等电机确实响应 */
    FB_POST_SETTLE,   /* 判稳+取均值: 拉之后 */
    FB_APPLY,         /* 差分 → 系数 → 归零 */
    FB_DONE,
    FB_FAIL,
} FbState_t;

static FbState_t s_state   = FB_IDLE;
static bool      s_started = false;
static uint32_t  s_movet0  = 0;           /* 加载指令下发时刻 */
static float     s_pos0[MOTOR_NUM];       /* 下发指令时刻的电机位置基准 (mm) */
static uint32_t  s_pos_tlast = 0;         /* 上次读电机位置时刻 */
static uint8_t   s_to_mask = 0;           /* 最近一次超时: 未到位电机掩码 */
static bool      s_no_resp = false;       /* POST_MOVE 是否超时(电机未到位) */
static uint8_t   s_err     = FB_ERR_NONE;

/* --- 稳定窗口 = 采样窗口 --- */
static float     s_ref[SENSOR_NUM];       /* 锚定参考 */
static bool      s_ref_valid = false;
static float     s_acc[SENSOR_NUM];       /* 窗口累积 */
static uint32_t  s_n         = 0;
static uint32_t  s_stable_since = 0;      /* 本轮稳定窗口起点 */
static uint32_t  s_settle_t0    = 0;      /* 本阶段起点(超时用) */
static uint32_t  s_settle_tlast = 0;      /* 上次采样时刻 */

/* --- 结果 --- */
static float     s_pre[SENSOR_NUM];
static float     s_post[SENSOR_NUM];
static float     s_delta[SENSOR_NUM];
static float     s_gain[SENSOR_NUM];

/* ==================== 内部 ==================== */

static void _motor_set_all(float mm)
{
    float d[SENSOR_NUM];
    for (int i = 0; i < SENSOR_NUM; i++) d[i] = mm;
    motor_sync_control(SENSOR_NUM, 0, d);
}

/**
 * @brief 向上位机主动上报「标定超时帧」(0xBB 0x04)
 */
static void _notify_timeout(uint8_t stage, uint8_t err, uint8_t mask)
{
    uint8_t f[8];
    const uint16_t n = cmd_packer_pack_fb_timeout_frame(f, stage, err, mask);
    if (n > 0 && n <= (uint16_t)sizeof(f)) {
        Usart_SendString(&huart2, f, n);
    }
}

static void _reset_window(const float f[SENSOR_NUM], uint32_t now)
{
    for (int i = 0; i < SENSOR_NUM; i++) {
        s_ref[i] = f[i];
        s_acc[i] = f[i];
    }
    s_ref_valid    = true;
    s_n            = 1;
    s_stable_since = now;
}

static void _flush_mean(float out[SENSOR_NUM])
{
    for (int i = 0; i < SENSOR_NUM; i++) {
        out[i] = (s_n > 0) ? (s_acc[i] / (float)s_n) : 0.0f;
    }
}

static void _settle_begin(uint32_t now)
{
    s_ref_valid    = false;
    s_n            = 0;
    s_stable_since = now;
    s_settle_t0    = now;
    s_settle_tlast = 0;
}

/**
 * @brief [判稳 + 取均值] 二合一
 * @param mean_out   稳定时输出本窗口的 6 路均值
 * @param timed_out  输出: true 表示是因超时放行(信号始终未稳定), 均值不可信
 * @return true = 可以继续 (已稳定 或 超时)
 */
static bool _settle_check(uint32_t now, float mean_out[SENSOR_NUM], bool *timed_out)
{
    if (timed_out) *timed_out = false;

    /* 阶段起点延迟到首次 tick 填入 (start 时 s_settle_t0 置 0) */
    if (s_settle_t0 == 0) {
        s_settle_t0    = now;
        s_settle_tlast = 0;
        s_ref_valid    = false;
        s_n            = 0;
        s_stable_since = now;
    }

    /* 兜底: 超时放行, 用已积累的样本, 但标记为不可信 */
    if ((now - s_settle_t0) >= FB_SETTLE_TIMEOUT_MS) {
        if (s_n == 0) {
            float f[SENSOR_NUM];
            force_filter_get_all(f);
            _reset_window(f, now);
        }
        _flush_mean(mean_out);
        if (timed_out) *timed_out = true;
        return true;
    }

    if (s_settle_tlast != 0 && (now - s_settle_tlast) < FB_SETTLE_POLL_MS) {
        return false;
    }
    s_settle_tlast = now;

    float f[SENSOR_NUM];
    force_filter_get_all(f);

    if (!s_ref_valid) {
        _reset_window(f, now);
        return false;
    }

    float dmax = 0.0f;
    for (int i = 0; i < SENSOR_NUM; i++) {
        const float d = fabsf(f[i] - s_ref[i]);   /* 相对锚定参考, 非逐次差分 */
        if (d > dmax) dmax = d;
    }

    if (dmax <= FB_SETTLE_TOL_N) {
        /* 稳定: 继续累积, 达到窗口长度即判稳并输出均值 */
        for (int i = 0; i < SENSOR_NUM; i++) s_acc[i] += f[i];
        s_n++;
        if ((now - s_stable_since) >= FB_SETTLE_STABLE_MS) {
            _flush_mean(mean_out);
            return true;
        }
    } else {
        /* 偏离: 重锚参考 + 清空累计 + 重新计时 */
        _reset_window(f, now);
    }
    return false;
}

/* ==================== 对外 API ==================== */

void force_balance_start(void)
{
    if (s_started) return;
    s_started = true;

    for (int i = 0; i < SENSOR_NUM; i++) {
        s_gain[i]  = 1.0f;
        s_delta[i] = 0.0f;
        s_pre[i]   = 0.0f;
        s_post[i]  = 0.0f;
    }

#if FB_CAL_ENABLE
    s_no_resp = false;
    s_err     = FB_ERR_NONE;
    _settle_begin(0);
    s_state = FB_PRE_SETTLE;
#else
    s_err   = FB_ERR_DISABLED;
    s_state = FB_DONE;
#endif
}

void force_balance_tick(uint32_t now_ms)
{
    switch (s_state)
    {
    case FB_PRE_SETTLE: {
        bool to = false;
        if (!_settle_check(now_ms, s_pre, &to)) break;

        if (to) {
            /* 上电后信号始终未稳定: 噪声过大/持续漂移/传感器异常 → 拒标 */
            s_err = FB_ERR_SETTLE_TIMEOUT;
            _notify_timeout(FB_STAGE_PRE_SETTLE, s_err, 0);
            s_state = FB_FAIL;
            break;
        }

        if (FB_PULL_MM != 0.0f) {
            /* 先刷新一次位置并记作基准, 再下发加载指令 */
            motor_read_positions();
            for (int i = 0; i < MOTOR_NUM; i++) {
                s_pos0[i] = global_motor[i].stepper_motor.current_pos;
            }
            _motor_set_all(FB_PULL_MM);

            s_movet0    = now_ms;
            s_pos_tlast = now_ms;
            s_to_mask   = 0;
            s_state     = FB_POST_MOVE;   /* 必须确认 6 路到位才继续 */
        } else {
            _settle_begin(now_ms);
            s_state = FB_POST_SETTLE;
        }
        break;
    }

    case FB_POST_MOVE: {
        /* 周期性读取 6 路电机位置 */
        if ((now_ms - s_pos_tlast) >= FB_POS_POLL_MS) {
            s_pos_tlast = now_ms;
            motor_read_positions();
        }

        const uint8_t mask = motor_reached_mask(s_pos0, FB_PULL_MM,
                                                FB_POS_TOL_MM, FB_VEL_TOL_MM_S);

        if (mask == 0) {
            /* 6 路全部到位且停稳 → 进入判稳 */
            _settle_begin(now_ms);
            s_state = FB_POST_SETTLE;
            break;
        }

        if ((now_ms - s_movet0) >= FB_MOVE_TIMEOUT_MS) {
            /* 超时: 上报超时帧, 归零, 判失败 */
            s_to_mask = mask;
            s_no_resp = true;
            s_err     = FB_ERR_MOVE_TIMEOUT;
            _notify_timeout(FB_STAGE_POST_MOVE, s_err, mask);
            _motor_set_all(0.0f);
            s_state = FB_FAIL;
        }
        break;
    }

    case FB_POST_SETTLE: {
        bool to = false;
        if (!_settle_check(now_ms, s_post, &to)) break;

        if (to) {
            /* 拉动后信号始终未稳定 ⇒ 该窗口均值不可信, 会算出错误系数 → 拒标 */
            s_err = FB_ERR_SETTLE_TIMEOUT;
            _notify_timeout(FB_STAGE_POST_SETTLE, s_err, 0);
            _motor_set_all(0.0f);
            s_state = FB_FAIL;
            break;
        }
        s_state = FB_APPLY;
        break;
    }

    case FB_APPLY: {
        /* --- 1. 差分: Δτ = τ_post - τ_pre (预紧 P 与重力 G 自动抵消) --- */
        float dmin = 1e30f, dmax = -1e30f, dsum = 0.0f;

        for (int i = 0; i < SENSOR_NUM; i++) {
            const float d = s_post[i] - s_pre[i];
            s_delta[i] = d;
            if (d < dmin) dmin = d;
            if (d > dmax) dmax = d;
            dsum += d;
        }
        const float dmean = dsum / (float)SENSOR_NUM;

        /* --- 2. 校验: 必须各丝同向增大, 且离散度可接受 --- */
        bool ok = true;
        if (s_no_resp) {
            s_err = FB_ERR_NO_RESPONSE;      /* 优先报"电机没动" */
            ok = false;
        } else if (dmean < FB_MIN_DELTA_N || dmin > 0.0f) {
            /* dmin<0 ⇒ 有通道负增长 ⇒ 方向反了; 否则是增量不足 */
            s_err = (dmin < 0.0f) ? FB_ERR_DELTA_NEG : FB_ERR_DELTA_SMALL;
            ok = false;
        } else if (dmin < FB_MIN_DELTA_N) {
            s_err = FB_ERR_DELTA_SMALL;
            ok = false;
        } else if (dmax > dmin * FB_SPREAD_LIMIT) {
            s_err = FB_ERR_SPREAD;
            ok = false;
        }

        if (!ok) {
            _motor_set_all(0.0f);
            s_state = FB_FAIL;
            break;
        }

        /* --- 3. g_i = mean(Δτ)/Δτ_i, 限幅后整体归一化 --- */
        float g_tmp[SENSOR_NUM];
        float gsum = 0.0f;
        for (int i = 0; i < SENSOR_NUM; i++) {
            float g = dmean / s_delta[i];
            if (g < FB_GAIN_MIN) g = FB_GAIN_MIN;
            if (g > FB_GAIN_MAX) g = FB_GAIN_MAX;
            g_tmp[i] = g;
            gsum += g;
        }
        const float gmean = gsum / (float)SENSOR_NUM;

        /* --- 4. 迭代累积 (支持多轮收敛) --- */
        if (gmean > 1e-6f) {
            for (int i = 0; i < SENSOR_NUM; i++) {
                float g = force_filter_get_balance(i) * g_tmp[i] / gmean;
                force_filter_set_balance(i, g);
                s_gain[i] = g;
            }
        }

        /* --- 5. 归零并结束 --- */
        _motor_set_all(0.0f);
        s_state = FB_DONE;
        break;
    }

    default:
        break;
    }
}

bool force_balance_is_busy(void)
{
    return (s_state == FB_PRE_SETTLE) || (s_state == FB_POST_MOVE) ||
           (s_state == FB_POST_SETTLE) || (s_state == FB_APPLY);
}

bool force_balance_is_done(void)
{
    return (s_state == FB_DONE) || (s_state == FB_FAIL);
}

bool force_balance_is_ok(void)
{
    return (s_state == FB_DONE);
}

const float *force_balance_delta(void)
{
    return s_delta;
}

const float *force_balance_gains(void)
{
    return s_gain;
}

uint8_t force_balance_status(void)
{
    switch (s_state) {
        case FB_DONE: return (uint8_t)FB_ST_OK;
        case FB_FAIL: return (uint8_t)FB_ST_FAIL;
        case FB_IDLE: return (uint8_t)FB_ST_IDLE;
        default:      return (uint8_t)FB_ST_BUSY;
    }
}

uint8_t force_balance_error(void)
{
    return s_err;
}

uint8_t force_balance_timeout_mask(void)
{
    return s_to_mask;
}
