/**
 * @file ForceFilter.c
 * @brief 六路肌腱张力信号调理实现
 *
 * @date 2026-10-03
 */

#include "ForceFilter.h"
#include <math.h>
#include <string.h>

typedef struct {
    float  win[FORCE_MED_WIN];   /* 滑动窗口, win[0] 最新 */
    uint8_t n;                   /* 已填充样本数 */
    float  y;                    /* IIR 输出 */
    float  last_raw;             /* 上次原始值 (尖峰判据) */
    bool   started;              /* 是否已有首个样本 */
} ForceCh_t;

static ForceCh_t s_ch[SENSOR_NUM];

/* 通道平衡系数 (1.0 = 不修正) */
static float s_balance[SENSOR_NUM];

void force_filter_init(void)
{
    memset(s_ch, 0, sizeof(s_ch));
    for (uint8_t i = 0; i < SENSOR_NUM; i++) s_balance[i] = 1.0f;
}

/* 取窗口前 n 个样本的中值 (n <= FORCE_MED_WIN) */
static float _median(const float *v, uint8_t n)
{
    float t[FORCE_MED_WIN];
    for (uint8_t i = 0; i < n; i++) t[i] = v[i];

    /* 插入排序 (n<=3) */
    for (uint8_t i = 1; i < n; i++) {
        float key = t[i];
        int j = (int)i - 1;
        while (j >= 0 && t[j] > key) { t[j + 1] = t[j]; j--; }
        t[j + 1] = key;
    }

    if (n == 0) return 0.0f;
    if (n == 1) return t[0];
    if (n == 2) return 0.5f * (t[0] + t[1]);
    return t[1];   /* n==3 取中位 */
}

float force_filter_update(uint8_t ch, float raw)
{
    if (ch >= SENSOR_NUM) return raw;

    ForceCh_t *p = &s_ch[ch];

    /* 1) 尖峰抑制: 与上次原始值比较, 跳变过大则丢弃本次 */
    if (p->started && fabsf(raw - p->last_raw) > FORCE_SPIKE_N) {
        raw = p->last_raw;
    }
    p->last_raw = raw;
    p->started  = true;

    /* 2) 滑动中值: 新的插到最前 */
    for (int i = FORCE_MED_WIN - 1; i > 0; i--) {
        p->win[i] = p->win[i - 1];
    }
    p->win[0] = raw;
    if (p->n < FORCE_MED_WIN) p->n++;

    const float med = _median(p->win, p->n);

    /* 3) 一阶 IIR; 首个样本直接作为初值, 避免从 0 缓慢爬升 */
    if (p->n == 1) {
        p->y = med;
    } else {
        p->y += FORCE_IIR_ALPHA * (med - p->y);
    }

    return p->y * s_balance[ch];
}

float force_filter_get(uint8_t ch)
{
    if (ch >= SENSOR_NUM) return 0.0f;
    return s_ch[ch].y * s_balance[ch];
}

void force_filter_get_all(float out[SENSOR_NUM])
{
    if (out == NULL) return;
    for (uint8_t i = 0; i < SENSOR_NUM; i++) {
        out[i] = s_ch[i].y * s_balance[i];
    }
}

/* ==================== 通道平衡系数 ==================== */

void force_filter_set_balance(uint8_t ch, float gain)
{
    if (ch >= SENSOR_NUM || gain <= 0.0f) return;
    s_balance[ch] = gain;
}

float force_filter_get_balance(uint8_t ch)
{
    if (ch >= SENSOR_NUM) return 1.0f;
    return s_balance[ch];
}

void force_filter_get_balance_all(float out[SENSOR_NUM])
{
    if (out == NULL) return;
    for (uint8_t i = 0; i < SENSOR_NUM; i++) {
        out[i] = s_balance[i];
    }
}

void force_filter_reset_balance(void)
{
    for (uint8_t i = 0; i < SENSOR_NUM; i++) s_balance[i] = 1.0f;
}
