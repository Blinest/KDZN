/**
 * @file SensorFilter.c
 * @brief 传感器软件滤波器实现（指数移动平均 EMA）
 *
 * filter = filter * (DEN-NUM)/DEN + new_val * NUM/DEN
 * 整数运算，无浮点开销
 *
 * @date 2026-07-02
 * @author blin
 */

#include "SensorFilter.h"

void SensorFilter_Init(SensorFilter *f)
{
    f->filter_val   = 0;
    f->first_sample = 1;
}

int32_t SensorFilter_Update(SensorFilter *f, int32_t new_val)
{
    if (f->first_sample) {
        f->filter_val   = new_val;
        f->first_sample = 0;
    } else {
        /* EMA: filter = filter * 4/5 + new_val * 1/5 */
        f->filter_val = (f->filter_val * (FILTER_ALPHA_DEN - FILTER_ALPHA_NUM)
                       + new_val * FILTER_ALPHA_NUM) / FILTER_ALPHA_DEN;
    }
    return f->filter_val;
}

void SensorFilter_Reset(SensorFilter *f)
{
    f->first_sample = 1;
}
