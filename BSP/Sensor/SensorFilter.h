/**
 * @file SensorFilter.h
 * @brief 传感器软件滤波器（指数移动平均 EMA）
 *
 * alpha = 1/5，整数运算，嵌入式友好
 * 首次采样直接初始化，校准后可重置
 *
 * @date 2026-07-02
 * @author blin
 */

#ifndef CONTROLSYSTEM_SENSORFILTER_H
#define CONTROLSYSTEM_SENSORFILTER_H

#include <stdint.h>

#define FILTER_ALPHA_NUM  1
#define FILTER_ALPHA_DEN  5

/**
 * @brief EMA 滤波器状态（每传感器一个实例）
 */
typedef struct {
    int32_t filter_val;     /**< 当前滤波输出 */
    uint8_t first_sample;   /**< 1=等待首次采样，用 val 直接初始化 */
} SensorFilter;

/**
 * @brief 初始化滤波器（清零，标记等待首次采样）
 */
void SensorFilter_Init(SensorFilter *f);

/**
 * @brief 输入新采样值，返回滤波输出
 * @param f         滤波器实例
 * @param new_val   本次原始采样值（int32_t）
 * @return 滤波后的输出值
 */
int32_t SensorFilter_Update(SensorFilter *f, int32_t new_val);

/**
 * @brief 重置滤波器（校准后调用，下次采样重新初始化）
 */
void SensorFilter_Reset(SensorFilter *f);

#endif // CONTROLSYSTEM_SENSORFILTER_H
