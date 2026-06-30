/**
 * @file Sensor.c
 * @brief 传感器驱动函数实现
 *
 * 实现传感器初始化、数据读取、自检等功能
 * 使用 CMCU-06 压力传感器（USART1，Modbus-RTU协议）
 *
 * @date 2026-04-02
 * @author blin
 */


#include "Sensor.h"
#include <stdio.h>

#include "CMCU-06.h"
#include "IMU.h"
#include "cmsis_os2.h"

// 初始化全局传感器数组
GlobalSensor global_sensor[SENSOR_NUM];


/**
 * @brief 传感器初始化函数
 *
 * 初始化所有传感器，包括：
 * 1. 清空传感器数据结构体
 * 2. 初始化 CMCU-06 压力传感器
 */
void sensor_init(void)
{
    // 1. 清空传感器数据结构
    for (int i = 0; i < SENSOR_NUM; i++) {
        global_sensor[i].press_sensor.raw_val = 10.0f;
        global_sensor[i].press_sensor.filter_val = 0.0f;
        global_sensor[i].press_sensor.val = 0.0f;
    }

    // 2. 初始化 CMCU-06 传感器硬件
    CMCU_06_Init();
}
/**
 * @brief 单传感器数据读取函数
 * @param sensor_id 传感器ID (1-6)
 *
 * 读取指定传感器的数据并更新全局结构体
 */
void sensor_single_read(uint8_t sensor_id)
{
    // 压力传感器数据读取
    CMCU_06_single_read(sensor_id);
}

/**
 * @brief 多传感器数据读取函数
 *
 * 批量读取所有 6 个压力传感器的数据
 */
void sensor_multi_read(void)
{
    for (int i = 0; i < SENSOR_NUM; i++) {
        CMCU_06_single_read(i + 1);
        osDelay(20);  // 发送8ms + 响应9ms，15ms 留 ~4ms 余量
    }
}

/**
 * @brief 传感器校准函数
 * @param sensor_id 传感器ID (1-6)
 * @param weight_10x 砝码重量值 (g×10, 例如 500g → 5000)
 */
void sensor_cal(uint8_t sensor_id, uint16_t weight_10x)
{
    if (sensor_id < 1 || sensor_id > SENSOR_NUM) {
        return;
    }
    CMCU_06_Cal(sensor_id, weight_10x);
}