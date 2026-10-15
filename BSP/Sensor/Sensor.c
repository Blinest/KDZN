/**
 * @file Sensor.c
 * @brief 传感器驱动函数实现
 *
 * 实现传感器初始化、数据读取、自检等功能
 * 使用 CMCU-06 压力传感器（USART2，Modbus-RTU协议）
 *
 * @date 2026-04-02
 * @author blin
 */


#include "Sensor.h"
#include <stdio.h>

#include "cmsis_os2.h"
#include "CMCU-06.h"
#include "IMU.h"
#include "ForceFilter.h"


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
        global_sensor[i].x = 0.0f;
        global_sensor[i].y = 0.0f;
    }

    // 2. 初始化 CMCU-06 传感器硬件
    CMCU_06_Init();

    // 3. 初始化张力信号调理 (尖峰抑制 + 中值 + 低通)
    force_filter_init();
}
/**
 * @brief 单传感器数据读取函数
 * @param sensor_id 传感器ID (1-6)
 *
 * 读取指定传感器的数据并更新全局结构体
 */
void sensor_single_read(uint8_t sensor_id)
{
    if (sensor_id < 1 || sensor_id > SENSOR_NUM) {
        return;
    }
    // 压力传感器数据读取（先清握手标志, 便于判断本次响应是否到达）
    CMCU_06_Clear_Rx_Flag();
    CMCU_06_single_read(sensor_id);
}

/**
 * @brief 多传感器数据读取函数
 *
 * 6 个 CMCU-06 共用 USART2(Modbus-RTU) 总线, 串行轮询:
 *   发一条请求 -> 等本地址响应(或超时) -> 再发下一条, 避免多机响应碰撞。
 */
void sensor_multi_read(void)
{
    for (uint8_t addr = 1; addr <= SENSOR_NUM; addr++)
    {
        CMCU_06_Clear_Rx_Flag();
        CMCU_06_single_read(addr);

        const uint32_t t0 = osKernelGetTickCount();
        while (CMCU_06_Last_Rx_Addr() != addr &&
               (osKernelGetTickCount() - t0) < SENSOR_RESP_TIMEOUT_MS)
        {
            osDelay(1);   /* 让出 CPU, 使 DataParserTask 能解析到达的响应 */
        }
    }
}

/**
 * @brief 传感器轮询单步(非阻塞), 供周期性采集任务调用
 */
uint8_t sensor_poll_step(uint32_t timeout_ms)
{
    static uint8_t  s_addr = 0;   /* 0 = 尚未开始 */
    static uint32_t s_tick = 0;

    const uint32_t now = osKernelGetTickCount();

    /* 上一条请求尚未收到响应且未超时 -> 继续等待, 本次不发 */
    if (s_addr != 0 &&
        CMCU_06_Last_Rx_Addr() != s_addr &&
        (now - s_tick) < timeout_ms)
    {
        return 0;
    }

    /* 推进到下一地址并发起读取 */
    s_addr = (s_addr >= SENSOR_NUM) ? 1 : (uint8_t)(s_addr + 1);
    CMCU_06_Clear_Rx_Flag();
    CMCU_06_single_read(s_addr);
    s_tick = now;

    return s_addr;
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