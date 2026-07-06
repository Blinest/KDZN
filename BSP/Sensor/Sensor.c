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
#include "Storage/FlashStorage.h"
#include "CR/CR.h"

// 初始化全局传感器数组
GlobalSensor global_sensor[SENSOR_NUM];


/**
 * @brief 传感器初始化函数
 *
 * 初始化所有传感器，包括：
 * 1. 清空传感器数据结构体
 * 2. 初始化 CMCU-06 压力传感器
 * 3. 从 Flash 恢复断电前的运行参数
 */
void sensor_init(void)
{
    // 1. 清空传感器数据结构
    for (int i = 0; i < SENSOR_NUM; i++) {
        global_sensor[i].press_sensor.raw_val = 0;
        global_sensor[i].press_sensor.filter_val = 0;
        global_sensor[i].press_sensor.val = 0;
        global_sensor[i].press_sensor.sensitivity_scale = 1.0f;
    }

    // 2. 初始化 CMCU-06 传感器硬件
    CMCU_06_Init();

    // 3. 从 Flash 恢复断电前的参数（滤波器状态 + 电机控制状态）
    param_load();
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
 * @brief 传感器数据置零
 *
 * 不对传感器进行硬件复位，仅对传感器进行硬件置零。
 */
void sensor_reset(void)
{
    for (int i = 1; i <= SENSOR_NUM; i++)
    {
        CMCU_06_Write_Protect(i, false);  // 1. 关闭写入保护
        HAL_Delay(1000);
        CMCU_06_ResetPins(i);             // 2. 去皮置零
        HAL_Delay(1000);
        CMCU_06_Write_Protect(i, true);   // 3. 重新打开写入保护
        HAL_Delay(1000);
    }

    /* 等待 DataTask 完成一轮传感器读取，确保传感器新值已更新到缓存 */
    HAL_Delay(1000);

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

/**
 * @brief 保存当前运行参数到内部 Flash
 *
 * 收集滤波器状态 + 电机控制状态，写入 Bank2 Sector7。
 * 阻塞操作（擦除+写入约数十毫秒），在 RTOS 任务中调用时需注意。
 */
void param_save(void)
{
    FlashStoreData data;
    memset(&data, 0, sizeof(data));

    data.magic = FLASH_STORE_MAGIC;

    /* 导出 EMA 滤波器状态 */
    CMCU_06_Filter_Export(data.filter_first_sample, data.filter_val);

    /* 导出电机压力控制状态 */
    CR_pressure_export(data.motor_target, data.prev_val);

    /* 导出压力灵敏度归一化系数 */
    for (int i = 0; i < SENSOR_NUM; i++) {
        data.sensitivity_scale[i] = global_sensor[i].press_sensor.sensitivity_scale;
    }

    FlashStorage_Save(&data);
}

/**
 * @brief 从内部 Flash 恢复运行参数
 *
 * 启动时调用。若 Flash 中无有效数据（首次上电），跳过恢复。
 */
void param_load(void)
{
    FlashStoreData data;

    if (FlashStorage_Load(&data) == HAL_OK) {
        /* 恢复 EMA 滤波器状态 */
        CMCU_06_Filter_Restore(data.filter_first_sample, data.filter_val);

        /* 恢复电机压力控制状态 */
        CR_pressure_restore(data.motor_target, data.prev_val);

        /* 恢复压力灵敏度归一化系数 */
        for (int i = 0; i < SENSOR_NUM; i++) {
            if (data.sensitivity_scale[i] > 0.0f)
                global_sensor[i].press_sensor.sensitivity_scale = data.sensitivity_scale[i];
        }
    }
    /* 若加载失败（magic 不匹配），不做任何处理，使用默认初始化值 */
}