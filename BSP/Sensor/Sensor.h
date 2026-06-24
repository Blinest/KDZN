/**
* @file Sensor.h
 * @brief 传感器驱动头文件
 *
 * 定义传感器数据结构和函数接口
 *
 * @date 2026-04-02
 * @author Psyduck
 */

#ifndef CONTROLSYSTEM_SENSOR_H
#define CONTROLSYSTEM_SENSOR_H

#include <stdint.h>

#define SENSOR_NUM 6

/**
 * @brief 全局传感器数据结构
 */
typedef struct
{
	float x;  /**< X轴数据 */
	float y;  /**< Y轴数据 */
	float z;  /**< Z轴数据 */
} GlobalSensor;

/**
 * @brief 传感器初始化函数
 */
void sensor_init(void);

/**
 * @brief 单传感器数据读取函数
 * @param sensor_id 传感器ID (1-6)
 */
void sensor_single_read(uint8_t sensor_id);

/**
 * @brief 多传感器数据读取函数
 */
void sensor_multi_read(void);

/**
 * @brief 传感器自检函数
 */
void sensor_self_test(uint8_t sensor_id);

/**
 * @brief 获取传感器原始数据
 * @param sensor_id 传感器ID
 * @param x 指向X轴数据的指针
 * @param y 指向Y轴数据的指针
 * @param z 指向Z轴数据的指针
 */
void sensor_get_raw_data(uint8_t sensor_id, int16_t* x, int16_t* y, int16_t* z);

/**
 * @brief 获取传感器角度数据
 * @param sensor_id 传感器ID
 * @param pitch 指向俯仰角的指针
 * @param roll 指向横滚角的指针
 * @param yaw 指向偏航角的指针
 */
void sensor_get_angle_data(uint8_t sensor_id, float* pitch, float* roll, float* yaw);
/**
 * @brief 传感器校准
 * @param sensor_id 传感器ID (1-6)
 * @param weight_10x 砝码重量值 (g×10, 例如 500g → 5000)
 */
void sensor_cal(uint8_t sensor_id, uint16_t weight_10x);
// 全局传感器数组声明
extern GlobalSensor global_sensor[SENSOR_NUM];

#endif //CONTROLSYSTEM_SENSOR_H