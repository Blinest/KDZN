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
 * @brief 单条 Modbus-RTU 响应等待超时 (ms)
 *
 * 6 个 CMCU-06 共用 USART1 @9600bps，8 字节请求 + 9 字节响应单次往返约 20ms，
 * 取 30ms 作为"未响应则跳过该地址"的上限。
 */
#define SENSOR_RESP_TIMEOUT_MS  30

/**
 * @brief 全局传感器数据结构
 */
typedef struct
{
	float x;  /**< 滤波+通道平衡后的张力 (N), 供控制/感知使用 */
	float y;  /**< 原始张力 (N), 供诊断对比 */
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
 *
 * 6 个 CMCU-06 共用 USART1(Modbus-RTU) 总线, 必须串行轮询:
 *   发一条请求 -> 等本地址响应到达(或超时) -> 再发下一条,
 * 否则多个从机会同时应答造成总线碰撞。
 *
 * 注意: 本函数为阻塞式(最多约 SENSOR_NUM * SENSOR_RESP_TIMEOUT_MS),
 *       仅供手动/调试指令使用; 连续采集请用 sensor_poll_step()。
 */
void sensor_multi_read(void);

/**
 * @brief 传感器轮询「单步」(非阻塞)
 *
 * 每次调用最多向 1 个地址发一Modbus读请求, 并等上一条响应到达/超时后才推进,
 * 需周期性调用(建议 10~20ms)。6 路全部轮一遍约 100~200ms。
 *
 * @param timeout_ms 距上次请求超过该时间仍未收到响应则强制进入下一地址
 * @return 本次实际发起请求的传感器ID(1-6); 0 表示本次仍在等待上一条响应
 */
uint8_t sensor_poll_step(uint32_t timeout_ms);

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