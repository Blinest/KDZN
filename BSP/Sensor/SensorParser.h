//
// Created by q3634 on 2026/6/24.
//

#ifndef CONTROLSYSTEM_STM32H7_SENSORPARSER_H
#define CONTROLSYSTEM_STM32H7_SENSORPARSER_H
/**
* @file SensorParser.h
 * @brief 传感器解析器类型定义
 *
 * 该文件定义 IMU 和 CMCU 传感器解析器相关的类型，供 Sensor 和 parser 模块共用。
 */


#include <stdint.h>

/**
 * @brief IMU 传感器解析状态机状态
 */
typedef enum {
	SENSOR_STATE_HEAD = 0,
	SENSOR_STATE_FUNC,
	SENSOR_STATE_LEN,
	SENSOR_STATE_DATA,
	SENSOR_STATE_CHECK
} SensorParseState_t;

/**
 * @brief IMU 协议解析器结构体
 */
typedef struct {
	SensorParseState_t state;
	uint8_t buf[32];        /**< 接收缓冲区 */
	uint8_t data_len;       /**< 解析数据长度 */
	uint8_t idx;            /**< 当前写入位置 */
} IMUParser;

/**
 * @brief CMCU 压力传感器解析状态机状态
 */
typedef enum {
	CMCU_STATE_HEAD = 0,
	CMCU_STATE_FUNC,
	CMCU_STATE_LEN,
	CMCU_STATE_DATA,
	CMCU_STATE_CRC1,
	CMCU_STATE_CRC2,
	CMCU_STATE_DONE
} CMCU_ParserState;

/**
 * @brief CMCU 解析器结构体
 */
typedef struct {
	CMCU_ParserState state;
	uint8_t buf[32];         /**< 存储帧数据 */
	uint8_t idx;             /**< 当前写入位置 */
	uint8_t data_len;        /**< 数据域长度 */
	uint16_t crc_calc;       /**< 计算出的 CRC */
	uint16_t crc_recv;       /**< 接收到的 CRC */
} CMCU_Parser;


#endif //CONTROLSYSTEM_STM32H7_SENSORPARSER_H