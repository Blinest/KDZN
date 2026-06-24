/**
* @file periph_cmd_parser.h
 * @brief 下位机指令解析库头文件
 *
 * 负责解析外设（电机/传感器）反馈的指令，更新全局结构体
 *
 * @date 2026-03-30
 * @author blin
 */

#ifndef __PERIPH_CMD_PARSER_H
#define __PERIPH_CMD_PARSER_H

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif


	/**
	 * @brief 下位机指令解析函数 (Peripheral -> STM32)
	 * @param byte 接收到的单字节数据
	 */
	void sensor_data_parser_feed_byte(uint8_t byte);
	void parse_sensor_feedback_data(void);
#ifdef __cplusplus
}
#endif

#endif /* __PERIPH_CMD_PARSER_H */