/**
* @file pc_cmd_parser.h
 * @brief 上位机指令解析库头文件
 *
 * 负责解析上位机发送的指令，更新全局结构体，并分发给下位机
 *
 * @date 2026-03-30
 * @author blin
 */

#ifndef __PC_CMD_PARSER_H
#define __PC_CMD_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	/**
	 * @brief 上位机指令解析函数 (PC -> STM32)
	 * @param byte 接收到的单字节数据
	 */
	void pc_cmd_parser_feed_byte(uint8_t byte);

	/**
	 * @brief 重置上位机指令解析器
	 */
	void pc_cmd_parser_reset_all(void);

#ifdef __cplusplus
}
#endif

#endif /* __PC_CMD_PARSER_H */