//
// Created by blin on 2026/4/30.
//

#ifndef CONTROLSYSTEM_CMCU_06_H
#define CONTROLSYSTEM_CMCU_06_H
#include <stdbool.h>
#include <stdint.h>

// 初始化CMCU-06
void CMCU_06_Init(void);

void CMCU_06_Write_Protect(uint8_t addr, bool state);


void CMCU_06_Reset(uint8_t addr);

void CMCU_06_ResetPins(uint8_t addr);

/**
 * @brief 写入砝码校准值
 * @param addr       传感器地址 (1-6)
 * @param weight_10x 砝码重量值（单位：g，×10倍存储，例如 500g → 5000）
 */
void CMCU_06_WriteCalWeight(uint8_t addr, uint16_t weight_10x);

/**
 * @brief 砝码校准完整流程
 * @param addr       传感器地址 (1-6)
 * @param weight_10x 砝码重量值 (g×10, 例如 500g → 5000)
 */
void CMCU_06_Cal(uint8_t addr, uint16_t weight_10x);

void CMCU_06_single_read(uint8_t addr);

/**
 * @brief 逐字节解析CMCU-06 Modbus-RTU响应
 * @param byte USART1接收到的字节
 */
void CMCU_06_Parse_Byte(uint8_t byte);

/**
 * @brief 复位Modbus-RTU解析状态机
 */
void CMCU_06_Parse_Reset(void);

static uint16_t CRC16_Modbus(uint8_t *buf, uint8_t len);
#endif //CONTROLSYSTEM_CMCU_06_H
