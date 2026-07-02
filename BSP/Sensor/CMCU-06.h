/**
 * @file CMCU-06.h
 * @brief CMCU-06 压力传感器 Modbus-RTU 驱动接口
 *
 * @date 2026-04-30
 * @author blin
 */

#ifndef CONTROLSYSTEM_CMCU_06_H
#define CONTROLSYSTEM_CMCU_06_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化 CMCU-06 传感器（关闭写保护 → 复位 → 恢复写保护）
 */
void CMCU_06_Init(void);

/**
 * @brief 写入保护控制
 * @param addr  传感器地址 (1-6)
 * @param state true=打开写保护, false=关闭写保护
 */
void CMCU_06_Write_Protect(uint8_t addr, bool state);

/**
 * @brief 复位传感器（读取两个保持寄存器实现复位）
 * @param addr 传感器地址 (1-6)
 */
void CMCU_06_Reset(uint8_t addr);

/**
 * @brief 去皮置零（清零当前载荷）
 * @param addr 传感器地址 (1-6)
 */
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

/**
 * @brief 发送单传感器数据读取命令
 * @param addr 传感器地址 (1-6)
 */
void CMCU_06_single_read(uint8_t addr);

/**
 * @brief 逐字节解析 CMCU-06 Modbus-RTU 响应
 * @param byte USART1 接收到的字节
 */
void CMCU_06_Parse_Byte(uint8_t byte);

/**
 * @brief 复位 Modbus-RTU 解析状态机
 */
void CMCU_06_Parse_Reset(void);

/**
 * @brief 导出所有传感器的 EMA 滤波器状态（用于持久化保存）
 * @param first_sample_out 输出首采样标志数组，长度 SENSOR_NUM
 * @param filter_val_out   输出滤波值数组，长度 SENSOR_NUM
 */
void CMCU_06_Filter_Export(uint8_t *first_sample_out, int32_t *filter_val_out);

/**
 * @brief 恢复所有传感器的 EMA 滤波器状态（用于断电恢复）
 * @param first_sample 首采样标志数组，长度 SENSOR_NUM
 * @param filter_val   滤波值数组，长度 SENSOR_NUM
 */
void CMCU_06_Filter_Restore(const uint8_t *first_sample, const int32_t *filter_val);

#endif /* CONTROLSYSTEM_CMCU_06_H */
