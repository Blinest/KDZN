//
// Created by blin on 2026/4/30.
//
#include "usart.h"
#include "CMCU-06.h"
#include "Sensor.h"
#include <string.h>   // for memcpy
#include "main.h"     // for HAL_Delay
#include "cmsis_os2.h"
// 内部函数：计算CRC16-Modbus
static uint16_t CRC16_Modbus(uint8_t *buf, uint8_t len);

void CMCU_06_Init(void)
{
    // 关闭写入保护 -> 复位 -> 恢复写入保护（6个传感器地址1-6）
    // 注意：此函数在调度器启动前调用，必须用 HAL_Delay 而非 osDelay
    for (uint8_t i = 1; i <= 6; i++)
    {
        CMCU_06_Write_Protect(i, false);  // 关闭写入保护
        HAL_Delay(10);
        CMCU_06_Reset(i);                 // 复位传感器
        HAL_Delay(50);
        CMCU_06_Write_Protect(i, true);   // 重新打开写入保护
    }
}

/**
 * 写入保护控制
 * 发送: [addr] [0x06] [0x00] [0x17] [0x00] [state] [CRC_L] [CRC_H]
 * state: 0x00 = 打开写入保护, 0x01 = 关闭写入保护
 */
void CMCU_06_Write_Protect(uint8_t addr, bool enable_protect)
{
	uint8_t cmd[8];
	cmd[0] = addr;
	cmd[1] = 0x06;
	cmd[2] = 0x00;
	cmd[3] = 0x17;
	cmd[4] = 0x00;
	cmd[5] = enable_protect ? 0x00 : 0x01;   // 写入保护值

	// 计算 CRC16-Modbus (前6字节)
	uint16_t crc = CRC16_Modbus(cmd, 6);
	cmd[6] = crc & 0xFF;        // CRC 低字节
	cmd[7] = (crc >> 8) & 0xFF; // CRC 高字节

	Usart_SendString(&huart1, cmd, 8);
}

/**
 * 复位传感器（读取两个保持寄存器实现复位）
 * 发送: [addr] [0x03] [0x00] [0x00] [0x00] [0x02] [CRC_L] [CRC_H]
 */
void CMCU_06_Reset(uint8_t addr)
{
	uint8_t cmd[8];
	cmd[0] = addr;
	cmd[1] = 0x03;
	cmd[2] = 0x00;
	cmd[3] = 0x00;
	cmd[4] = 0x00;
	cmd[5] = 0x02;

	uint16_t crc = CRC16_Modbus(cmd, 6);
	cmd[6] = crc & 0xFF;
	cmd[7] = (crc >> 8) & 0xFF;

	Usart_SendString(&huart1, cmd, 8);
}

/**
 * 去皮置零（清零当前载荷）
 * 发送: [addr] [0x06] [0x00] [0x15] [0x00] [0x01] [CRC_L] [CRC_H]
 */
void CMCU_06_ResetPins(uint8_t addr)
{
	uint8_t cmd[8];
	cmd[0] = addr;
	cmd[1] = 0x06;
	cmd[2] = 0x00;
	cmd[3] = 0x15;
	cmd[4] = 0x00;
	cmd[5] = 0x01;   // 去皮命令值

	uint16_t crc = CRC16_Modbus(cmd, 6);
	cmd[6] = crc & 0xFF;
	cmd[7] = (crc >> 8) & 0xFF;

	Usart_SendString(&huart1, cmd, 8);
}


/**
 * 写入砝码校准值
 * 将已知重量的砝码值写入传感器校准寄存器，传感器据此计算内部缩放系数
 * 发送: [addr] [0x06] [0x00] [0x16] [weight_H] [weight_L] [CRC_L] [CRC_H]
 * @param addr  传感器地址 (1-6)
 * @param weight_10x 砝码重量值（单位：g，×10倍存储，例如 500g → 5000）
 */
void CMCU_06_WriteCalWeight(uint8_t addr, uint16_t weight_10x)
{
    uint8_t cmd[8];
    cmd[0] = addr;
    cmd[1] = 0x06;           // 写单寄存器功能码
    cmd[2] = 0x00;           // 校准寄存器地址 0x0016
    cmd[3] = 0x16;
    cmd[4] = (weight_10x >> 8) & 0xFF;  // 数据高字节
    cmd[5] = weight_10x & 0xFF;         // 数据低字节

    uint16_t crc = CRC16_Modbus(cmd, 6);
    cmd[6] = crc & 0xFF;
    cmd[7] = (crc >> 8) & 0xFF;

    Usart_SendString(&huart1, cmd, 8);
}

/**
 * 砝码校准完整流程：
 *   1. 关闭写入保护
 *   2. 置0（去皮）
 *   3. 写入砝码值
 *   4. 重新打开写入保护
 */
void CMCU_06_Cal(uint8_t addr, uint16_t weight_10x)
{
    // 1. 关闭写入保护
    CMCU_06_Write_Protect(addr, false);
    HAL_Delay(10);

    // 2. 置0去皮
    CMCU_06_ResetPins(addr);
    HAL_Delay(50);

    // 3. 写入砝码校准值
    CMCU_06_WriteCalWeight(addr, weight_10x);
    HAL_Delay(10);

    // 4. 重新打开写入保护
    CMCU_06_Write_Protect(addr, true);
}

/**
 * 单次读取传感器数据
 * 发送读指令后，等待接收9字节（地址+功能码+数据长度+4字节数据+CRC低+CRC高）
 */
void CMCU_06_single_read(uint8_t addr)
{
	// 1. 发送读命令（与 Reset 命令结构相同，但独立发送）
	uint8_t cmd[8];
	cmd[0] = addr;
	cmd[1] = 0x03;
	cmd[2] = 0x00;
	cmd[3] = 0x00;
	cmd[4] = 0x00;
	cmd[5] = 0x02;

	uint16_t crc = CRC16_Modbus(cmd, 6);
	cmd[6] = crc & 0xFF;
	cmd[7] = (crc >> 8) & 0xFF;

	Usart_SendString(&huart1, cmd, 8);
}

/* ==================== Modbus-RTU 响应解析 ==================== */

/* 解析状态机 */
typedef enum {
    CMCU_PARSE_WAIT_ADDR = 0,
    CMCU_PARSE_WAIT_FUNC,
    CMCU_PARSE_WAIT_LEN,
    CMCU_PARSE_WAIT_DATA,
    CMCU_PARSE_WAIT_CRC_L,
    CMCU_PARSE_WAIT_CRC_H,
} CMCU_ParseState_t;

/* Modbus读保持寄存器响应最大字节数: [addr][func][len][data...][CRC_L][CRC_H] */
#define CMCU_RSP_BUF_SIZE  9

static struct {
    CMCU_ParseState_t state;
    uint8_t buf[CMCU_RSP_BUF_SIZE];
    uint8_t idx;
    uint8_t data_len;   /* 数据字段字节数（来自第3字节） */
    uint8_t slave_addr; /* 从机地址 */
    uint8_t func;       /* 功能码 */
} s_cmcu_parse;

/**
 * @brief 复位Modbus-RTU解析状态机
 */
void CMCU_06_Parse_Reset(void)
{
    s_cmcu_parse.state = CMCU_PARSE_WAIT_ADDR;
    s_cmcu_parse.idx = 0;
    s_cmcu_parse.data_len = 0;
    s_cmcu_parse.slave_addr = 0;
    s_cmcu_parse.func = 0;
}

/**
 * @brief 逐字节解析CMCU-06 Modbus-RTU响应
 *
 * Modbus-RTU 读保持寄存器响应格式:
 *   [Addr][0x03][ByteCnt][DataH][DataL][DataH][DataL][CRCL][CRCH] = 9字节
 * 4字节原始数据对应2个寄存器，第1个寄存器为力值数据
 *
 * @param byte 从USART1接收到的字节
 */
void CMCU_06_Parse_Byte(uint8_t byte)
{
    switch (s_cmcu_parse.state)
    {
    case CMCU_PARSE_WAIT_ADDR:
        /* 从机地址：1-6 为压力传感器 */
        if (byte >= 1 && byte <= 6)
        {
            s_cmcu_parse.slave_addr = byte;
            s_cmcu_parse.buf[0] = byte;
            s_cmcu_parse.idx = 1;
            s_cmcu_parse.state = CMCU_PARSE_WAIT_FUNC;
        }
        break;

    case CMCU_PARSE_WAIT_FUNC:
        if (byte == 0x03) /* 读保持寄存器功能码 */
        {
            s_cmcu_parse.func = byte;
            s_cmcu_parse.buf[1] = byte;
            s_cmcu_parse.idx = 2;
            s_cmcu_parse.state = CMCU_PARSE_WAIT_LEN;
        }
        else
        {
            CMCU_06_Parse_Reset();
        }
        break;

    case CMCU_PARSE_WAIT_LEN:
        s_cmcu_parse.data_len = byte;
        s_cmcu_parse.buf[2] = byte;
        s_cmcu_parse.idx = 3;

        if (s_cmcu_parse.data_len == 4) /* 期望4字节数据（2个寄存器） */
        {
            s_cmcu_parse.state = CMCU_PARSE_WAIT_DATA;
        }
        else
        {
            CMCU_06_Parse_Reset();
        }
        break;

    case CMCU_PARSE_WAIT_DATA:
        if (s_cmcu_parse.idx < CMCU_RSP_BUF_SIZE)
        {
            s_cmcu_parse.buf[s_cmcu_parse.idx++] = byte;
        }

        /* 已收完 data_len 字节数据？ */
        if (s_cmcu_parse.idx >= (uint8_t)(3 + s_cmcu_parse.data_len))
        {
            s_cmcu_parse.state = CMCU_PARSE_WAIT_CRC_L;
        }
        break;

    case CMCU_PARSE_WAIT_CRC_L:
        s_cmcu_parse.buf[s_cmcu_parse.idx++] = byte;
        s_cmcu_parse.state = CMCU_PARSE_WAIT_CRC_H;
        break;

    case CMCU_PARSE_WAIT_CRC_H:
        s_cmcu_parse.buf[s_cmcu_parse.idx++] = byte;

        /* 完整帧已接收，验证CRC并提取数据 */
        {
            uint8_t len = s_cmcu_parse.idx;
            uint16_t calc_crc = CRC16_Modbus(s_cmcu_parse.buf, len - 2);
            uint16_t recv_crc = (uint16_t)(s_cmcu_parse.buf[len - 1] << 8) | s_cmcu_parse.buf[len - 2];

            if (calc_crc == recv_crc)
            {
                /* CRC验证通过，提取力值数据（大端序4字节） */
                int32_t raw = ((int32_t)s_cmcu_parse.buf[3] << 24)
                            | ((int32_t)s_cmcu_parse.buf[4] << 16)
                            | ((int32_t)s_cmcu_parse.buf[5] << 8)
                            | (int32_t)s_cmcu_parse.buf[6];


                 /* 具体转换系数根据实际传感器标定调整 */
                float force = (float)raw / 100;

                uint8_t sensor_idx = s_cmcu_parse.slave_addr - 1;
                if (sensor_idx < SENSOR_NUM)
                {
                    global_sensor[sensor_idx].press_sensor.raw_val = (int32_t)(force * 100);
                    global_sensor[sensor_idx].press_sensor.val = force;

                }
            }
        }

        CMCU_06_Parse_Reset();
        break;

    default:
        CMCU_06_Parse_Reset();
        break;
    }
}

// CRC16-Modbus 计算函数（多项式0x8005，初始0xFFFF）
static uint16_t CRC16_Modbus(uint8_t *buf, uint8_t len)
{
	uint16_t crc = 0xFFFF;
	for (uint8_t i = 0; i < len; i++) {
		crc ^= buf[i];
		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 0x0001) {
				crc = (crc >> 1) ^ 0xA001;
			} else {
				crc >>= 1;
			}
		}
	}
	return crc;
}