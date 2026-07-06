/**
 * @file CMCU-06.c
 * @brief CMCU-06 压力传感器 Modbus-RTU 驱动实现
 *
 * 包含 Modbus 帧收发、解析状态机、校准流程、EMA 滤波。
 *
 * @date 2026-04-30
 * @author blin
 */
#include "usart.h"
#include "CMCU-06.h"
#include "Sensor.h"
#include "SensorFilter.h"
#include <string.h>   // for memcpy
#include "main.h"     // for HAL_Delay
#include "cmsis_os2.h"

/* 每传感器独立的 EMA 滤波器实例 */
static SensorFilter s_filters[SENSOR_NUM];

/* Modbus读保持寄存器响应最大字节数: [addr][func][len][data...][CRC_L][CRC_H] */
#define CMCU_RSP_BUF_SIZE  9

// 内部函数：计算CRC16-Modbus
static uint16_t CRC16_Modbus(uint8_t *buf, uint8_t len);

void CMCU_06_Init(void)
{
    /* 初始化 EMA 滤波器 */
    for (int i = 0; i < SENSOR_NUM; i++) {
        SensorFilter_Init(&s_filters[i]);
    }

    // 关闭写入保护 -> 复位 -> 恢复写入保护（6个传感器地址1-6）
    // 注意：此函数在调度器启动前调用，必须用 HAL_Delay 而非 osDelay
    /*
    for (uint8_t i = 1; i <= 6; i++)
    {
        CMCU_06_Write_Protect(i, false);  // 关闭写入保护
        HAL_Delay(10);
        CMCU_06_Reset(i);                 // 复位传感器
        HAL_Delay(50);
        CMCU_06_Write_Protect(i, true);   // 重新打开写入保护
    }
    */
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
    /* 校准后丢弃旧滤波状态，下次采样重新初始化 */
    SensorFilter_Reset(&s_filters[addr - 1]);

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
static CMCU_Parser s_cmcu_parse;


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

/* CMCU_Parser 定义在 SensorParser.h */

/**
 * @brief 复位Modbus-RTU解析状态机
 */
void CMCU_06_Parse_Reset(void)
{
    s_cmcu_parse.state = CMCU_STATE_HEAD;
    s_cmcu_parse.idx = 0;
    s_cmcu_parse.data_len = 0;
    s_cmcu_parse.slave_addr = 0;
    s_cmcu_parse.func = 0;
    s_cmcu_parse.expected_addr = 0;
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
    case CMCU_STATE_HEAD:
        /* 从机地址：1-6 为压力传感器 */
        if (byte >= 1 && byte <= 6)
        {
            s_cmcu_parse.slave_addr = byte;
            s_cmcu_parse.buf[0] = byte;
            s_cmcu_parse.idx = 1;
            s_cmcu_parse.state = CMCU_STATE_FUNC;
        }
        break;

    case CMCU_STATE_FUNC:
        if (byte == 0x03) /* 读保持寄存器功能码 */
        {
            s_cmcu_parse.func = byte;
            s_cmcu_parse.buf[1] = byte;
            s_cmcu_parse.idx = 2;
            s_cmcu_parse.state = CMCU_STATE_LEN;
        }
        else if (byte == 0x06) /* 写寄存器回复（去皮/写保护），直接丢弃重置 */
        {
            CMCU_06_Parse_Reset();
        }
        else
        {
            CMCU_06_Parse_Reset();
        }
        break;

    case CMCU_STATE_LEN:
        s_cmcu_parse.data_len = byte;
        s_cmcu_parse.buf[2] = byte;
        s_cmcu_parse.idx = 3;

        if (s_cmcu_parse.data_len == 4) /* 期望4字节数据（2个寄存器） */
        {
            s_cmcu_parse.state = CMCU_STATE_DATA;
        }
        else
        {
            CMCU_06_Parse_Reset();
        }
        break;

    case CMCU_STATE_DATA:
        if (s_cmcu_parse.idx < CMCU_RSP_BUF_SIZE)
        {
            s_cmcu_parse.buf[s_cmcu_parse.idx++] = byte;
        }

        /* 已收完 data_len 字节数据？ */
        if (s_cmcu_parse.idx >= (uint8_t)(3 + s_cmcu_parse.data_len))
        {
            s_cmcu_parse.state = CMCU_STATE_CRC1;
        }
        break;

    case CMCU_STATE_CRC1:
        s_cmcu_parse.buf[s_cmcu_parse.idx++] = byte;
        s_cmcu_parse.state = CMCU_STATE_CRC2;
        break;

    case CMCU_STATE_CRC2:
        s_cmcu_parse.buf[s_cmcu_parse.idx++] = byte;

        /* 完整帧已接收，验证CRC并提取数据 */
        {
            uint8_t len = s_cmcu_parse.idx;
            uint16_t calc_crc = CRC16_Modbus(s_cmcu_parse.buf, len - 2);
            uint16_t recv_crc = (uint16_t)(s_cmcu_parse.buf[len - 1] << 8) | s_cmcu_parse.buf[len - 2];

            if (calc_crc == recv_crc)
            {
                /* CRC验证通过，提取力值数据（低字优先 + 字内大端） */
                /* buf[3-4] = 低16位（字内大端：buf[3]=高字节, buf[4]=低字节） */
                /* buf[5-6] = 高16位（字内大端：buf[5]=高字节, buf[6]=低字节） */
                int32_t raw = ((int32_t)s_cmcu_parse.buf[5] << 24)
                            | ((int32_t)s_cmcu_parse.buf[6] << 16)
                            | ((int32_t)s_cmcu_parse.buf[3] << 8)
                            | (int32_t)s_cmcu_parse.buf[4];


                 /* 具体转换系数根据实际传感器标定调整 */
                float force = (float)raw;

                uint8_t sensor_idx = s_cmcu_parse.slave_addr - 1;
                if (sensor_idx < SENSOR_NUM)
                {
                    global_sensor[sensor_idx].press_sensor.raw_val = (int32_t)force;

                    /* 用 sensitivity_scale 归一化后写入 val */
                    float scale = global_sensor[sensor_idx].press_sensor.sensitivity_scale;
                    if (scale <= 0.0f) scale = 1.0f;
                    global_sensor[sensor_idx].press_sensor.val =
                        (int32_t)(force * scale);

                    /* EMA 软件滤波，输出写入 filter_val */
                    global_sensor[sensor_idx].press_sensor.filter_val =
                        SensorFilter_Update(&s_filters[sensor_idx], (int32_t)force);

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

/* ==================== 滤波器状态导出/恢复 ==================== */

void CMCU_06_Filter_Export(uint8_t *first_sample_out, int32_t *filter_val_out)
{
    for (int i = 0; i < SENSOR_NUM; i++) {
        first_sample_out[i] = s_filters[i].first_sample;
        filter_val_out[i]   = s_filters[i].filter_val;
    }
}

void CMCU_06_Filter_Restore(const uint8_t *first_sample, const int32_t *filter_val)
{
    for (int i = 0; i < SENSOR_NUM; i++) {
        s_filters[i].first_sample = first_sample[i];
        s_filters[i].filter_val   = filter_val[i];
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