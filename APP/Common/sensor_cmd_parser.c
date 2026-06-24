/**
 * @file sensor_cmd_parser.c
 * @brief 下位机指令解析库
 *
 * 负责解析外设（传感器）反馈的指令，更新全局结构体
 *
 * @date 2026-03-30
 * @author blin
 */

#include "sensor_cmd_parser.h"
#include "XV2_cmd_parser.h"
#include "Sensor/Sensor.h"
#include "cmsis_os2.h"
#include "string.h"
#include "cmd_packer.h"
#include "can_driver.h"


/* 帧头定义 */
#define SENSOR_ID 0x01 /* 传感器地址定义 */
/* 功能码定义 */
#define FUNC_SENSOR_FEEDBACK  0x03    /* 传感器反馈功能码 */

/* 帧解析状态机 */
typedef enum {
 SENSOR_STATE_HEAD = 0,   /* 等待帧头 */
 SENSOR_STATE_FUNC,       /* 已收到帧头，等待功能码 */
 SENSOR_STATE_LEN,        /* 已收到功能码，等待数据长度 L */
 SENSOR_STATE_DATA,       /* 已知 L，接收 L 字节数据 */
 SENSOR_STATE_CHECK       /* 等待校验和字节 */
} SensorParseState_t;


/* --- 传感器反馈解析缓冲区与状态 --- */
#define SENSOR_BUF_SIZE  32
static uint8_t s_sensorBuf[SENSOR_BUF_SIZE];
static uint8_t s_sensorLen;
static uint8_t s_sensorIdx;
static SensorParseState_t s_sensorState = SENSOR_STATE_HEAD;


/**
 * @brief 重置传感器反馈解析状态
 */
static void sensor_parser_reset(void) {
    s_sensorState = SENSOR_STATE_HEAD;
    s_sensorLen = 0;
    s_sensorIdx = 0;
	// 也可以把缓冲数组给清空，不过似乎会极大拖慢执行速度，而且理论上不会出现异常问题，即使出现异常也已经做过异常判断了，去除
	// for (size_t i = 0; i < SENSOR_BUF_SIZE; ++i) {
	// 	s_sensorBuf[i] = 0;
	// }
}



/**
 * @brief 传感器指令解析函数 (Peripheral -> STM32)
 * @param byte 接收到的单字节数据
 */
void sensor_data_parser_feed_byte(uint8_t byte)
{
    uint8_t receive = byte;

    switch (s_sensorState)
    {
        case SENSOR_STATE_HEAD:
            if (receive == SENSOR_ID)
            {
                s_sensorBuf[0] = receive;
                s_sensorIdx = 1;
                s_sensorState = SENSOR_STATE_FUNC;
            }
            break;

        case SENSOR_STATE_FUNC:
            s_sensorBuf[1] = receive;
            s_sensorIdx = 2;

            if (receive == FUNC_SENSOR_FEEDBACK) {
                s_sensorState = SENSOR_STATE_LEN;
            } else {
                // 不是传感器反馈功能码，重置状态
                sensor_parser_reset();
            }
            break;

        case SENSOR_STATE_LEN:
            s_sensorLen = receive;
            s_sensorBuf[2] = receive;
            if (s_sensorLen > (SENSOR_BUF_SIZE - 4) || s_sensorLen == 0)
            {
                sensor_parser_reset();
                break;
            }

            s_sensorIdx = 3;
            s_sensorState = (s_sensorLen == 0) ? SENSOR_STATE_CHECK : SENSOR_STATE_DATA;
            break;

        case SENSOR_STATE_DATA:
            if (s_sensorIdx < SENSOR_BUF_SIZE)
                s_sensorBuf[s_sensorIdx++] = receive;

            if (s_sensorIdx >= (uint8_t)(3 + s_sensorLen))
            {
                s_sensorState = SENSOR_STATE_CHECK;
            }
            break;

        case SENSOR_STATE_CHECK:
            // 存储校验字节
            s_sensorBuf[s_sensorIdx] = receive;

            // 计算校验和
            uint16_t sum = 0;
            for (int i = 0; i < s_sensorIdx; i++)
                sum += s_sensorBuf[i];

            if ((sum & 0xFF) == receive) {
                // 校验通过，解析传感器数据
                parse_sensor_feedback_data();
            }
            sensor_parser_reset();
            break;

        default:
            sensor_parser_reset();
            break;
    }
}

/**
 * @brief 解析传感器反馈数据并更新全局结构体
 */
void parse_sensor_feedback_data(void)
{
    uint8_t func = s_sensorBuf[1];
    uint8_t data_len = s_sensorLen;

    if (func == FUNC_SENSOR_FEEDBACK && data_len >= 7)
    {
        uint8_t sensor_id = s_sensorBuf[3];

        if (sensor_id >= 1 && sensor_id <= SENSOR_NUM)
        {
            // 解析传感器数据 (大端序)
            int16_t x = (int16_t)((s_sensorBuf[4] << 8) | s_sensorBuf[5]);
            int16_t y = (int16_t)((s_sensorBuf[6] << 8) | s_sensorBuf[7]);
            int16_t z = (int16_t)((s_sensorBuf[8] << 8) | s_sensorBuf[9]);

            // 更新全局传感器结构体 (原始数据)
            global_sensor[sensor_id-1].x = (uint16_t)x; // 俯仰角
            global_sensor[sensor_id-1].y = (uint16_t)y; // 横滚角
            global_sensor[sensor_id-1].z = (uint16_t)z; // 偏航角


            // 调试输出
            #ifdef DEBUG_SENSOR
            printf("[SENSOR] 传感器%d数据更新: X=%d, Y=%d, Z=%d (角度: %.2f, %.2f, %.2f)\n",
                   sensor_id, x, y, z,
                   sensor_angle[sensor_id-1][0],
                   sensor_angle[sensor_id-1][1],
                   sensor_angle[sensor_id-1][2]);
            #endif

            // 触发数据上报
            cmd_packer_send_status_frame();
        }
    }
}


