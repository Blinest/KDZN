/**
 * @file pc_cmd_parser.c
 * @brief 上位机指令解析库
 *
 * 负责解析上位机发送的指令，更新全局结构体，并分发给下位机
 *
 * @date 2026-03-30
 * @author blin
 */
#include "pc_cmd_parser.h"
#include "Motor/Motor.h"
#include "Sensor/Sensor.h"
#include "CR/SDM.h"
#include "CR/Perception.h"
#include "proto.h"
#include "usart.h"
#include "cmsis_os2.h"
#include "string.h"
#include <stdio.h>

#include "cmd_packer.h"



/* 帧头定义（协议 V2：下行仅 0xAA） */
#define FRAME_HEAD_DOWN       0xAA

/* 全部下行功能码统一由 proto.h 的 PROTO_C_* 定义 (单一来源) */

/* 帧解析状态机 */
typedef enum {
 CMD_STATE_HEAD = 0,   /* 等待帧头 0xAA */
 CMD_STATE_FUNC,       /* 已收到帧头，等待功能码 */
 CMD_STATE_LEN,        /* 已收到功能码，等待数据长度 L */
 CMD_STATE_DATA,       /* 已知 L，接收 L 字节数据 */
 CMD_STATE_CHECK       /* 等待校验和字节 */
} CmdParseState_t;

/* 解析缓冲区与状态 */
#define CTRL_BUF_SIZE  128
static uint8_t s_ctrlBuf[CTRL_BUF_SIZE];
static uint8_t s_ctrlLen;
static uint16_t s_ctrlIdx; // 提升为 16 位以确保安全
static CmdParseState_t s_ctrlState = CMD_STATE_HEAD;

/* 连接状态 */
extern bool is_connected;

/**
 * @brief 重置控制指令解析状态
 */
static void pc_cmd_parser_reset(void) {
    s_ctrlState = CMD_STATE_HEAD;
    s_ctrlLen = 0;
    s_ctrlIdx = 0;
    // 增加：清理缓冲区
    memset(s_ctrlBuf, 0, CTRL_BUF_SIZE);
}

/* ==================== 公共处理器 (新旧功能码共用) ==================== */

static void _h_motor_disable(void)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        motor_enable(global_motor[i].id, false);
        osDelay(1);
    }
}

static void _h_motor_enable(void)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        motor_enable(global_motor[i].id, true);
        osDelay(1);
    }
}

/**
 * @brief 单电机定位: addr(1) dir(1) dist(2) vel(2) —— 数据段 6 字节
 *
 * 修 bug: 旧实现判断条件写成 `>= 8`, 注释写 `1+2+2+2`, 实际只读 6 字节。
 *         三处不一致 ⇒ 上位机按 6 字节发送时指令被静默丢弃。现统一为 `>= 6`。
 */
static void _h_motor_single(const uint8_t *d, uint8_t len)
{
    if (len < 6) return;

    const uint8_t  addr = d[0];
    const uint8_t  dir  = d[1];
    const uint16_t dist = (uint16_t)(((uint16_t)d[2] << 8) | d[3]);
    const uint16_t vel  = (uint16_t)(((uint16_t)d[4] << 8) | d[5]);

    if (addr < 1u || addr > MOTOR_NUM) return;
    motor_single_control((uint8_t)(addr - 1u), dir,
                         (float)dist / 100.0f, (float)vel / 100.0f);
}

/** @brief 多电机同步(连续): count(1) start_addr(1) count×int16(×0.01mm,带符号) */
static void _h_motor_sync(const uint8_t *d, uint8_t len)
{
    if (len < 2) return;

    uint8_t       count      = d[0];
    const uint8_t start_addr = d[1];

    if ((uint16_t)len < (uint16_t)(2u + (uint16_t)count * 2u)) return;

    const uint8_t start_idx = (start_addr > 0u) ? (uint8_t)(start_addr - 1u) : 0u;
    if (start_idx >= MOTOR_NUM) return;

    if (count > (uint8_t)(MOTOR_NUM - start_idx)) {
        count = (uint8_t)(MOTOR_NUM - start_idx);
    }
    if (count == 0u) return;

    float distances[MOTOR_NUM] = {0};
    for (uint8_t i = 0u; i < count; i++) {
        const int16_t raw = (int16_t)(((uint16_t)d[2 + i * 2] << 8)
                                    |  (uint16_t)d[3 + i * 2]);
        distances[i] = (float)raw / 100.0f;
    }
    motor_sync_control(count, start_idx, distances);
}

/** @brief 多电机自定义: count(1) + count×[addr(1) dir(1) dist(2)×0.01mm] */
static void _h_motor_custom(const uint8_t *d, uint8_t len)
{
    if (len < 1) return;

    const uint8_t count = d[0];
    if (count == 0u || count > MOTOR_NUM) return;
    if ((uint16_t)len < (uint16_t)(1u + (uint16_t)count * 4u)) return;

    for (uint8_t i = 0u; i < count; i++) {
        const uint8_t  addr = d[1 + i * 4];
        const uint8_t  dir  = d[2 + i * 4];
        const uint16_t dist = (uint16_t)(((uint16_t)d[3 + i * 4] << 8) | d[4 + i * 4]);

        if (addr < 1u || addr > MOTOR_NUM) continue;
        motor_single_control((uint8_t)(addr - 1u), dir,
                             (float)dist / 100.0f, 10.0f);   /* 默认 10 mm/s */
    }
}

/** @brief 单段弯曲: seg(1) dir(1) angle(2)×0.01°  (dir: 0=朝上 1=朝下) */
static void _h_single_bend(const uint8_t *d, uint8_t len)
{
    if (len < 4) return;

    uint8_t seg = d[0];
    if (seg < 1u || seg > 2u) seg = 1u;                 /* 段号兜底 */

    const char    dirc = (d[1] == 0u) ? 'u' : 'd';
    const uint16_t ang = (uint16_t)(((uint16_t)d[2] << 8) | d[3]);

    armBend((int)seg, dirc, (double)ang / 100.0);
}

/** @brief 设置安装方向: dx(2) dy(2) dz(2), 单位向量 ×1000 */
static void _h_set_mount(const uint8_t *d, uint8_t len)
{
    if (len < 6) return;

    float dir[3];
    for (int k = 0; k < 3; k++) {
        const int16_t v = (int16_t)(((uint16_t)d[2 * k] << 8) | d[2 * k + 1]);
        dir[k] = (float)v / 1000.0f;
    }
    sdm_set_mount(dir);
}

/** @brief 设置重力补偿增益: gain(2) ×1000 */
static void _h_grav_comp(const uint8_t *d, uint8_t len)
{
    if (len < 2) return;
    const int16_t g = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
    sdm_set_grav_comp_gain((float)g / 1000.0f);
}

/** @brief 版本/能力查询: client_ver(1) → 回握手应答帧(携带本机版本/通道数) */
static void _h_handshake(const uint8_t *d, uint8_t len)
{
    (void)d;
    if (len < 1) return;

    uint8_t ack[16];
    const uint16_t n = cmd_packer_pack_handshake_ack(ack);
    if (n > 0 && n <= (uint16_t)sizeof(ack)) {
        Usart_SendString(&huart2, ack, n);
    }
}

/**
 * @brief 解析并执行控制指令
 */
static void pc_cmd_parse_and_execute(void)
{
    uint8_t head     = s_ctrlBuf[0];
    uint8_t func     = s_ctrlBuf[1];
    uint8_t data_len = s_ctrlLen;
    if (data_len > 0 && (uint8_t)(3 + data_len) > CTRL_BUF_SIZE)
        return;

    const uint8_t *d = &s_ctrlBuf[3];   /* 数据段起点 */

    switch (head)
    {
        case PROTO_DOWNLINK_MAGIC:
            switch (func)
            {
                /* ---------- 0x0_ 系统 ---------- */
                case PROTO_C_SYS_MOTOR_OFF:     /* 0x00 */
                    _h_motor_disable();  break;
                case PROTO_C_SYS_MOTOR_ON:      /* 0x01 */
                    _h_motor_enable();   break;
                case PROTO_C_SYS_MOTOR_STOP:    /* 0x02 */
                    motor_stop_all();    break;
                case PROTO_C_SYS_HANDSHAKE:     /* 0x09 */
                    _h_handshake(d, data_len); break;

                /* ---------- 0x1_ 电机运动 ---------- */
                case PROTO_C_MOTOR_SINGLE:      /* 0x10 */
                    _h_motor_single(d, data_len); break;
                case PROTO_C_MOTOR_SYNC:        /* 0x11 */
                    _h_motor_sync(d, data_len);   break;
                case PROTO_C_MOTOR_CUSTOM:      /* 0x12 */
                    _h_motor_custom(d, data_len); break;
                case PROTO_C_MOTOR_DEMO:        /* 0x13 */
                    action_group_demo();          break;

                /* ---------- 0x2_ 传感器 ---------- */
                case PROTO_C_SENSOR_INIT:       /* 0x20 */
                    sensor_init();                break;
                case PROTO_C_SENSOR_READ:       /* 0x21 */
                    if (data_len >= 1) {
                        sensor_single_read(d[0]);
                    }
                    break;
                case PROTO_C_SENSOR_READ_ALL:   /* 0x22 */
                    sensor_multi_read();          break;
                case PROTO_C_SENSOR_CAL:        /* 0x23 */
                    if (data_len >= 3) {
                        sensor_cal(d[0], (uint16_t)(((uint16_t)d[1] << 8) | d[2]));
                    }
                    break;
                case PROTO_C_PERCEPT_ZERO:      /* 0x24 */
                    perception_capture_baseline(); break;

                /* ---------- 0x3_ 配置 ---------- */
                case PROTO_C_SET_MOUNT:         /* 0x30 */
                    _h_set_mount(d, data_len);    break;
                case PROTO_C_GRAV_COMP:         /* 0x31 */
                    _h_grav_comp(d, data_len);    break;

                /* ---------- 0x4_ 动作 ---------- */
                case PROTO_C_SINGLE_BEND:       /* 0x40 */
                    _h_single_bend(d, data_len);  break;

                default:
                    break;
            }
            break;

        default:
            break;
    }
}

/**
 * @brief 上位机指令解析函数 (PC -> STM32)，将电机数据存入临时数据缓冲区，用于后续执行
 * @param byte 接收到的单字节数据
 */
void pc_cmd_parser_feed_byte(uint8_t byte)
{
    uint8_t receive = byte;

    switch (s_ctrlState)
    {
        case CMD_STATE_HEAD:
            if (receive == PROTO_DOWNLINK_MAGIC)
            {
                // 解析下一个字节，并将帧头存入发送数据缓冲区
                s_ctrlBuf[0] = receive;
                s_ctrlIdx = 1;
                s_ctrlState = CMD_STATE_FUNC;
            }
            break;

        /* 根据帧头进行不同的控制操作 */
        case CMD_STATE_FUNC:
            /* 保存功能码 */
            s_ctrlBuf[1] = receive;
            s_ctrlLen = 2;
    		s_ctrlState = CMD_STATE_LEN;
            break;

        case CMD_STATE_LEN:
            s_ctrlLen = receive;
            s_ctrlBuf[2] = receive;

            if (s_ctrlLen + 4 > CTRL_BUF_SIZE) // 帧头(1) + 功能(1) + 长度(1) + 校验(1)
            {
                pc_cmd_parser_reset();
                break;
            }

            s_ctrlIdx = 3;
            s_ctrlState = (s_ctrlLen == 0) ? CMD_STATE_CHECK : CMD_STATE_DATA;
            break;

        case CMD_STATE_DATA:
            s_ctrlBuf[s_ctrlIdx++] = receive;

            if (s_ctrlIdx >= (uint8_t)(3 + s_ctrlLen)) // 去除校验位
            {
                s_ctrlState = CMD_STATE_CHECK;
            }
            break;

        case CMD_STATE_CHECK:
            s_ctrlBuf[s_ctrlIdx] = receive; // 存入校验位
            uint16_t sum = 0;

            for (int i = 0; i < s_ctrlIdx; i++)
                sum += s_ctrlBuf[i];

            if ((sum & 0xFF) == receive) {
                is_connected = true; // 收到暗号，解锁连接
                pc_cmd_parse_and_execute();
            }
            pc_cmd_parser_reset();
            break;

        default:
            pc_cmd_parser_reset();
            break;
    }
}

/**
 * @brief 重置上位机指令解析器
 */
void pc_cmd_parser_reset_all(void) {
    pc_cmd_parser_reset();
}