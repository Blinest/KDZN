/**
 * @file cmd_packer.c
 * @brief 指令打包库
 *
 * 负责将全局结构体中的数据打包成数据包，发送给上位机。
 *
 * 结构上分两层:
 *   1) 数据段构造 (_payload_*)  —— 只关心字段, 与帧头无关
 *   2) 帧封装 (proto_pack_v1 / proto_pack_v2) —— 按当前协议版本选择帧头
 *
 *   V1 (legacy): [0xBB][func][LEN][DATA][SUM]
 *   V2 (扩展)  : [0x55][DEV][FUNC][LEN][DATA][SUM]   ← 不同设备 → 不同界面
 *
 * @date 2026-03-30
 * @author blin
 */

#include "cmd_packer.h"
#include "CR/CR.h"
#include "proto.h"
#include "cmsis_os2.h"

/* 引用外部队列 */
extern osMessageQueueId_t SensorDataQueueHandle;

/* ==================== 数据段构造 (与帧头无关) ==================== */

/** 系统状态数据段: 73 字节 */
static uint16_t _payload_status(uint8_t *p,
                                GlobalMotor motor[MOTOR_NUM],
                                GlobalSensor sensor[SENSOR_NUM],
                                const ContinuumRobot *cr,
                                uint8_t state)
{
    uint16_t i = 0;

    p[i++] = MOTOR_NUM;
    p[i++] = SENSOR_NUM;

    /* 电机数据: pos/vel/acc 各 int16 (×0.01), state uint8 → 每路 7 字节 */
    for (int k = 0; k < MOTOR_NUM; k++) {
        int16_t pos = (int16_t)(motor[k].stepper_motor.current_pos * 100); /* mm   */
        int16_t vel = (int16_t)(motor[k].stepper_motor.current_vel * 100); /* mm/s */
        int16_t acc = (int16_t)(motor[k].stepper_motor.current_acc * 100); /* mm/s²*/
        p[i++] = (uint8_t)(((uint16_t)pos >> 8) & 0xFF); p[i++] = (uint8_t)(pos & 0xFF);
        p[i++] = (uint8_t)(((uint16_t)vel >> 8) & 0xFF); p[i++] = (uint8_t)(vel & 0xFF);
        p[i++] = (uint8_t)(((uint16_t)acc >> 8) & 0xFF); p[i++] = (uint8_t)(acc & 0xFF);
        p[i++] = (uint8_t)motor[k].state;
    }

    /* 传感器数据: x/y 各 int16 (×0.01 N) → 每路 4 字节
     * x = 原始值; y = 滤波值  */
    for (int k = 0; k < SENSOR_NUM; k++) {
        int16_t x = (int16_t)(sensor[k].x * 100);
        int16_t y = (int16_t)(sensor[k].y * 100);
        p[i++] = (uint8_t)(((uint16_t)x >> 8) & 0xFF); p[i++] = (uint8_t)(x & 0xFF);
        p[i++] = (uint8_t)(((uint16_t)y >> 8) & 0xFF); p[i++] = (uint8_t)(y & 0xFF);
    }

    int16_t scale   = (int16_t)(cr->operation_space.scale * 100);
    int16_t armbend = (int16_t)(cr->joint_space.total_target_theta * 100);
    p[i++] = (uint8_t)(((uint16_t)scale   >> 8) & 0xFF); p[i++] = (uint8_t)(scale & 0xFF);
    p[i++] = (uint8_t)(((uint16_t)armbend >> 8) & 0xFF); p[i++] = (uint8_t)(armbend & 0xFF);
    p[i++] = state;

    return i;   /* = 85 */
}

/** 标定事件数据段: 3 字节 */
static uint16_t _payload_calib_event(uint8_t *p, uint8_t stage, uint8_t err, uint8_t mask)
{
    p[0] = stage;
    p[1] = err;
    p[2] = mask;
    return 3;
}

/* ==================== 对外接口 ==================== */

/**
 * @brief 打包系统状态帧 (自动按当前协议版本选帧头)
 *
 * 数据段 73 字节 (仅电机 + 传感器 + CR 部分状态), 总帧长 78 字节。
 *
 * @note 2026-10-15: 张力感知数据段已全部去除。
 */
uint16_t cmd_packer_pack_status_frame(uint8_t* frame, GlobalMotor motor[MOTOR_NUM],
                                      GlobalSensor sensor[SENSOR_NUM],
                                      const ContinuumRobot *cr, uint8_t state)
{
    uint8_t pl[96];
    const uint16_t n = _payload_status(pl, motor, sensor, cr, state);
    return proto_pack_uplink(frame, PROTO_DEV_DRIVE, PROTO_F_DRIVE_STATUS, pl, n);
}

/**
 * @brief 打包标定超时帧 (总长 7 字节, 数据段 3)
 */
uint16_t cmd_packer_pack_fb_timeout_frame(uint8_t* frame, uint8_t stage,
                                          uint8_t err, uint8_t mask)
{
    uint8_t pl[4];
    const uint16_t n = _payload_calib_event(pl, stage, err, mask);
    return proto_pack_uplink(frame, PROTO_DEV_CALIB, PROTO_F_CALIB_EVENT, pl, n);
}

/**
 * @brief 打包握手应答帧
 * 数据段: ver(1) motor_n(1) sensor_n(1) flags(1)
 */
uint16_t cmd_packer_pack_handshake_ack(uint8_t* frame)
{
    uint8_t pl[4];
    pl[0] = (uint8_t)PROTO_VERSION;
    pl[1] = MOTOR_NUM;
    pl[2] = SENSOR_NUM;
    pl[3] = 0;                          /* 能力标志位, 预留 */

    return proto_pack_uplink(frame, PROTO_DEV_SYS, PROTO_F_SYS_HANDSHAKE_ACK, pl, 4);
}

/**
 * @brief 发送打包后的帧到队列
 */
void cmd_packer_send_frame_to_queue(uint8_t* frame, uint16_t frame_len) {
    for (uint16_t i = 0; i < frame_len; i++) {
        uint8_t msg = frame[i];
        osMessageQueuePut(SensorDataQueueHandle, &msg, 0, 0);
    }
}

/**
 * @brief 发送系统状态帧到上位机
 */
void cmd_packer_send_status_frame(void) {
    uint8_t packed_frame[128];
	uint8_t state = CR.state;
    uint16_t frame_len = cmd_packer_pack_status_frame(packed_frame, global_motor, global_sensor, &CR, state);
    cmd_packer_send_frame_to_queue(packed_frame, frame_len);
}
