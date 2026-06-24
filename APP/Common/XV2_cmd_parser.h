/**
 * X_V2 步进闭环驱动 — 电机控制库
 *
 * Motor 结构体 + 指令构建 + 指令解析
 */

#ifndef X_V2_MOTOR_H
#define X_V2_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "../../BSP/Motor/Motor.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  功能码
 * ================================================================ */
#define X_V2_FC_MOTOR_ENABLE       0xF3
#define X_V2_FC_SPEED_MODE         0xF6
#define X_V2_FC_POSITION_MODE      0xFD
#define X_V2_FC_IMMEDIATE_STOP     0xFE
#define X_V2_FC_SYNC_MOTION        0xFF
#define X_V2_FC_SET_HOME           0x93
#define X_V2_FC_TRIGGER_HOMING     0x9A
#define X_V2_FC_ABORT_HOMING       0x9C
#define X_V2_FC_READ_HOMING_PARAM  0x22
#define X_V2_FC_WRITE_HOMING_PARAM 0x4C
#define X_V2_FC_READ_HOMING_STATUS 0x3B
#define X_V2_FC_TRIGGER_CALIB      0x06
#define X_V2_FC_CLEAR_POSITION     0x0A
#define X_V2_FC_RELEASE_STALL      0x0E
#define X_V2_FC_FACTORY_RESET      0x0F
#define X_V2_FC_READ_FW_VERSION    0x1F
#define X_V2_FC_READ_PHASE_RL      0x20
#define X_V2_FC_READ_POS_PID       0x21
#define X_V2_FC_READ_BUS_VOLTAGE   0x24
#define X_V2_FC_READ_PHASE_CURRENT 0x27
#define X_V2_FC_READ_ENCODER       0x31
#define X_V2_FC_READ_INPUT_PULSES  0x32
#define X_V2_FC_READ_TARGET_POS    0x33
#define X_V2_FC_READ_REALTIME_TPOS 0x34
#define X_V2_FC_READ_REALTIME_SPD  0x35
#define X_V2_FC_READ_REALTIME_POS  0x36
#define X_V2_FC_READ_POS_ERROR     0x37
#define X_V2_FC_READ_MOTOR_STATUS  0x3A
#define X_V2_FC_READ_DRV_CONFIG    0x42
#define X_V2_FC_READ_SYS_STATUS    0x43
#define X_V2_FC_WRITE_MICROSTEP    0x84
#define X_V2_FC_WRITE_ID_ADDR      0xAE
#define X_V2_FC_WRITE_LOOP_MODE    0x46
#define X_V2_FC_WRITE_OPEN_CURRENT 0x44
#define X_V2_FC_WRITE_DRV_CONFIG   0x48
#define X_V2_FC_WRITE_POS_PID      0x4A
#define X_V2_FC_STORE_SPEED_PARAM  0xF7
#define X_V2_FC_WRITE_VEL_SCALE    0x4F

#define X_V2_DIR_CW    0x00
#define X_V2_DIR_CCW   0x01
#define X_V2_POS_REL   0x00
#define X_V2_POS_ABS   0x01

#define X_V2_ST_OK          0x02
#define X_V2_ST_COND_FAIL   0xE2
#define X_V2_ST_ERROR       0xEE
#define X_V2_ST_REACHED     0x9F

/* ================================================================
 *  返回值
 * ================================================================ */
typedef enum {
    X_V2_OK             =  0,
    X_V2_ERR_TOO_SHORT  = -1,
    X_V2_ERR_UNKNOWN_FC = -2,
    X_V2_ERR_CHECKSUM   = -3,
    X_V2_ERR_LENGTH     = -4,
	X_V2_ERR_ADDR		 = -5
} X_V2_result_t;


/* ================================================================
 *  工具
 * ================================================================ */
static inline uint16_t X_V2_be16(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | p[1];
}
static inline uint32_t X_V2_be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

X_V2_result_t X_V2_parse_can(GlobalMotor *m, uint32_t can_ext_id,const uint8_t *buf, uint32_t len, bool verify_cs);
X_V2_result_t X_V2_build(GlobalMotor *m, uint8_t fc);
const char* X_V2_cmd_name(uint8_t fc);
void demo_parse(const char *desc, GlobalMotor *m, const uint8_t *buf, int len);
void demo_build(const char *desc, GlobalMotor *m, uint8_t fc);

#ifdef __cplusplus
}
#endif
#endif