/**
 * EmM42_V5.0 帧打包/解包库
 *
 * 帧格式:
 *   [0xBB] [功能码] [长度] [MOTOR_NUM] [SENSOR_NUM]
 *   [电机1 X(2) Y(2) Z(2) state(1)] [电机2 ...]
 *   [传感器1 X(2) Y(2) Z(2)] [传感器2 ...]
 *   [scale1(2)] [scale2(2)] [sys_state(1)]
 *   [校验和]
 *
 * 电机 X=位移  Y=速度  Z=加速度
 * 传感器 X/Y/Z = 传感器数值
 */

#ifndef EMM42_FRAME_H
#define EMM42_FRAME_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  帧常量
 * ================================================================ */

#define FRAME_MAGIC         0xBB
#define FRAME_MOTOR_BYTES   7    /* X(2) + Y(2) + Z(2) + state(1) */
#define FRAME_SENSOR_BYTES  6    /* X(2) + Y(2) + Z(2) */
#define FRAME_TAIL_BYTES    5    /* scale1(2) + scale2(2) + sys_state(1) */
#define FRAME_HEAD_BYTES    3    /* magic(1) + fc(1) + len(1) */
#define FRAME_CS_BYTES      1

/* 功能码 */
#define FC_MOTOR_STATUS     0x02
#define FC_MOTOR_CMD        0x03
#define FC_SENSOR_STATUS    0x04
#define FC_SYSTEM_STATUS    0x05

/* 最大节点数 (按需修改) */
#define MAX_MOTOR_NUM       8
#define MAX_SENSOR_NUM      8





/* ================================================================
 *  工具
 * ================================================================ */

static inline void frame_put_be16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}

static inline uint16_t frame_get_be16(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | p[1];
}

/* XOR 校验 (从功能码到 sys_state) */
static inline uint8_t frame_checksum(const uint8_t *data, int len) {
    uint8_t cs = 0;
    for (int i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

/* 帧总长度计算 */
static inline int frame_total_len(uint8_t motor_num, uint8_t sensor_num) {
    return FRAME_HEAD_BYTES
         + 2                                        /* motor_num + sensor_num */
         + motor_num  * FRAME_MOTOR_BYTES
         + sensor_num * FRAME_SENSOR_BYTES
         + FRAME_TAIL_BYTES
         + FRAME_CS_BYTES;
}

/* payload长度 (长度字段的值: 从motor_num到sys_state) */
static inline int frame_payload_len(uint8_t motor_num, uint8_t sensor_num) {
    return 2
         + motor_num  * FRAME_MOTOR_BYTES
         + sensor_num * FRAME_SENSOR_BYTES
         + FRAME_TAIL_BYTES;
}


/* ================================================================
 *  打包: FrameData → buf (可直接发送)
 *
 *  返回: 帧总字节数, 负值表示错误
 * ================================================================ */

int frame_pack(uint8_t *buf, int buf_size, const FrameData *f)
{
    if (!buf || !f) return -1;

    int total = frame_total_len(f->motor_num, f->sensor_num);
    if (buf_size < total) return -2;
    if (f->motor_num > MAX_MOTOR_NUM || f->sensor_num > MAX_SENSOR_NUM) return -3;

    int i = 0;

    /* 帧头 */
    buf[i++] = FRAME_MAGIC;
    buf[i++] = f->func_code;
    buf[i++] = (uint8_t)frame_payload_len(f->motor_num, f->sensor_num);

    /* 节点数 */
    buf[i++] = f->motor_num;
    buf[i++] = f->sensor_num;

    /* 电机数据 */
    for (int m = 0; m < f->motor_num; m++) {
        frame_put_be16(&buf[i], f->motors[m].displacement); i += 2;
        frame_put_be16(&buf[i], f->motors[m].speed);        i += 2;
        frame_put_be16(&buf[i], f->motors[m].accel);        i += 2;
        buf[i++] = f->motors[m].state;
    }

    /* 传感器数据 */
    for (int s = 0; s < f->sensor_num; s++) {
        frame_put_be16(&buf[i], f->sensors[s].x); i += 2;
        frame_put_be16(&buf[i], f->sensors[s].y); i += 2;
        frame_put_be16(&buf[i], f->sensors[s].z); i += 2;
    }

    /* 尾部 */
    frame_put_be16(&buf[i], f->scale1);   i += 2;
    frame_put_be16(&buf[i], f->scale2);   i += 2;
    buf[i++] = f->sys_state;

    /* 校验和: 从 func_code 到 sys_state (跳过 magic, 包含长度字段) */
    buf[i] = frame_checksum(&buf[1], i - 1);

    return total;
}


/* ================================================================
 *  解包: buf → FrameData
 *
 *  返回: 解析成功字节数, 负值表示错误
 * ================================================================ */

int frame_unpack(FrameData *f, const uint8_t *buf, int len, bool verify_cs)
{
    if (!f || !buf || len < FRAME_HEAD_BYTES + 2 + FRAME_TAIL_BYTES + 1)
        return -1;

    memset(f, 0, sizeof(*f));

    if (buf[0] != FRAME_MAGIC) return -2;

    f->func_code  = buf[1];
    uint8_t plen  = buf[2];  /* payload长度 */

    int expected = FRAME_HEAD_BYTES + plen + 1;
    if (len < expected) return -3;

    /* 校验 */
    if (verify_cs) {
        uint8_t cs = frame_checksum(&buf[1], plen + 2);  /* fc + len + payload */
        if (buf[expected - 1] != cs) return -4;
    }

    int i = 3;
    f->motor_num  = buf[i++];
    f->sensor_num = buf[i++];

    if (f->motor_num > MAX_MOTOR_NUM || f->sensor_num > MAX_SENSOR_NUM)
        return -5;

    /* 电机数据 */
    for (int m = 0; m < f->motor_num; m++) {
        f->motors[m].displacement = frame_get_be16(&buf[i]); i += 2;
        f->motors[m].speed        = frame_get_be16(&buf[i]); i += 2;
        f->motors[m].accel        = frame_get_be16(&buf[i]); i += 2;
        f->motors[m].state        = buf[i++];
    }

    /* 传感器数据 */
    for (int s = 0; s < f->sensor_num; s++) {
        f->sensors[s].x = frame_get_be16(&buf[i]); i += 2;
        f->sensors[s].y = frame_get_be16(&buf[i]); i += 2;
        f->sensors[s].z = frame_get_be16(&buf[i]); i += 2;
    }

    /* 尾部 */
    f->scale1    = frame_get_be16(&buf[i]); i += 2;
    f->scale2    = frame_get_be16(&buf[i]); i += 2;
    f->sys_state = buf[i++];

    return expected;
}


/* ================================================================
 *  Motor → FrameMotor 快捷转换
 * ================================================================ */

/* 从 Motor 结构体填充 FrameMotor
 * displacement = target_pos, speed = target_vel, accel = target_acc */
static inline void frame_from_motor(FrameMotor *fm, const GlobalMotor *m) {
    fm->displacement = m->target_pos;
    fm->speed        = m->target_vel;
    fm->accel        = m->target_acc;
    fm->state        = m->state ? 1 : 0;
}

/* 从 FrameMotor 写回 Motor */
static inline void frame_to_motor(GlobalMotor *m, const FrameMotor *fm) {
    m->target_pos = fm->displacement;
    m->target_vel = fm->speed;
    m->target_acc = fm->accel;
    m->state      = (fm->state != 0);
}


/* ================================================================
 *  调试打印
 * ================================================================ */

static void frame_print_hex(const uint8_t *d, int n) {
    for (int i = 0; i < n; i++) printf("%02X ", d[i]);
    printf("\n");
}

static void frame_print(const FrameData *f) {
    printf("┌── Frame ── fc=0x%02X motors=%d sensors=%d\n",
           f->func_code, f->motor_num, f->sensor_num);
    for (int i = 0; i < f->motor_num; i++)
        printf("│ motor[%d]  disp=0x%04X spd=0x%04X acc=0x%04X state=%d\n",
               i, f->motors[i].displacement, f->motors[i].speed,
               f->motors[i].accel, f->motors[i].state);
    for (int i = 0; i < f->sensor_num; i++)
        printf("│ sensor[%d] X=0x%04X Y=0x%04X Z=0x%04X\n",
               i, f->sensors[i].x, f->sensors[i].y, f->sensors[i].z);
    printf("│ scale1=0x%04X scale2=0x%04X sys_state=%d\n",
           f->scale1, f->scale2, f->sys_state);
    printf("└────────────\n");
}


#ifdef __cplusplus
}
#endif
#endif