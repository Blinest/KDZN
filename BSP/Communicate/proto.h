/**
 * @file proto.h
 * @brief 通信协议层（V2）—— 帧封装 + 设备 ID 扩展
 *
 * ---------------------------------------------------------------------------
 * 本工程已**完全迁移到 V2**，不再兼容 V1（0xBB 帧头 / 0x03~0x08 旧功能码）。
 *
 * 下行 (上位机 → 下位机):  0xAA
 *   [0] 0xAA | [1] 功能码 | [2] LEN | [3..] DATA | [..] SUM
 *
 * 上行 (下位机 → 上位机):  0x55
 *   [0] 0x55 | [1] DEV | [2] FUNC | [3] LEN | [4..] DATA | [..] SUM
 *              ↑
 *              设备/模块 ID：不同设备 → 上位机分派到不同渲染界面
 *
 *   SUM = 前面所有字节之和 & 0xFF
 *   LEN = 数据段字节数 (不含帧头/功能码/长度/校验)
 *
 * ---------------------------------------------------------------------------
 * 扩展方式:
 *   1) 新设备只需分配一个 DEV ID, 上位机按 DEV 路由到对应界面, 内核不需改动;
 *   2) 设备内新增帧类型用该设备下的 FUNC 编号, 互不干扰;
 *   3) 预分配区间: 0x00~0x0F 核心自有 | 0x10~0x7F 外设 |
 *                  0x80~0xFE 实验/厂商扩展 | 0xFF 全局广播
 *
 * @date 2026-10-04
 */
#ifndef __PROTO_H
#define __PROTO_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 帧头 ==================== */
#define PROTO_DOWNLINK_MAGIC   0xAAu   /* 上位机 → 下位机 */
#define PROTO_UPLINK_MAGIC     0x55u   /* 下位机 → 上位机 */

/* 协议版本 (用于握手应答的 ver 字段) */
#define PROTO_VERSION          2u

/* ==================== 上行设备 / 模块 ID ====================
 * 上位机凭此字段把帧分派给不同界面渲染。 */
typedef enum {
    PROTO_DEV_SYS     = 0x00,  /* 主控系统: 握手应答 / ACK / 错误 */
    PROTO_DEV_DRIVE   = 0x01,  /* 电机驱动: 系统状态帧 */
    PROTO_DEV_TENSION = 0x02,  /* 张力感知: 感知帧 */
    PROTO_DEV_CALIB   = 0x03,  /* 标定: 事件帧 */
    PROTO_DEV_IMU     = 0x04,  /* 预留: 姿态/IMU */
    PROTO_DEV_END     = 0x05,  /* 预留: 末端/喷管 */
    /* 0x10 ~ 0x7F 外设 | 0x80 ~ 0xFE 实验扩展 | 0xFF 全局 */
} ProtoDevice_t;

/* ==================== 各设备下的功能码 ==================== */
#define PROTO_F_SYS_HANDSHAKE_ACK  0x00u  /* DEV_SYS     : 握手应答 */
#define PROTO_F_SYS_ACK            0x01u  /* DEV_SYS     : 通用 ACK/NAK */
#define PROTO_F_SYS_ERROR          0xFFu  /* DEV_SYS     : 错误告警 */

#define PROTO_F_DRIVE_STATUS       0x00u  /* DEV_DRIVE   : 系统状态帧  L=85 */
#define PROTO_F_TENSION_PERCEPT    0x00u  /* DEV_TENSION : 张力感知帧  L=50 */
#define PROTO_F_CALIB_EVENT        0x00u  /* DEV_CALIB   : 标定事件帧  L=3  */

/* ==================== 下行功能码 (0xAA) ====================
 * 按类别分组: 0x0_ 系统 | 0x1_ 电机运动 | 0x2_ 传感器
 *             0x3_ 配置 | 0x4_ 动作   | 0x5_ 标定            */

/* --- 0x0_ 系统 --- */
#define PROTO_C_SYS_MOTOR_OFF      0x00u  /* 6 路电机失能 (无数据段) */
#define PROTO_C_SYS_MOTOR_ON       0x01u  /* 6 路电机使能 (无数据段) */
#define PROTO_C_SYS_MOTOR_STOP     0x02u  /* 6 路急停     (无数据段) */
#define PROTO_C_SYS_HANDSHAKE      0x09u  /* 版本/能力查询: client_ver(1) */

/* --- 0x1_ 电机运动 --- */
#define PROTO_C_MOTOR_SINGLE       0x10u  /* addr(1) dir(1) dist(2)×100 vel(2)×100 */
#define PROTO_C_MOTOR_SYNC         0x11u  /* count(1) start_addr(1) n×int16 ×100 */
#define PROTO_C_MOTOR_CUSTOM       0x12u  /* count(1) n×[addr(1) dir(1) dist(2)×100] */
#define PROTO_C_MOTOR_DEMO         0x13u  /* 动作组演示 (无数据段) */

/* --- 0x2_ 传感器 --- */
#define PROTO_C_SENSOR_INIT        0x20u  /* (无数据段) */
#define PROTO_C_SENSOR_READ        0x21u  /* id(1) */
#define PROTO_C_SENSOR_READ_ALL    0x22u  /* (无数据段) */
#define PROTO_C_SENSOR_CAL         0x23u  /* id(1) weight(2) 单位 g×10 */
#define PROTO_C_PERCEPT_ZERO       0x24u  /* 张力感知零点标定 (无数据段) */

/* --- 0x3_ 配置 --- */
#define PROTO_C_SET_MOUNT          0x30u  /* dx(2) dy(2) dz(2), 单位向量 ×1000 */
#define PROTO_C_GRAV_COMP          0x31u  /* gain(2) ×1000 */

/* --- 0x4_ 动作 --- */
/* 单段弯曲: seg(1) dir(1) angle(2)×100  (dir: 0=朝上, 1=朝下) */
#define PROTO_C_SINGLE_BEND        0x40u

/* --- 0x5_ 标定 (预留) --- */
#define PROTO_C_CALIB_BALANCE      0x50u  /* pull(2)×100 mm (预留) */

/* ==================== 帧封装 ==================== */

/** 计算并追加校验和: 把 out[0..len-1] 之和的低 8 位写入 out[len], 返回 len+1 */
uint16_t proto_append_checksum(uint8_t *out, uint16_t len);

/**
 * @brief 封装一帧上行数据
 * @param out     输出缓冲区 (建议 >= n + 5)
 * @param dev     设备 ID (ProtoDevice_t)
 * @param func    该设备下的功能码
 * @param payload 数据段 (可为 NULL, n=0)
 * @param n       数据段字节数 (<= 255)
 * @return 总帧长; 参数非法返回 0
 */
uint16_t proto_pack_uplink(uint8_t *out, uint8_t dev, uint8_t func,
                           const uint8_t *payload, uint16_t n);

#ifdef __cplusplus
}
#endif

#endif /* __PROTO_H */
