/**
* @file cmd_packer.h
 * @brief 指令打包库头文件
 *
 * 负责将全局结构体中的数据打包成数据包，发送给上位机
 *
 * @date 2026-03-30
 * @author blin
 */

#ifndef __CMD_PACKER_H
#define __CMD_PACKER_H

#include <stdint.h>
#include <stdbool.h>
#include "Motor/Motor.h"
#include "Sensor/Sensor.h"
#include "CR/CR.h"
#include "CR/Perception.h"
#ifdef __cplusplus
extern "C" {
#endif

	/* ================================================================
	 *  数据结构
	 * ================================================================ */

	/* 电机数据: X=位移, Y=速度, Z=加速度 */
	typedef struct {
		uint16_t displacement;  /* X: 位移 (编码值, ×0.01为实际值) */
		uint16_t speed;         /* Y: 速度 */
		uint16_t accel;         /* Z: 加速度 */
		uint8_t  state;         /* 使能状态 0/1 */
	} FrameMotor;

	/* 传感器数据: X, Y, Z */
	typedef struct {
		uint16_t x;
		uint16_t y;
		uint16_t z;
	} FrameSensor;

	/* 完整帧结构 */
	typedef struct {
		uint8_t     func_code;
		uint8_t     motor_num;
		uint8_t     sensor_num;
		FrameMotor  motor[MOTOR_NUM];
		FrameSensor sensor[SENSOR_NUM];
		uint16_t    scale1;
		uint16_t    scale2;
		uint8_t     sys_state;
	} FrameData;

	/**
	 * @brief 打包系统状态帧 (test_frame 格式)
	 * @param frame 存储打包后的数据缓冲区
	 * @param motor_pos 电机位置数组 [MOTOR_NUM][3]
	 * @param sensor_angle 传感器角度数组 [SENSOR_NUM][3]
	 * @param scale 缩放比例
	 * @param state 系统状态
	 * @return 打包后的总长度
	 */
	uint16_t cmd_packer_pack_status_frame(uint8_t* frame, GlobalMotor motor_pos[MOTOR_NUM], GlobalSensor sensor_angle[SENSOR_NUM], const ContinuumRobot *lqts, uint8_t state);

	/**
	 * @brief 打包六路张力感知帧
	 *
	 * 数据段 50 字节, 总帧长 54 字节 (含帧头/功能码/长度/校验)。
	 * 帧头由当前协议版本决定:
	 *   V2: [0x55][DEV_TENSION=0x02][FUNC=0x00][LEN=50][DATA...][SUM]
	 *   V1: [0xBB][0x03][LEN=50][DATA...][SUM]
	 *
	 * 数据段布局 (大端):
	 *   [0]      contact.detected (0/1)
	 *   [1]      contact.seg (0/1)
	 *   [2-3]    s_norm        ×10000  int16
	 *   [4-5]    disturbance   ×10000  int16 (无量纲份额不平衡)
	 *   [6-7]    F_seg[0]      ×10000  int16 (无量纲)
	 *   [8-9]    F_seg[1]      ×10000  int16
	 *   [10-11]  F_mag         ×100    int16 (N, 粗略)
	 *   [12-17]  health[0..5]          uint8 (0=OK 1=松弛 2=断丝 3=过张 4=过载)
	 *   [18-29]  share_err[0..5] ×10000 int16 ×6 = 12B
	 *   [30-41]  force[0..5]     ×100   int16 ×6 = 12B (N, 滤波+平衡后)
	 *   [42-47]  balance_gain[0..5]     uint8 ×6 (×100, 100=1.00)
	 *   [48]     calib_status (0=未运行 1=进行中 2=成功 3=失败)
	 *   [49]     calib_error  (0~7)
	 *
	 * @param frame 输出缓冲区 (≥54 字节)
	 * @param p     感知结果
	 * @return 打包后的总长度
	 */
	uint16_t cmd_packer_pack_percept_frame(uint8_t* frame, const PerceptState* p);

	/**
	 * @brief 打包握手应答帧 —— 恒定使用 V2 帧头
	 *
	 * [0x55][DEV_SYS=0x00][HANDSHAKE_ACK=0x00][LEN=4][ver][motor_n][sensor_n][flags][SUM]
	 *
	 * @param frame 输出缓冲区 (≥9 字节)
	 * @return 打包后的总长度
	 */
	uint16_t cmd_packer_pack_handshake_ack(uint8_t* frame);

	/**
	 * @brief 打包标定超时帧 (帧头 0xBB, 功能码 0x04) —— 由下位机主动上报
	 *
	 * 帧格式 (共 7 字节):
	 *   [0] 0xBB
	 *   [1] 0x04
	 *   [2] 数据长度 (=3)
	 *   [3] stage: 超时发生的阶段 (1=PRE_SETTLE 2=POST_MOVE 3=POST_SETTLE)
	 *   [4] err  : 错误码 (见 FbErr_t)
	 *   [5] mask : 「未到位」电机位掩码 (bit i = 电机 i)
	 *   [6] checksum (前 6 字节和取低 8 位)
	 *
	 * @param frame 输出缓冲区 (≥7 字节)
	 * @param stage 阶段号
	 * @param err   错误码
	 * @param mask  未到位电机掩码
	 * @return 打包后的总长度
	 */
	uint16_t cmd_packer_pack_fb_timeout_frame(uint8_t* frame, uint8_t stage,
	                                          uint8_t err, uint8_t mask);



	/**
	 * @brief 发送打包后的帧到队列
	 * @param frame 打包后的数据帧
	 * @param frame_len 帧长度
	 */
	void cmd_packer_send_frame_to_queue(uint8_t* frame, uint16_t frame_len);

	/**
	 * @brief 发送系统状态帧到上位机
	 * 这是供其他模块调用的高级接口
	 */
	void cmd_packer_send_status_frame(void);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_PACKER_H */