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
	 * @brief 打包系统状态帧
	 *
	 * 数据段 73 字节, 总帧长 78 字节 (V2: 0x55 + DEV + FUNC + LEN + DATA + SUM)。
	 *   V2: [0x55][DEV_DRIVE=0x01][FUNC=0x00][73][数据段][SUM]
	 *
	 * 数据段布局 (大端):
	 *   [0]     电机数量 (6)
	 *   [1]     传感器数量 (6)
	 *   [2-43]  6×电机, 每路 7 B: pos(2) vel(2) acc(2) state(1), ×0.01 mm / mm·s⁻¹ / mm·s⁻²
	 *   [44-67] 6×传感器, 每路 4 B: x(2) y(2), ×0.01 N
	 *           x = 滤波+通道平衡后张力; y = 原始值
	 *   [68-69] scale   int16 ×100
	 *   [70-71] armbend int16 ×100
	 *   [72]    state   uint8
	 *
	 * @note 2026-10-15: 张力感知数据段已全部去除, 状态帧只含
	 *       6 路电机 + 6 路传感器 + CR 的 scale/armbend/state。
	 *
	 * @param frame 输出缓冲区 (≥78 字节)
	 * @return 打包后的总长度
	 */
	uint16_t cmd_packer_pack_status_frame(uint8_t* frame, GlobalMotor motor_pos[MOTOR_NUM], GlobalSensor sensor_angle[SENSOR_NUM], const ContinuumRobot *lqts, uint8_t state);

	/* 2026-10-15: cmd_packer_pack_percept_frame() 已移除, 张力感知数据不再上报 */

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