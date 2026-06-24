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