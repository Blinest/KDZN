/**
 * @file cmd_packer.c
 * @brief 指令打包库
 *
 * 负责将全局结构体中的数据打包成数据包，发送给上位机
 *
 * @date 2026-03-30
 * @author blin
 */

#include "cmd_packer.h"
#include "cmsis_os2.h"

// 引用外部队列
extern osMessageQueueId_t SensorMessageQueueHandle;

/**
 * @brief 打包系统状态帧 (test_frame 格式)
 * @param frame 存储打包后的数据缓冲区
 * @param motor 电机全局结构体
 * @param sensor 传感器全局结构体
 * @param scale 缩放比例
 * @param state 系统状态
 * @return 打包后的总长度
 */
uint16_t cmd_packer_pack_status_frame(uint8_t* frame, GlobalMotor motor[MOTOR_NUM], GlobalSensor sensor[SENSOR_NUM], const ContinuumRobot *lqts, uint8_t state) {
    // 打包帧格式：帧头 + 功能码 + 数据长度 + 电机数量 + 传感器数量
	uint16_t idx = 0;
    frame[idx++] = 0xBB; // 帧头
    frame[idx++] = 0x02; // 功能码 (多传感器批量读取)

	// 预留数据长度字节的位置，稍后计算
	uint16_t data_length_pos = idx;
	idx += 1; // 为数据长度预留位置

    frame[idx++] = MOTOR_NUM;
    frame[idx++] = SENSOR_NUM;

    // 电机数据
    for (int i = 0; i < MOTOR_NUM; i++) {
        int16_t pos = (int16_t)(motor[i].stepper_motor.current_pos * 100); // mm
        int16_t vel = (int16_t)(motor[i].stepper_motor.current_vel * 100); // mm/s
        int16_t acc = (int16_t)(motor[i].stepper_motor.current_acc * 100); // mm/s^2
    	// 调试用 4.12
    	// int16_t pos = (int16_t)(motor[i].stepper_motor.target_pos * 100); // mm
    	// int16_t vel = (int16_t)(motor[i].stepper_motor.target_vel * 100); // mm/s
    	// int16_t acc = (int16_t)(motor[i].stepper_motor.current_acc * 100); // mm/s^2
    	int8_t motor_state = (int8_t)(motor[i].state);
    	// 填入数据包
        frame[idx++] = (pos >> 8) & 0xFF; frame[idx++] = pos & 0xFF;
        frame[idx++] = (vel >> 8) & 0xFF; frame[idx++] = vel & 0xFF;
        frame[idx++] = (acc >> 8) & 0xFF; frame[idx++] = acc & 0xFF;
    	frame[idx++] = motor_state & 0xFF;
    }
    // 传感器数据
    for (int i = 0; i < SENSOR_NUM; i++) {
        int32_t raw = sensor[i].press_sensor.raw_val;
        int32_t val = sensor[i].press_sensor.val;
        int32_t filter = sensor[i].press_sensor.filter_val;
        frame[idx++] = (raw >> 24) & 0xFF; frame[idx++] = (raw >> 16) & 0xFF;
        frame[idx++] = (raw >> 8) & 0xFF; frame[idx++] = raw & 0xFF;
        frame[idx++] = (val >> 24) & 0xFF; frame[idx++] = (val >> 16) & 0xFF;
        frame[idx++] = (val >> 8) & 0xFF; frame[idx++] = val & 0xFF;
        frame[idx++] = (filter >> 24) & 0xFF; frame[idx++] = (filter >> 16) & 0xFF;
        frame[idx++] = (filter >> 8) & 0xFF; frame[idx++] = filter & 0xFF;
    }
    // scale
    int16_t s_val = (int16_t)(lqts->operation_space.scale * 100);
    frame[idx++] = (s_val >> 8) & 0xFF; frame[idx++] = s_val & 0xFF;
	// armbend
	int16_t s_val2 = (int16_t)(lqts->joint_space.total_target_theta * 100);
	frame[idx++] = s_val2 >> 8; frame[idx++] = s_val2 & 0xFF;
    frame[idx++] = state; // 使用传入的 state

	// 计算数据长度（从电机数量字节开始到state字节的所有字节）
	// 数据长度 = 当前idx - 3 （因为0:帧头,1:功能码,2:数据长度）
	frame[data_length_pos] = (uint8_t)(idx-3);

    // 校验和
    uint16_t checksum = 0;
    for (int i = 0; i < idx; i++) {
        checksum += frame[i];
    }
    frame[idx++] = checksum & 0xFF;

    return idx;
}

/**
 * @brief 发送打包后的帧到队列
 * @param frame 打包后的数据帧
 * @param frame_len 帧长度
 */
void cmd_packer_send_frame_to_queue(uint8_t* frame, uint16_t frame_len) {
    for (int i = 0; i < frame_len; i++) {
        uint8_t msg = frame[i];
        osMessageQueuePut(SensorMessageQueueHandle, &msg, 0, 0);
    }
}

/**
 * @brief 发送系统状态帧到上位机
 * 这是供其他模块调用的高级接口
 */
void cmd_packer_send_status_frame(void) {
    uint8_t packed_frame[128];
	uint8_t state = CR.state;
    uint16_t frame_len = cmd_packer_pack_status_frame(packed_frame, global_motor, global_sensor, &CR, state);
    cmd_packer_send_frame_to_queue(packed_frame, frame_len);
}
