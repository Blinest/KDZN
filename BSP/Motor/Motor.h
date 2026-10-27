//
// Created by blin on 2026/3/7.
//

#ifndef CONTROLSYSTEM_MOTOR_H
#define CONTROLSYSTEM_MOTOR_H
#include "stdint.h"

#include <stdbool.h>
#include <stdint.h>
/**********************************************************
***	编写作者：Lin

***	qq：1071378062
**********************************************************/

#define MOTOR_NUM 6 // 定义电机数量
#define MOTOR_ID 7// 定义电机起始 ID 

// ==================== Emm_V5 步进闭环：反馈指令帧结构体 ====================
// 帧格式以 `0x6B` 结尾（文档称“校验字节”，发送端固定为 0x6B）
// 常见 ACK：addr + func + status + 0x6B
// status：0x02=正确，0xEE=错误（按你提供的表）

typedef enum {
	EMM_STATUS_OK  = 0x02,
	EMM_STATUS_ERR = 0xEE,
} Emm_Status_t;

typedef struct {
	uint8_t addr;   // 地址
	uint8_t func;   // 功能码
	uint8_t status; // 02:正确, EE:错误
	uint8_t check;  // 固定 0x6B
} Emm_AckFrame_t;

// 读取电机目标位置（功能码 0x33）
// 正常返回：addr + 0x33 + sign + pos(4字节, 大端) + 0x6B
// 错误返回：addr + 0x33 + 0xEE + 0x6B
typedef struct {
	uint8_t addr;      // 地址
	uint8_t func;      // 0x33
	uint8_t sign;      // 符号字节（表格里写“符号(01)”）
	uint8_t pos_be[4]; // 目标位置，4字节大端
	uint8_t check;     // 固定 0x6B
} Emm_posFrame_t;

typedef union {
	Emm_AckFrame_t  ack;
	Emm_posFrame_t tpos;
	uint8_t raw[16]; // 兜底：保留原始字节
} Emm_FeedbackFrame_t;

// 4字节大端转 int32（避免依赖编译器对 inline/C99 的支持）
#define EMMV5_POS_BE_TO_I32(pos_be) \
((int32_t)( \
((uint32_t)((pos_be)[0]) << 24) | \
((uint32_t)((pos_be)[1]) << 16) | \
((uint32_t)((pos_be)[2]) << 8)  | \
((uint32_t)((pos_be)[3]) << 0)  \
))

// ==================== 电机外设反馈数据结构体 ====================
typedef struct MotorFeedback {
    uint8_t addr;          // 电机地址
    uint8_t func;          // 功能码
	uint8_t total_byte;
	uint8_t configure;
	uint8_t vol; //电压
	uint16_t current; //电流
	uint16_t encoding; //磁编码器值
	uint8_t motor_data_target[3];
    int16_t motor_data_cur[3];   // pos, vel, acc 数据 (单位: 0.01mm, 0.01mm/s, 0.01mm/s^2),目前电机反馈只有位置和速度，无法实现加速度
    uint8_t state;         // 电机状态 (02: 运行, E2:无响应 ,EE: 异常)，判断时出问题直接判断EE即可
} MotorFeedback;

// ==================== 步进电机参数 ====================

typedef struct
{
	uint8_t daocheng;
	uint16_t xifen;
	float step_angle;
	float current_pos; // mm
	float target_pos; // mm
	float current_vel; // mm/s
	float target_vel; // mm/s
	float current_acc; //mm/s^2
} StepperMotor;

typedef struct ServoMotor
{
	uint8_t daocheng;
	uint8_t xifen;
	float step_angle;
	float current_pos; // mm
	float target_pos; // mm
	float current_vel; // mm/s
	float target_vel; // mm/s
} ServoMotor;

// ==================== 全局电机结构体 ====================
typedef struct GlobalMotor
{
	int id;
	bool state;
	StepperMotor stepper_motor;
	ServoMotor servo_motor;
	uint8_t last_response_time;
	uint8_t timeout_threshold;
	float current_pos; // deg (由 0x36 实时位置/0x33 目标位置回复写入)
	float current_vel; // rpm
	float current_acc; // rad/s^2
	float target_pos;
	float target_vel;
	float target_acc;
	float vel_max;
	uint8_t size;
	uint8_t cmd[32];
} GlobalMotor;

void motor_init();
void motor_run(int idx, float vel, float target, uint8_t snf);
void motor_enable(uint8_t addr, bool enable);
void motor_stop_all();
void motor_single_control(uint8_t idx, uint8_t direction, float distance, float vel);
void motor_sync_control(uint8_t count, uint8_t start_idx, float distance[]);
typedef void (*Kinematic)(float R[], float theta[], float phi, float deltaL[]);
void motor_kinematic_control(Kinematic kinematic, float R[], float theta[], float phi, float deltaL[]);

// 新增函数 - 添加于2026-03-27 by Psyduck
void motor_status_check(void);

/**
 * @brief 依据 6 路电机的实际使能状态刷新 CR.state
 *
 * 全部电机 state==true → CR.state = 1; 有任何一路未使能 → CR.state = 0。
 * state 来自电机真实回复(0xF3 回复 status==0x02 / 0x3A 状态标志位 bit0),
 * 所以 CR.state 只会在「电机真的使能」之后才置 1。
 */
void motor_update_enable_state(void);

/**
 * @brief 读取 6 路电机实时位置 (仅位置, 比 motor_status_check 轻)
 * @note  阻塞 ~6ms (每路 osDelay(1) 等响应); 响应由 CAN 回调解入
 *        global_motor[].stepper_motor.current_pos (mm)
 */
void motor_read_positions(void);

/**
 * @brief 检查 6 路电机是否都已完成指令位移并停下
 *
 * 用于标定等「必须确认到位才能继续」的场合。
 * 判据 (6 路全部满足):
 *   ① |current_pos - pos0| >= |delta_mm| - pos_tol   (位移量到位)
 *   ② |current_vel|        <= vel_tol                (已停稳)
 *
 * @param pos0     指令下发时刻的位置基准 (mm, [MOTOR_NUM])
 * @param delta_mm 指令位移 (mm, 带符号)
 * @param pos_tol  到位容差 (mm)
 * @param vel_tol  判定停止的速度上限 (mm/s)
 * @return true = 6 路全部到位
 */
bool motor_all_reached(const float pos0[MOTOR_NUM], float delta_mm,
                       float pos_tol, float vel_tol);

/**
 * @brief 返回「未到位」电机位掩码 (bit i = 电机 i 未到位)
 * @return 0 表示全部到位
 */
uint8_t motor_reached_mask(const float pos0[MOTOR_NUM], float delta_mm,
                           float pos_tol, float vel_tol);

float motor_angle_to_displacement(uint8_t motor_index, float angle);
float motor_displacement_to_angle(uint8_t motor_index, float displacement);

extern GlobalMotor global_motor[MOTOR_NUM];
extern MotorFeedback motor_feedback[MOTOR_NUM];
#endif //CONTROLSYSTEM_MOTOR_H