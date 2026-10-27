//
// Created by blin on 2026/3/7.
//
/**
* @file motor.c
 * @brief 电机指令处理模块
 *
 * 本模块提供电机指令处理功能：
 * - motor_init()：电机初始化，初始化流程包括电机参数设置、
 * - motor_run()：启动电机，并设置绝对目标位置
 * - motor_position_control_snf()：
 * - motor_emergency_stop_all()：紧急停止所有电机
 *
 */
#include "Motor.h"

#include "math.h"
#include <stdio.h>
#include "usart.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include "X_V2.h"
#include "CR/SDM.h"
#include "CR/CR.h"

// 创建电机与电机反馈数据结构体
MotorFeedback motor_feedback[MOTOR_NUM];
GlobalMotor global_motor[MOTOR_NUM];

//电机初始化函数
void motor_init()
{
	// 初始化电机相关参数
	for (int i = 0; i < MOTOR_NUM; i++)
	{
		global_motor[i].id = MOTOR_ID + i;
		global_motor[i].stepper_motor.daocheng = 1; // 根据丝杠导程设置，1mm
		global_motor[i].stepper_motor.xifen = 256; // 平滑控制
		global_motor[i].stepper_motor.step_angle = 1.8; // 步距角
		global_motor[i].stepper_motor.target_vel = 3; // 默认线速度 mm/s
		global_motor[i].stepper_motor.current_vel = 10;
		/* 最大线速度 (mm/s) = 5.0, 按导程 1mm 折算最大转速 = 5*60/1 = 300 rpm。
		 * 最大转速不单独配置, 由 motor_run() 按 rpm = mm_per_s * 60 / 导程 推导。 */
		global_motor[i].vel_max = 5;
		global_motor[i].current_acc = 0; // 由于位移量较小，为提高响应速度，直接启动，不做加减速处理 (0-255)
	}
}

/**
 *
 * @param addr : 电机地址
 * @param enable : 电机是否使能
 */

void motor_enable(uint8_t addr,bool enable)
{
	// 更新电机状态，(电机使能状态 0x02)
	X_V2_En_Control(addr, enable, 0);
}

/**
  * @brief 启动步进电机，并达到指定位置（带限制条件）
  * @param idx: 电机索引
  * @param vel 速度值, mm/s
  * @param target: 目标位置(绝对位置), mm
  * @param snf: 同步标志位，true同步
  */
void motor_run(int idx, float vel, float target, uint8_t snf) {

	// ==================== 执行控制 ====================
	const uint16_t xifen = global_motor[idx].stepper_motor.xifen;
	const float daocheng = global_motor[idx].stepper_motor.daocheng;
	const double step_angle = global_motor[idx].stepper_motor.step_angle;
	// 方向确定
	const int dir = target > 0 ? 0 : 1;

	// 速度计算: 入参 vel 是线速度 mm/s; 最大转速 rpm 由本式推导 (导程是唯一换算依据)
	const float vel_rpm = vel * 60.0f / daocheng ;
	const uint16_t vel_rpm_abs = (uint16_t)(fabsf(vel_rpm) + 0.5f);

	// 位置计算
	const float angle = 360.0f  * target / daocheng;
	float angle_abs = fabsf(angle);

	// 脉冲数计算
	const uint32_t clk = (uint32_t)(angle_abs / step_angle * xifen);
	(void)clk;  /* 保留：X_V2_Pos_Control 备用路径使用 */

	// 更新电机状态
	global_motor[idx].target_vel = vel_rpm;
	global_motor[idx].stepper_motor.target_vel = vel;
	global_motor[idx].target_pos = angle;
	global_motor[idx].stepper_motor.target_pos = target;
	// 测试用
	// float val = target;
	// int int_part = (int)val;
	// int frac_part = (int)((val - int_part) * 100 + 0.5);  // 保留两位小数，四舍五入
	// if (frac_part < 0) frac_part = -frac_part;  // 小数部分取绝对值
	// if (frac_part >= 100) {  // 处理进位，如 1.999 -> 2.00
	// 	int_part += 1;
	// 	frac_part -= 100;
	// }

	// char test[32];
	// int len = snprintf(test, sizeof(test), "%d.%02d", int_part, frac_part);
	// if (len > 0 && len < sizeof(test)) {
	// 	Usart_SendString(&huart1, (uint8_t*)test, len);
	// } else {
	// 	Usart_SendString(&huart1, (uint8_t*)"ERR_FMT\r\n", 9);
	// }
	// 直通限速位置模式
	// X_V2_Bypass_Pos_LV_Control(global_motor[idx].id, dir, vel_rpm_abs, angle_abs, 1, snf);
	// X_V2_Pos_Control(global_motor[idx].id, dir, vel_rpm_abs, 0, clk, 1, snf);
	// 加速度和减速度 (RPM/s)，根据实际系统调整
	uint16_t acc = 500;   // 加速斜率
	uint16_t dec = 500;   // 减速斜率

	// 使用梯形曲线加减速位置模式
	X_V2_Traj_Pos_Control(global_motor[idx].id, dir, acc, dec, vel_rpm_abs, angle_abs, 1, snf);

}

/**
 * @brief 速度模式驱动电机（用于外部位置环）
 * @param idx       电机索引
 * @param vel_rpm   目标速度 (RPM)，可为正或负，内部自动处理方向
 * @param acc_rpm_s 加速度 (RPM/s)
 */
void motor_run_velocity_mode(uint8_t idx, float vel_rpm, uint16_t acc_rpm_s) {
	uint8_t dir = (vel_rpm >= 0) ? 0 : 1;
	float abs_vel = fabsf(vel_rpm);
	// 使用限电流版本可选
	X_V2_Vel_Control(global_motor[idx].id, dir, acc_rpm_s, abs_vel, false);
	// 记录当前目标速度
	global_motor[idx].target_vel = vel_rpm;
}

/**
 * 多电机停止函数
 *
 */
void motor_stop_all()
{
	for(int i = 0; i < MOTOR_NUM; i++) {
		X_V2_Stop_Now(global_motor[i].id, false);
		global_motor[i].state = 0;
	}
}

/**
 * @brief 单电机控制函数（绝对位置控制）
 * @param idx : 电机索引号
 * @param direction ：电机旋转方向
 * @param distance ：步进电机移动距离 mm
 * @param vel ：步进电机位移速度 mm/s
 */
void motor_single_control(uint8_t idx, uint8_t direction, float distance, float vel)
{
	// 位置换算，0正1负
	float displacement = (direction == 0) ? distance : -distance;

	// 1. 检查所有限制条件 (使用默认速度)：解除限制 4.9
	//float default_speed = global_motor[idx].vel_max * 0.5f; // 使用50%最大速度
	//if (vel > default_speed) vel = default_speed;
	// float val = vel;
	// int int_part = (int)val;
	// int frac_part = (int)((val - int_part) * 100 + 0.5);  // 保留两位小数，四舍五入
	// if (frac_part < 0) frac_part = -frac_part;  // 小数部分取绝对值
	// if (frac_part >= 100) {  // 处理进位，如 1.999 -> 2.00
	// 	int_part += 1;
	// 	frac_part -= 100;
	// }
	//
	// char test[32];
	// int len = snprintf(test, sizeof(test), "%d.%02d", int_part, frac_part);
	// if (len > 0 && len < sizeof(test)) {
	// 	Usart_SendString(&huart1, (uint8_t*)test, len);
	// } else {
	// 	Usart_SendString(&huart1, (uint8_t*)"ERR_FMT\r\n", 9);
	// }

	// 调用底层控制函数
	motor_run(idx, vel, displacement, 0);
}

// 多电机同步控制函数
/**
 *
 * @param count :电机数量
 * @param start_idx ：电机初始索引值
 * @param distance ：电机移动距离数组
 */
void motor_sync_control(uint8_t count, uint8_t start_idx, float distance[])
{
	float max_distance = 0;
	uint16_t speed[MOTOR_NUM];

	// 边界裁剪：防止越界访问 global_motor[] / speed[] / distance[]
	if (start_idx >= MOTOR_NUM) return;
	if (count > (uint8_t)(MOTOR_NUM - start_idx)) {
		count = (uint8_t)(MOTOR_NUM - start_idx);
	}
	if (count == 0) return;

	// 求本批次的最大绝对位移，作为速度分配的基准
	for (uint8_t i = 0; i < count; i++)
	{
		max_distance = fmaxf(max_distance, fabsf(distance[i]));
	}

	// ==================== 速度分配 ====================
	// 动态速度调整：线性比例控制，考虑每个电机的最大速度限制
	for (int i = start_idx; i < start_idx + count; i++)
	{
		float abs_distance = fabsf(distance[i-start_idx]);
		float ratio = (max_distance > 0) ? (abs_distance / max_distance) : 0;

		// 步进电机速度分配计算
		/* vel_max 语义统一为「线速度 mm/s」; 最大转速 rpm 由 motor_run()
		 * 用 rpm = mm_per_s * 60 / 导程 推导 (导程变化时只需改导程, 不必再折算上限)。
		 * [2026-10-27] 原写法 vel_max/60*daocheng 把 vel_max 当成 rpm 用,
		 * 导程改成 1mm 后把上限压到 120/60*1 = 2 mm/s。 */
		float vel_max = (float)global_motor[i].vel_max;

		// 计算当前步进电机速度
		float calculated_speed = ratio * vel_max;
		speed[i] =  calculated_speed == 0? (uint16_t)vel_max: (uint16_t)calculated_speed;
		// 储存目标电机位移与速度
		global_motor[i].target_pos = distance[i-start_idx];
		global_motor[i].stepper_motor.target_vel = speed[i];
	}

	// ==================== 执行同步控制 ====================
	// 最后一个电机用于其他控制
	for (int i = start_idx; i < start_idx + count; i++)
	{
		motor_run(i, global_motor[i].stepper_motor.target_vel, global_motor[i].target_pos, true);

		// 等待当前电机的 3 帧全部发出，避免 burst 堵塞
		uint32_t wait = 50000; // ~1ms @480MHz
		while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < 3 && wait-- > 0) {
			for (volatile int d = 0; d < 48; d++); // ~100ns
		}
		if (wait == 0) {
			// CAN 堵塞 → 等待后重启恢复
			HAL_FDCAN_Stop(&hfdcan1);
			osDelay(20);  // 等待总线错误帧消散
			HAL_FDCAN_Start(&hfdcan1);
			// 重新激活中断通知
			HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
			HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, 0);
		}

		// 帧组间延迟：给总线时间发送当前电机的帧，再发下一个电机
		osDelay(2);  // 2ms，500kbps 下足够发完 3 帧（~0.7ms）
	}

	// 触发同步控制
	X_V2_Synchronous_motion(0);
	osDelay(10);
}

/**
 *
 * @param kinematic : 运动学函数，接入不同运动学模型
 * @param R : 半径 mm
 * @param theta : 弯曲角 rad
 * @param phi : 旋转角 rad
 * @param deltaL : 变化长度 mm
 */
void motor_kinematic_control (Kinematic kinematic, float R[], float theta[], float phi, float deltaL[])
{
	// ==================== 计算肌腱长度变化 ====================
	kinematic(R, theta, phi, deltaL);

	// ==================== 检查计算结果 ====================
	for (int i = 0; i < MOTOR_NUM; i++) {
		if (isnan(deltaL[i]) || isinf(deltaL[i])) {

			deltaL[i] = 0.0f; // 设为0防止错误传播
		}
	}

	// ==================== 执行同步控制 ====================
	/* 注意: motor_sync_control 的第 2 个参数是 global_motor[] 的数组索引(0 基),
	 * 不是电机地址; 这里必须是 (MOTOR_NUM, 0)。
	 * [修正 2026-10-15] 原为 (9, MOTOR_ID, deltaL):
	 *   - count=9 超范围(会被裁剪);
	 *   - start_idx=MOTOR_ID 在 MOTOR_ID=6 时 >= MOTOR_NUM, 函数直接 return,
	 *     运动学控制完全失效。 */
	motor_sync_control(MOTOR_NUM, 0, deltaL);
}

/**
 * @brief 将步进电机的角度信息转换为位移信息
 * @param motor_index: 电机索引
 * @param angle: 角度信息（单位：度）
 * @return 位移信息（单位：mm）
 */
float motor_angle_to_displacement(uint8_t motor_index, float angle)
{
    if (motor_index >= MOTOR_NUM) {
        return 0.0f;
    }

    StepperMotor *stepper = &global_motor[motor_index].stepper_motor;

    // 计算每转的步数
    float steps_per_rev = 360.0f / stepper->step_angle * stepper->xifen;

    // 计算角度对应的步数
    float steps = angle / 360.0f * steps_per_rev;

    // 计算位移：步数 * 导程 / 每转步数
    float displacement = steps * stepper -> daocheng / (360.0f / stepper->step_angle * stepper->xifen);

    // 更新电机结构体中的位置信息
    stepper -> current_pos = displacement;
    global_motor[motor_index].current_pos = angle * 180.0f / 3.1415926f; // 转换为弧度并存储

    return displacement;
}

/**
 * @brief 将位移信息转换为步进电机的角度信息
 * @param motor_index: 电机索引
 * @param displacement: 位移信息（单位：mm）
 * @return 角度信息（单位：度）
 */
float motor_displacement_to_angle(uint8_t motor_index, float displacement)
{
    if (motor_index >= MOTOR_NUM) {
        return 0.0f;
    }

    StepperMotor *stepper = &global_motor[motor_index].stepper_motor;

    // 计算每转的步数
    float steps_per_rev = 360.0f / stepper->step_angle * stepper->xifen;

    // 计算位移对应的步数
    float steps = displacement * steps_per_rev / stepper->daocheng;

    // 计算角度：步数 / 每转步数 * 360度
    float angle = steps / steps_per_rev * 360.0f;

    // 更新电机结构体中的位置信息
    stepper->current_pos = displacement;
    global_motor[motor_index].current_pos = angle * 180.0f / 3.1415926f; // 转换为弧度并存储

    return angle;
}

// ==================== 电机状态定期检查 ====================

/**
 * @brief 电机状态定期检查函数
 *
 * 应定期调用（如每100ms），检查电机状态和限制条件
 */
void motor_status_check(void)
{

    for (int i = 0; i < MOTOR_NUM; i++) {
    	X_V2_Read_Sys_Params(global_motor[i].id, S_CPOS);
    	osDelay(1); // 延时等待响应
    	X_V2_Read_Sys_Params(global_motor[i].id, S_VEL);
    	osDelay(1); // 延时等待响应
    	/* 0x3A 状态标志位 (bit0 = 使能状态) → 回调解入 global_motor[].state,
    	 * 供 motor_update_enable_state() 判断 CR.state */
    	X_V2_Read_Sys_Params(global_motor[i].id, S_FLAG);
    	osDelay(1); // 延时等待响应
    }
}

/**
 * @brief 依据 6 路电机的「实际使能状态」刷新 CR.state
 *
 * 判据: global_motor[0..5].state 全部为 true → CR.state = 1;
 *       只要有任何一路未使能 → CR.state = 0。
 *
 * state 的来源是电机的真实回复:
 *   - 0xF3 使能命令的回复 [F3][status][6B], status==0x02 视为使能成功;
 *   - 0x3A 状态标志位回复 (bit0 = 使能状态), 由 motor_status_check() 每 200ms 轮询刷新。
 *
 * 因此 CR.state 只会在"电机真的使能"之后才置 1, 掉线/失能会立刻回到 0。
 */
void motor_update_enable_state(void)
{
	bool all_enabled = true;

	for (int i = 0; i < MOTOR_NUM; i++) {
		if (!global_motor[i].state) { all_enabled = false; break; }
	}
	CR.state = all_enabled;
}

void motor_read_positions(void)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        X_V2_Read_Sys_Params(global_motor[i].id, S_CPOS);
        osDelay(1);   // 等响应回来 (由 CAN 回调解入 stepper_motor.current_pos)
    }
}

uint8_t motor_reached_mask(const float pos0[MOTOR_NUM], float delta_mm,
                           float pos_tol, float vel_tol)
{
    uint8_t mask = 0;
    const float need = fabsf(delta_mm) - pos_tol;
    if (need <= 0.0f) return 0;      /* 指令位移小于容差, 无需等待 */

    for (int i = 0; i < MOTOR_NUM; i++) {
        const float moved = fabsf(global_motor[i].stepper_motor.current_pos - pos0[i]);
        const float vel   = fabsf(global_motor[i].stepper_motor.current_vel);
        if (moved < need || vel > vel_tol) mask |= (uint8_t)(1u << i);
    }
    return mask;
}

bool motor_all_reached(const float pos0[MOTOR_NUM], float delta_mm,
                       float pos_tol, float vel_tol)
{
    return motor_reached_mask(pos0, delta_mm, pos_tol, vel_tol) == 0;
}
