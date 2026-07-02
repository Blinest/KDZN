/**
 * @file Motor.c
 * @brief 电机指令处理模块
 *
 * 本模块提供电机指令处理功能：
 * - motor_init()：电机初始化
 * - motor_run()：启动电机，并设置绝对目标位置
 * - motor_pressure_control()：基于压力传感器反馈的闭环控制
 * - motor_emergency_stop_all()：紧急停止所有电机
 *
 * @date 2026-03-07
 * @author blin
 */
#include "Motor.h"

#include "math.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#include <stdio.h>
#include "usart.h"
#include "cmsis_os2.h"
#include "fdcan.h"
// #include "Emm_V5.h"
#include "X_V2.h"
#include "CR/SDM.h"

/* ==================== 压力闭环控制参数 ==================== */
#define PRESS_HIGH      200      /**< 压力上限 */
#define PRESS_LOW         0      /**< 压力下限 */
#define PRESS_DELTA      10      /**< 最小力值变化阈值，低于此值视为无明显变化 */
#define PRESS_STEP_MIN   0.2f    /**< 最小步长 (mm)，精细调整 */
#define PRESS_STEP_MAX   2.0f    /**< 最大步长 (mm)，快速响应 */
#define PRESS_VEL        0.5f    /**< 调整速度 (mm/s) */

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
		global_motor[i].stepper_motor.target_vel = 10; // 如果要完成指标，至少是 10mm/s
		global_motor[i].stepper_motor.current_vel = 10;
		global_motor[i].vel_max = 120; // 满足指标要求
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

	// 速度计算
	const float vel_rpm = vel * 60.0f / daocheng ;
	const uint16_t vel_rpm_abs = (uint16_t)(fabsf(vel_rpm) + 0.5f);

	// 位置计算
	const float angle = 360.0f  * target / daocheng;
	float angle_abs = fabsf(angle);

	// 脉冲数计算
	const uint32_t clk = (uint32_t)(angle_abs / step_angle * xifen);

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

	// 检查每个电机的位移限制
	for (int i = start_idx; i < count; i++)
	{
		float displacement = distance[i];
		// 计算绝对位移用于速度分配
		float abs_distance = fabsf(displacement);
		max_distance = fmax(max_distance, abs_distance);
	}

	// ==================== 速度分配 ====================
	// 动态速度调整：线性比例控制，考虑每个电机的最大速度限制
	for (int i = start_idx; i < start_idx + count; i++)
	{
		float abs_distance = fabsf(distance[i-start_idx]);
		float ratio = (max_distance > 0) ? (abs_distance / max_distance) : 0;

		// 步进电机速度分配计算
		float vel_max = global_motor[i].vel_max / 60.0f * global_motor[i].stepper_motor.daocheng;

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
	motor_sync_control(9, MOTOR_ID, deltaL);
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
    global_motor[motor_index].current_pos = angle * 180.0f / M_PI;

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
    global_motor[motor_index].current_pos = angle * 180.0f / M_PI;

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
    }
}

/**
 * @brief 基于压力传感器反馈的电机独立控制（动态步长）
 *
 * 控制逻辑（双条件触发 + 动态步长）：
 *   1. 力值超出正常范围 [PRESS_LOW, PRESS_HIGH]
 *   2. 力值相对上次触发点的变化量 >= PRESS_DELTA
 *
 * 动态步长算法：
 *   step = STEP_MIN + (STEP_MAX - STEP_MIN) × (delta - PRESS_DELTA) / (PRESS_HIGH - PRESS_DELTA)
 *   限制在 [STEP_MIN, STEP_MAX] 范围内
 *
 *   delta 刚过阈值 → 小步长（精细调整）
 *   delta 接近 PRESS_HIGH → 大步长（快速响应）
 *
 * - val > PRESS_HIGH + 变化显著: 电机前进（释放压力）
 * - val < PRESS_LOW  + 变化显著: 电机后退（增加张力）
 */

/* 模块级静态变量，供 export/restore 访问 */
static float s_press_target[MOTOR_NUM];
static int32_t s_press_prev_val[MOTOR_NUM];
static bool s_pressure_state_restored = false;  /* true=已从 Flash 恢复，跳过默认初始化 */

void motor_pressure_export(float *target_out, int32_t *prev_val_out)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        target_out[i]   = s_press_target[i];
        prev_val_out[i] = s_press_prev_val[i];
    }
}

void motor_pressure_restore(const float *target, const int32_t *prev_val)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        s_press_target[i]   = target[i];
        s_press_prev_val[i] = prev_val[i];
    }
    s_pressure_state_restored = true;
}

/**
 * @brief 根据力值变化幅度计算动态步长
 *
 * 线性映射: delta [PRESS_DELTA, PRESS_HIGH] → step [STEP_MIN, STEP_MAX]
 *
 * @param delta 力值变化幅度（绝对值）
 * @return 步长 (mm)
 */
static float _calc_dynamic_step(int32_t delta)
{
    /* delta 归一化到 [0, 1] */
    float ratio = (float)(delta - PRESS_DELTA) / (float)(PRESS_HIGH - PRESS_DELTA);
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    /* 线性插值 */
    float step = PRESS_STEP_MIN + (PRESS_STEP_MAX - PRESS_STEP_MIN) * ratio;
    return step;
}

void motor_pressure_control(void)
{
    static bool inited = false;

    if (!inited) {
        /* 如果未从 Flash 恢复，则用当前值初始化 */
        if (!s_pressure_state_restored) {
            for (int i = 0; i < MOTOR_NUM; i++) {
                s_press_target[i]   = global_motor[i].stepper_motor.current_pos;
                s_press_prev_val[i] = global_sensor[i].press_sensor.filter_val;
            }
        }
        inited = true;
    }

    for (int i = 0; i < MOTOR_NUM; i++) {
        int32_t val = global_sensor[i].press_sensor.filter_val;
        int32_t delta = val - s_press_prev_val[i];
        if (delta < 0) delta = -delta;

        /* 条件1: 力值超出正常范围 */
        bool out_of_range = (val > PRESS_HIGH) || (val < PRESS_LOW);
        /* 条件2: 力值相对上次触发有明显变化 */
        bool significant_change = (delta >= PRESS_DELTA);

        if (out_of_range && significant_change) {
            /* 动态步长：变化越大，步长越大 */
            float step = _calc_dynamic_step(delta);

            if (val > PRESS_HIGH) {
                s_press_target[i] -= step;  // 压力过高 → 前进释放
            } else {
                s_press_target[i] += step;  // 压力过低 → 后退收紧
            }
            s_press_prev_val[i] = val;
            motor_run(i, PRESS_VEL, s_press_target[i], 0);
        }
    }
}
