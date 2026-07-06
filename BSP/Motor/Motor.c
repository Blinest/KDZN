/**
 * @file Motor.c
 * @brief 电机指令处理模块
 *
 * 本模块仅完成电机基础控制（初始化、启停、位置/速度控制）。
 * 上层数据融合和压力闭环控制移至 CR 层（CR_pressure_control）。
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
#include "X_V2.h"

// 创建电机与电机反馈数据结构体
MotorFeedback motor_feedback[MOTOR_NUM];
GlobalMotor global_motor[MOTOR_NUM];

// ==================== 电机初始化 ====================

void motor_init()
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        global_motor[i].id = MOTOR_ID + i;
        global_motor[i].stepper_motor.daocheng = 1;
        global_motor[i].stepper_motor.xifen = 256;
        global_motor[i].stepper_motor.step_angle = 1.8;
        global_motor[i].stepper_motor.target_vel = 10;
        global_motor[i].stepper_motor.current_vel = 10;
        global_motor[i].vel_max = 120;
        global_motor[i].current_acc = 0;
    }
}

// ==================== 电机使能 ====================

void motor_enable(uint8_t addr, bool enable)
{
    X_V2_En_Control(addr, enable, 0);
}

// ==================== 绝对位置驱动（核心接口）====================

void motor_run(int idx, float vel, float target, uint8_t snf)
{
    const uint16_t xifen = global_motor[idx].stepper_motor.xifen;
    const float daocheng = global_motor[idx].stepper_motor.daocheng;
    const double step_angle = global_motor[idx].stepper_motor.step_angle;
    const int dir = target > 0 ? 0 : 1;

    float vel_rpm = vel * 60.0f / daocheng;
    uint16_t vel_rpm_abs = (uint16_t)(fabsf(vel_rpm) + 0.5f);
    float angle = 360.0f * target / daocheng;
    float angle_abs = fabsf(angle);
    uint32_t clk = (uint32_t)(angle_abs / step_angle * xifen);

    global_motor[idx].target_vel = vel_rpm;
    global_motor[idx].stepper_motor.target_vel = vel;
    global_motor[idx].target_pos = angle;
    global_motor[idx].stepper_motor.target_pos = target;

    uint16_t acc = 500;
    uint16_t dec = 500;
    X_V2_Traj_Pos_Control(global_motor[idx].id, dir, acc, dec, vel_rpm_abs, angle_abs, 1, snf);
}

// ==================== 速度模式（外部位置环用）====================

void motor_run_velocity_mode(uint8_t idx, float vel_rpm, uint16_t acc_rpm_s)
{
    uint8_t dir = (vel_rpm >= 0) ? 0 : 1;
    float abs_vel = fabsf(vel_rpm);
    X_V2_Vel_Control(global_motor[idx].id, dir, acc_rpm_s, abs_vel, false);
    global_motor[idx].target_vel = vel_rpm;
}

// ==================== 停止 ====================

void motor_stop_all()
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        X_V2_Stop_Now(global_motor[i].id, false);
        global_motor[i].state = 0;
    }
}

// ==================== 单电机绝对位置控制 ====================

void motor_single_control(uint8_t idx, uint8_t direction, float distance, float vel)
{
    float displacement = (direction == 0) ? distance : -distance;
    motor_run(idx, vel, displacement, 0);
}

// ==================== 多电机同步位置控制 ====================

void motor_sync_control(uint8_t count, uint8_t start_idx, float distance[])
{
    float max_distance = 0;
    uint16_t speed[MOTOR_NUM];

    for (int i = start_idx; i < count; i++)
    {
        float abs_distance = fabsf(distance[i]);
        max_distance = fmax(max_distance, abs_distance);
    }

    for (int i = start_idx; i < start_idx + count; i++)
    {
        float abs_distance = fabsf(distance[i - start_idx]);
        float ratio = (max_distance > 0) ? (abs_distance / max_distance) : 0;
        float vel_max = global_motor[i].vel_max / 60.0f * global_motor[i].stepper_motor.daocheng;
        float calculated_speed = ratio * vel_max;
        speed[i] = (calculated_speed == 0) ? (uint16_t)vel_max : (uint16_t)calculated_speed;
        global_motor[i].target_pos = distance[i - start_idx];
        global_motor[i].stepper_motor.target_vel = speed[i];
    }

    for (int i = start_idx; i < start_idx + count; i++)
    {
        motor_run(i, global_motor[i].stepper_motor.target_vel, global_motor[i].target_pos, true);

        /* 等待 TX FIFO 有空间（开启 AutoRetransmission 后由硬件保证送达） */
        uint32_t wait = 50000;
        while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < 3 && wait-- > 0) {
            for (volatile int d = 0; d < 48; d++);
        }
        if (wait == 0) {
            /* TX FIFO 满，忙等一帧时间（500kbps 下约 200μs/帧） */
            for (volatile int d = 0; d < 24000; d++);
            while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < 3) {
                for (volatile int d = 0; d < 4800; d++);
            }
        }
        for (volatile int d = 0; d < 24000; d++);  // ~50us 帧间隔
    }

    X_V2_Synchronous_motion(0);
    HAL_Delay(500);  // 等待同步运动完成
}

// ==================== 选择性多电机同步控制 ====================

void motor_sync_selective_control(uint8_t count, const uint8_t idx[], const float distance[])
{
    float max_distance = 0;
    uint16_t speed[MOTOR_NUM];

    for (int j = 0; j < count; j++)
    {
        float abs_distance = fabsf(distance[j]);
        max_distance = fmax(max_distance, abs_distance);
    }

    for (int j = 0; j < count; j++)
    {
        int i = idx[j];
        float abs_distance = fabsf(distance[j]);
        float ratio = (max_distance > 0) ? (abs_distance / max_distance) : 0;
        float vel_max = global_motor[i].vel_max / 60.0f * global_motor[i].stepper_motor.daocheng;
        float calculated_speed = ratio * vel_max;
        speed[i] = (calculated_speed == 0) ? (uint16_t)vel_max : (uint16_t)calculated_speed;
        global_motor[i].target_pos = distance[j];
        global_motor[i].stepper_motor.target_vel = speed[i];
    }

    for (int j = 0; j < count; j++)
    {
        int i = idx[j];
        motor_run(i, global_motor[i].stepper_motor.target_vel, global_motor[i].target_pos, true);

        uint32_t wait = 50000;
        while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < 3 && wait-- > 0) {
            for (volatile int d = 0; d < 48; d++);
        }
        if (wait == 0) {
            for (volatile int d = 0; d < 24000; d++);
            while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < 3) {
                for (volatile int d = 0; d < 4800; d++);
            }
        }
        for (volatile int d = 0; d < 24000; d++);
    }

    X_V2_Synchronous_motion(0);
    for (volatile int d = 0; d < 48000; d++);
}

// ==================== 角度 ↔ 位移 转换 ====================

float motor_angle_to_displacement(uint8_t motor_index, float angle)
{
    if (motor_index >= MOTOR_NUM) return 0.0f;

    StepperMotor *stepper = &global_motor[motor_index].stepper_motor;
    float steps_per_rev = 360.0f / stepper->step_angle * stepper->xifen;
    float steps = angle / 360.0f * steps_per_rev;
    float displacement = steps * stepper->daocheng / (360.0f / stepper->step_angle * stepper->xifen);

    stepper->current_pos = displacement;
    global_motor[motor_index].current_pos = angle * 180.0f / M_PI;
    return displacement;
}

float motor_displacement_to_angle(uint8_t motor_index, float displacement)
{
    if (motor_index >= MOTOR_NUM) return 0.0f;

    StepperMotor *stepper = &global_motor[motor_index].stepper_motor;
    float steps_per_rev = 360.0f / stepper->step_angle * stepper->xifen;
    float steps = displacement * steps_per_rev / stepper->daocheng;
    float angle = steps / steps_per_rev * 360.0f;

    stepper->current_pos = displacement;
    global_motor[motor_index].current_pos = angle * 180.0f / M_PI;
    return angle;
}

// ==================== 电机状态检查 ====================

void motor_status_check(void)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        X_V2_Read_Sys_Params(global_motor[i].id, S_CPOS);
        HAL_Delay(1);
        X_V2_Read_Sys_Params(global_motor[i].id, S_VEL);
        HAL_Delay(1);
    }
}
