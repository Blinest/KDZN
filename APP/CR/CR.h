#ifndef __CR_H
#define __CR_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
/* 驱动丝/段几何常量 (CR.h 和 SDM.h 共用) */
#define SDM_SEGMENTS        2
#define SDM_WIRES           6
#define SDM_WIRES_PER_SEG   3

/**********************************************************
***	编写作者：blinest

***	qq：1071378062
**********************************************************/

// 数学常数
#define PI 3.14159265358979323846
#define EPS 1e-9

typedef struct JointSpace
{
    float target_phi[SDM_SEGMENTS];
    float target_theta[SDM_SEGMENTS];
    float total_target_theta;
    float deltaL[SDM_WIRES];
} JointSpace;

typedef struct OperationSpace
{
    float scale;
}OperationSpace;

typedef struct ArmParams
{
    double L;                /**< 每段长度 (m) */
    double direction_gain[4]; /**< 方向增益，对应(u,r,d,l) */
} ArmParams;

typedef struct ContinuumRobot
{
    JointSpace joint_space;
    OperationSpace operation_space;
    float drive_radius_mm;   /**< 驱动丝半径 (mm) */
    ArmParams arm_params[2];
    bool state;
} ContinuumRobot;

void CR_init(void);
/** Execute one SDM step using current force/position feedback. */
void cr_kinematic_step(void);
uint8_t armBend(int seg, char direction, double val);
uint8_t armBend_edit(int seg, char direction, double val, double g_u, double g_r, double g_d, double g_l, double seg1_limit, double seg2_limit);
void deltaL_update(void);
void auto_straight(void);
void armRotate(float theta_deg, float step_deg);
void action_group_demo(void);
int direction_to_index(char direction);
double tendonCompensation(int seg, char direction, double angle_deg);

extern ContinuumRobot CR;

/* ==================== 通用运动学入口 ==================== */
/**
 * @param calc   运动学函数: (R, theta[], phi, deltaL[]) → 计算丝长变化
 * @param R      驱动丝半径 (mm)
 * @param theta  各段弯曲角 (rad), 长度 SDM_SEGMENTS
 * @param phi    各段弯曲方向 (rad), 长度 SDM_SEGMENTS
 */
void CR_kinematic_control(void (*calc)(float R, const float theta[], float phi, float deltaL[]),
                           float R, const float theta[], const float phi[]);

/* ==================== 压力灵敏度标定 ==================== */
/**
 * @brief 执行压力灵敏度自动标定
 *
 * 让每个电机前后移动 0.2mm，测量各通道压力变化幅值，
 * 计算归一化缩放系数 gain_scale[i]，使得各通道对相同 PID 输出
 * 产生一致的压力响应。
 */
void CR_calibrate_pressure_sensitivity(void);

/* ==================== 压力闭环控制 ==================== */
void CR_pressure_control(void);
void CR_pressure_export(float *target_out, int32_t *prev_val_out);
void CR_pressure_restore(const float *target, const int32_t *prev_val);

#endif
