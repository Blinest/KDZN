#ifndef __CR_H
#define __CR_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
/* 臂体几何常量 */
#define SEGMENT_COUNT       2
#define WIRE_COUNT          6
#define WIRES_PER_SEG       3

/**********************************************************
***	编写作者：blinest

***	qq：1071378062
**********************************************************/

// 数学常数
#define PI 3.14159265358979323846
#define EPS 1e-9

typedef struct JointSpace
{
    float target_phi[SEGMENT_COUNT];
    float target_theta[SEGMENT_COUNT];
    float total_target_theta;
    float deltaL[WIRE_COUNT];
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
/** 完整步进 — 含动力学力控模型 */
void cr_dynamics_step(void);
/** 纯运动学步进 — PCC逆运动学 + 力安全（无动力学修正） */
void CR_kinematic_control(void (*calc)(float R, const float theta[], float phi, float deltaL[]),
                           float R, const float theta[], const float phi[]);
void cr_kinematic_step(void);
uint8_t armBend(int seg, char direction, double val);
uint8_t armBend_edit(int seg, char direction, double val, double g_u, double g_r, double g_d, double g_l, double seg1_limit, double seg2_limit);
void deltaL_update(void);
void auto_straight(void);
void armRotate(float theta_deg, float step_deg);
void action_group_demo(void);
int direction_to_index(char direction);

/* ====================== 补偿模型 ===========================*/
double tendonCompensation(int seg, char direction, double angle_deg);

/* ==================== 压力灵敏度标定 ==================== */

void CR_calibrate_pressure_sensitivity(void);

/* ==================== 压力闭环控制 ==================== */
void CR_pressure_control(void);
void CR_pressure_export(float *target_out, int32_t *prev_val_out);
void CR_pressure_restore(const float *target, const int32_t *prev_val);

extern ContinuumRobot CR;

/* ==================== 压力控制参数 ==================== */
typedef struct {
    int32_t  high;            /**< 压力上限 */
    int32_t  low;             /**< 压力下限 */
    int32_t  delta;           /**< 最小力值变化阈值 */
    float    kp;              /**< PID 比例增益 */
    float    ki;              /**< PID 积分增益 */
    float    kd;              /**< PID 微分增益 */
    float    i_limit;         /**< 积分项限幅 */
    float    output_max;      /**< 单次最大输出 (mm) */
    float    calib_step;      /**< 标定步长 (mm) */
    uint32_t calib_settle_ms; /**< 标定稳定等待 (ms) */
    float    calib_thresh;    /**< 最小有效变化量 */
} CR_PressureConfig;

/** 默认压力控制参数实例 */
extern const CR_PressureConfig s_press_cfg;

#endif
