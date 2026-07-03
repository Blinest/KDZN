/**
 *上层控制实现，用于处理上层指令解析和执行，提供电机控制和传感器数据读取等功能
 *功能包括：
 *1. 电机控制：基于运动学的电机控制
 *2. 传感器数据读取
 *3. 指令解析：解析上层指令，执行相应的操作，如控制电机、读取传感器数据等
 *4. 样机控制：根据指令控制样机的运动，如弯曲等
 *5. 错误处理：处理指令解析错误、通信错误等情况，确保系统稳定运行
 */

#include "CR.h"
#include "usart.h"
#include "Motor/Motor.h"
#include <stdio.h>
#include "math.h"
#include "Sensor/Sensor.h"
#include "SDM.h"
#include "cmsis_os2.h"

#define CR_THETA1_MAX 60
#define CR_THETA1_MIN -40
#define CR_THETA2_MAX 60
#define CR_THETA2_MIN -40
#define CR_ANGLE_RANGE 30
#define pi 3.1415926535

/*
 臂体补偿器
*/
bool tendon_comp = true;

/**********************************************************
***	编写作者：blinest

***	qq：1071378062
**********************************************************/

ContinuumRobot CR;

/** 默认 SDM 力控增益 (可外部修改) */
float sdm_K_force = 0.05f;

void CR_init(void)
{
    /* SDM 初始化: 半径 30mm, 刚度 0.2 N·m/rad, 力峰值 100N, 恢复 0.3, 尖端质量 0.15kg
     * 臂体水平安装，沿X轴方向 */
    CR.arm_params[0].L = 0.225;
    CR.arm_params[1].L = 0.225;
    float cable_r[SDM_SEGMENTS] = { 0.030f, 0.030f };
    float mount_dir[3] = { 1.0f, 0.0f, 0.0f };  /* 水平沿X轴 */
    sdm_init(cable_r, 0.2f, 100.0f, 0.3f, 0.15f, mount_dir);

    CR.operation_space.scale = 20;
    CR.joint_space.target_theta[0] = 0.5f;
    CR.joint_space.target_theta[1] = 0.3f;
    CR.joint_space.target_phi[0]   = 0.0f;
    CR.joint_space.target_phi[1]   = 0.0f;

    CR.parameter.r[0] = 70;
    CR.parameter.r[1] = 75;
    CR.parameter.r[2] = 80;
    motor_init();
    sensor_init();
}

// 用于控制喷管弯曲
uint8_t armBend(int seg, char direction, double val)
{
    if (seg == 1) {
        CR.joint_space.target_theta[0] = direction == 1 ? (float)val : -(float)val;
    } else {
        CR.joint_space.target_theta[1] = direction == 1 ? (float)val : -(float)val;
    }
    return armBend_edit(seg, direction, val, 0, 0, 0, 0, 90.0, 60.0);
}

/** @brief 从 global_sensor 提取 6 路肌腱力 (N) */
static void _get_forces(float forces[SENSOR_NUM])
{
    for (int i = 0; i < SENSOR_NUM; i++) {
        forces[i] = global_sensor[i].press_sensor.val;
    }
}

/** @brief 执行 SDM 一步控制, 结果写入 CR.joint_space.deltaL */
static void _sdm_run(void)
{
    float forces[SENSOR_NUM];
    _get_forces(forces);

    /* 力安全前置检查：任意一路超限则跳过本次运动 */
    float force_limit = sdm_get_force_peak_limit();
    for (int i = 0; i < SENSOR_NUM; i++) {
        if (forces[i] >= force_limit) {
            for (int j = 0; j < SDM_WIRES; j++)
                CR.joint_space.deltaL[j] = 0.0f;
            return;
        }
    }

    /* 从电机编码器读取实际位移反馈 */
    float deltaL_actual[SDM_WIRES];
    for (int i = 0; i < SDM_WIRES; i++) {
        deltaL_actual[i] = global_motor[i].stepper_motor.current_pos;
    }

    float R = CR.parameter.r[0] / 1000.0f;  /* mm → m */

    sdm_step(forces,
             CR.joint_space.target_theta,
             CR.joint_space.target_phi,
             deltaL_actual,
             sdm_K_force,
             R,
             CR.joint_space.deltaL);

    /* NaN/Inf 保护 */
    for (int i = 0; i < SDM_WIRES; i++) {
        if (isnan(CR.joint_space.deltaL[i]) || isinf(CR.joint_space.deltaL[i]))
            CR.joint_space.deltaL[i] = 0.0f;
    }
}

void auto_straight(void)
{
    for (int i = 0; i < SDM_SEGMENTS; i++) {
        CR.joint_space.target_theta[i] = 0;
        CR.joint_space.target_phi[i]   = 0;
    }
    CR.operation_space.scale = 0;
    _sdm_run();
    motor_sync_control(SDM_WIRES, 0, CR.joint_space.deltaL);
}

/**
 * @brief 臂体360度旋转（保持弯曲角度不变，phi从0旋转到2π）
 * @param theta_deg 弯曲角度（度），旋转过程中保持不变
 * @param step_deg  每步旋转角度（度），默认30度=12步完成一圈
 */
void armRotate(float theta_deg, float step_deg)
{
    if (theta_deg < 0) theta_deg = 0;
    if (theta_deg > 90) theta_deg = 90;
    if (step_deg <= 0) step_deg = 30.0f;

    float theta_rad = theta_deg * pi / 180.0f;

    // 1. 先弯曲到指定角度（phi=0），等待到位
    CR.joint_space.target_theta[0] = theta_rad / 2.0f;
    CR.joint_space.target_theta[1] = theta_rad;
    CR.joint_space.target_phi[0]   = 0;
    CR.joint_space.target_phi[1]   = 0;
    _sdm_run();
    motor_sync_control(SDM_WIRES, 0, CR.joint_space.deltaL);
    osDelay(4000);

    // 2. 逐步旋转 phi
    for (float phi_deg = step_deg; phi_deg <= 360.0f; phi_deg += step_deg)
    {
        CR.joint_space.target_phi[0] = phi_deg * pi / 180.0f;
        CR.joint_space.target_phi[1] = phi_deg * pi / 180.0f;
        _sdm_run();
        motor_sync_control(SDM_WIRES, 0, CR.joint_space.deltaL);
        osDelay(1000);
    }

    // 3. 归零
    auto_straight();
    osDelay(5000);
}

/**
 * @brief 动作组演示函数
 *
 * 执行顺序：360°旋转 → 上弯 → 回零 → 下弯 → 回零 → 左弯 → 回零 → 右弯 → 回零
 * 每个动作之间留有延时，确保运动完整执行
 * 所有动作参数在此函数内部定义
 */
void action_group_demo(void)
{
    const float angle = 30.0f;  // 弯曲角度（度）

    // 1. 360度旋转（保持30度弯曲）
    armRotate(30.0f, 30.0f);

    // 2. 向上弯曲
    armBend(1, 'u', angle);
    osDelay(3000);

    // 3. 回零
    auto_straight();
    osDelay(2000);

    // 4. 向下弯曲
    armBend(1, 'd', angle);
    osDelay(3000);

    // 5. 回零
    auto_straight();
    osDelay(2000);

    // 6. 向左弯曲
    armBend(1, 'l', angle);
    osDelay(3000);

    // 7. 回零
    auto_straight();
    osDelay(2000);

    // 8. 向右弯曲
    armBend(1, 'r', angle);
    osDelay(3000);

    // 9. 回零
    auto_straight();
    osDelay(2000);
}


int direction_to_index(char direction) {
    switch(direction) {
        case 'u': return 0;
        case 'r': return 1;
        case 'd': return 2;
        case 'l': return 3;
        default: return 0; // 默认返回'u'的索引
    }
}

// 补偿模型：基于力矩平衡实现
double tendonCompensation(int seg, char direction, double angle_deg)
{
    int dir_idx = direction_to_index(direction);
    double angle_rad = angle_deg * pi / 180.0;
    double dir_gain =  CR.arm_params[seg-1].direction_gain[dir_idx];
    double theta_ideal = angle_rad;
    // 摩擦引起的角度损失，理论上与肌腱张力成正比，由于目前没有张力反馈，不进行张力补偿，
    // double r = lqts.parameter.r / 1000;
    // double friction_loss_rad = friction_torque / (bending_stiffness_Nm2 + 0.001)  * (1.0 - exp(-angle_rad)); // 使用指数函数平滑

    //材料弹性引起的角度损失，考虑弹性恢复力矩，目前不需要考虑
    //double elastic_coeff = (lqts.arm_params[seg-1].backbone_stiffness / (lqts.arm_params[seg-1].L* lqts.arm_params[seg-1].L));
    //double elastic_loss = elastic_coeff * angle_rad * lqts.arm_params[seg-1].material_damping;

    // 大角度时的几何非线性补偿
    // 当弯曲时，肌腱的有效力臂会减小：R_eff = R * cos(theta/2)
    double geometric_factor = 1.0;
    if (angle_rad > 0.3) { // 大约17度以上开始考虑
    // 使用平滑过渡，避免突变
        double t = (angle_rad - 0.3) / 1.2; // 归一化到[0,1]，假设最大90度=1.57弧度

        geometric_factor = 1.0 + 0.15 * t * (1.0 - cos(angle_rad));
    }
    // 重力补偿
    double gravity_factor = 1.0;
    if (direction == 'u') {
        // 向上弯曲，对抗重力，需要额外补偿
        gravity_factor = 1.0 + 0.08 * (1.0 - cos(angle_rad));
    } else if (direction == 'd') {
        // 向下弯曲，重力辅助，可以减少补偿
        gravity_factor = 1.0 - 0.03 * (1.0 - cos(angle_rad));
    }


    double theta_compensated = theta_ideal * dir_gain * geometric_factor * gravity_factor;

    double max_ratio = 1.3;
    double min_ratio = 0.7;

    double min_allowed = min_ratio * angle_rad;
    double max_allowed = max_ratio * angle_rad;

    if (theta_compensated < min_allowed) {
        theta_compensated = min_allowed;
    }
    else if (theta_compensated > max_allowed) {
        theta_compensated = max_allowed;

    }
    return  theta_compensated;
}

uint8_t armBend_edit(int seg, char direction, double val, double g_u, double g_r, double g_d, double g_l, double seg1_limit, double seg2_limit)
{
    // 节段、角度限制检查
    if(seg != 1 && seg != 2) return 1;
    if (seg == 1 && (val > seg1_limit || val < 0)) return 1;
    if (seg == 2 && (val > seg2_limit || val < 0)) return 1;
    float val_rad = val * pi / 180.0;

    // 使用肌腱补偿器
    float compensated_angle_rad = 0;
    if(tendon_comp) {
       compensated_angle_rad = tendonCompensation(seg, direction, val);
    } else {
        compensated_angle_rad = val * pi / 180.0;
    }

    // 检查补偿后的角度是否超出安全范围
    double compensated_deg = compensated_angle_rad * 180.0 / pi;
    double max_angle = (seg == 1) ? 120.0 : 60.0;  // 允许一定的超调，目前第一段臂体可以超调到120°左右
    if (compensated_deg > max_angle) {
        compensated_angle_rad = max_angle * pi / 180.0;
    }

    // 设置 phi 角度，并进行简单的扭转补偿
    float phi = 0;
    switch (direction)
    {
        case 'u': phi = 0; break;
        case 'r': phi = pi / 2 - val_rad * g_r; break;
        case 'd': phi = pi;; break;
        case 'l': phi = 3 * pi / 2 + val_rad * g_l; break;
        default: return 1;
    }

    // 更新补偿后的关节角度 (两段模型)
    if (seg == 1) {
        CR.joint_space.target_theta[0] = compensated_angle_rad;
        CR.joint_space.target_phi[0] = phi;
    } else {
        CR.joint_space.target_theta[1] = compensated_angle_rad;
        CR.joint_space.target_phi[1] = phi;
    }
    _sdm_run();

    // 校验 + 驱动步进电机
    for (int i = 0; i < SDM_WIRES; i++) {
        if (isnan(CR.joint_space.deltaL[i]) || isinf(CR.joint_space.deltaL[i]))
            CR.joint_space.deltaL[i] = 0.0f;
    }
    motor_sync_control(SDM_WIRES, 0, CR.joint_space.deltaL);
    return 0;
}