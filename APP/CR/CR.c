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

    CR.drive_radius_mm = 6.0f;
    motor_init();
    sensor_init();
    /* 臂体归中 */
   //auto_straight();
    /* 压力灵敏度自动标定 */
    //CR_calibrate_pressure_sensitivity();
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
void cr_get_tendon_forces(float forces[SENSOR_NUM])
{
    for (int i = 0; i < SENSOR_NUM; i++) {
        forces[i] = global_sensor[i].press_sensor.val;
    }
}

/** @brief 执行一步运动学 + 力控计算, 结果写入 CR.joint_space.deltaL */
void cr_kinematic_step(void)
{
    float forces[SENSOR_NUM];
    cr_get_tendon_forces(forces);

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

    float R = CR.drive_radius_mm / 1000.0f;

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

    /* 直接发绝对位置 0 给所有电机，不走 SDM 模型（SDM 在 theta=0 时输出为 0） */
    float zero_targets[SDM_WIRES] = {0};
    motor_sync_control(SDM_WIRES, 0, zero_targets);
    HAL_Delay(10);
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
    cr_kinematic_step();
    motor_sync_control(SDM_WIRES, 0, CR.joint_space.deltaL);
    osDelay(4000);

    // 2. 逐步旋转 phi
    for (float phi_deg = step_deg; phi_deg <= 360.0f; phi_deg += step_deg)
    {
        CR.joint_space.target_phi[0] = phi_deg * pi / 180.0f;
        CR.joint_space.target_phi[1] = phi_deg * pi / 180.0f;
        cr_kinematic_step();
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
    cr_kinematic_step();

    // 校验 + 驱动步进电机
    for (int i = 0; i < SDM_WIRES; i++) {
        if (isnan(CR.joint_space.deltaL[i]) || isinf(CR.joint_space.deltaL[i]))
            CR.joint_space.deltaL[i] = 0.0f;
    }
    motor_sync_control(SDM_WIRES, 0, CR.joint_space.deltaL);
    return 0;
}

/* ==================== 通用运动学入口 ==================== */

void CR_kinematic_control(void (*calc)(float R, const float theta[], float phi, float deltaL[]),
                           float R, const float theta[], const float phi[])
{
    float deltaL[SDM_WIRES] = {0};

    /* 调用运动学计算丝长 */
    calc(R, theta, phi[0], deltaL);

    /* NaN/Inf 保护 */
    for (int i = 0; i < SDM_WIRES; i++) {
        if (isnan(deltaL[i]) || isinf(deltaL[i]))
            deltaL[i] = 0.0f;
    }

    /* 同步驱动电机 */
    motor_sync_control(SDM_WIRES, 0, deltaL);
}

/* ==================== 压力闭环控制（PID同步模式） ==================== */

#define PRESS_HIGH       100      /**< 压力上限 */
#define PRESS_LOW        -100     /**< 压力下限 */
#define PRESS_DELTA      8        /**< 最小力值变化阈值 */

/* PID 参数 */
#define PID_KP           0.01f    /**< 比例增益 */
#define PID_KI           0.0f     /**< 积分增益（先关掉） */
#define PID_KD           0.0f     /**< 微分增益（先关掉） */
#define PID_ILIMIT       50.0f    /**< 积分项限幅 */
#define PID_OUTPUT_MAX   0.5f     /**< 单次最大输出 (mm) */

/* 压力灵敏度标定参数 */
#define CALIB_STEP       0.2f     /**< 标定步长 (mm) */
#define CALIB_SETTLE_MS  200      /**< 标定稳定等待 (ms) */
#define CALIB_THRESH     5        /**< 最小有效变化量，低于此视为传感器无响应 */

/**
 * @brief 每通道压力灵敏度缩放系数
 *
 * 标定后：gain_scale[i] 使得各通道对相同 PID 输出产生一致的压力响应。
 * 默认值为 1.0（未标定时无效）。
 */
static float s_gain_scale[MOTOR_NUM] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

/* 每通道 PID 状态 */
typedef struct {
    float integral;   /**< 积分累积 */
    float prev_err;   /**< 上次偏差（用于微分） */
    bool  initialized;
} PID_State;

static PID_State s_pid[MOTOR_NUM];

/* 状态变量 */
static float s_press_target[MOTOR_NUM];
static int32_t s_press_prev_val[MOTOR_NUM];
static bool s_pressure_state_restored = false;

void CR_pressure_export(float *target_out, int32_t *prev_val_out)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        target_out[i]   = s_press_target[i];
        prev_val_out[i] = s_press_prev_val[i];
    }
}

void CR_pressure_restore(const float *target, const int32_t *prev_val)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        s_press_target[i]   = target[i];
        s_press_prev_val[i] = prev_val[i];
    }
    s_pressure_state_restored = true;
}

/* ==================== 压力灵敏度自动标定 ==================== */

/**
 * @brief 压力灵敏度自动标定
 *
 * 原理：每个电机独立前/后移动 CALIB_STEP mm，记录压力变化峰峰值，
 *       计算缩放系数 gain_scale[i] = 基准幅值 / 实测幅值，
 *       使各通道对同量 PID 输出产生一致的压力变化。
 *
 * 标定结果会被 Flash 持久化保存（配合 param_save）。
 */
void CR_calibrate_pressure_sensitivity(void)
{
    float fwd[MOTOR_NUM];
    float bwd[MOTOR_NUM];
    float amp[MOTOR_NUM];
    float amp_max = 0.0f;
    float motor_pos0[MOTOR_NUM];

    /* 1. 记录电机当前位置 */
    for (int i = 0; i < MOTOR_NUM; i++) {
        motor_pos0[i] = global_motor[i].stepper_motor.current_pos;
    }

    /* 2. 统一前进 CALIB_STEP mm（压力增大方向） */
    {
        float targets[MOTOR_NUM];
        for (int i = 0; i < MOTOR_NUM; i++) {
            targets[i] = motor_pos0[i] + CALIB_STEP;
        }
        motor_sync_control(MOTOR_NUM, 0, targets);
    }
    osDelay(CALIB_SETTLE_MS);
    /* 等待 DataTask 读取到最新的传感器数据 */
    osDelay(300);

    for (int i = 0; i < MOTOR_NUM; i++) {
        fwd[i] = (float)global_sensor[i].press_sensor.val;
    }

    /* 3. 后退同样步长 */
    {
        float targets[MOTOR_NUM];
        for (int i = 0; i < MOTOR_NUM; i++) {
            targets[i] = motor_pos0[i] - CALIB_STEP;
        }
        motor_sync_control(MOTOR_NUM, 0, targets);
    }
    osDelay(CALIB_SETTLE_MS);
    /* 等待 DataTask 读取到最新的传感器数据 */
    osDelay(300);

    for (int i = 0; i < MOTOR_NUM; i++) {
        bwd[i] = (float)global_sensor[i].press_sensor.val;
    }

    /* 4. 计算每通道变化幅值（前进-后退的峰峰值） */
    for (int i = 0; i < MOTOR_NUM; i++) {
        amp[i] = fabsf(fwd[i] - bwd[i]);
        if (amp[i] > amp_max) amp_max = amp[i];
    }

    /* 5. 计算缩放系数 */
    if (amp_max >= CALIB_THRESH) {
        for (int i = 0; i < MOTOR_NUM; i++) {
            if (amp[i] >= CALIB_THRESH)
                s_gain_scale[i] = amp_max / amp[i];
            else
                s_gain_scale[i] = 1.0f;
        }
    } else {
        for (int i = 0; i < MOTOR_NUM; i++)
            s_gain_scale[i] = 1.0f;
    }

    /* 6. 回到初始位置 */
    {
        float targets[MOTOR_NUM];
        for (int i = 0; i < MOTOR_NUM; i++) {
            targets[i] = motor_pos0[i];
        }
        motor_sync_control(MOTOR_NUM, 0, targets);
    }
    osDelay(CALIB_SETTLE_MS);
}

/**
 * @brief PID 压力闭环控制 — 同步模式
 *
 * 对每个超限通道计算 PID 输出，一次指令内所有触发通道同步驱动。
 * 偏差 = 当前值 - 目标边界（PRESS_HIGH 或 PRESS_LOW）
 * 输出方向：力值偏大 → 回退电机，力值偏小 → 前进电机
 */
void CR_pressure_control(void)
{
    static bool inited = false;

    if (!inited) {
        if (!s_pressure_state_restored) {
            for (int i = 0; i < MOTOR_NUM; i++) {
                s_press_target[i]   = global_motor[i].stepper_motor.current_pos;
                s_press_prev_val[i] = global_sensor[i].press_sensor.val;
                s_pid[i].integral    = 0.0f;
                s_pid[i].prev_err    = 0.0f;
                s_pid[i].initialized = false;
            }
        }
        inited = true;
    }

    uint8_t trigger_idx[MOTOR_NUM];
    float   trigger_targets[MOTOR_NUM];
    int     update_count = 0;

    for (int i = 0; i < MOTOR_NUM; i++) {
        int32_t raw_val = global_sensor[i].press_sensor.val;

        /* 计算偏差：超出上限或下限的差值（带符号） */
        int32_t err = 0;
        if (raw_val > PRESS_HIGH)
            err = PRESS_HIGH - raw_val;    /* 负值，需要回退 */
        else if (raw_val < PRESS_LOW)
            err = PRESS_LOW - raw_val;     /* 正值，需要前进 */
        else
            continue;  /* 在范围内，不触发该通道 */

        int32_t delta = raw_val - s_press_prev_val[i];
        if (delta < 0) delta = -delta;

        if (delta >= PRESS_DELTA) {
            /* PID 计算 */
            float err_f = (float)err;

            /* P 项（含灵敏度归一化） */
            float p_out = PID_KP * err_f * s_gain_scale[i];

            /* I 项：偏差积分，带限幅 */
            if (s_pid[i].initialized) {
                s_pid[i].integral += err_f;
            } else {
                s_pid[i].integral = 0.0f;
                s_pid[i].initialized = true;
            }
            if (s_pid[i].integral > PID_ILIMIT)  s_pid[i].integral = PID_ILIMIT;
            if (s_pid[i].integral < -PID_ILIMIT) s_pid[i].integral = -PID_ILIMIT;
            float i_out = PID_KI * s_pid[i].integral;

            /* D 项 */
            float d_out = 0.0f;
            if (s_pid[i].initialized) {
                float derr = err_f - s_pid[i].prev_err;
                d_out = PID_KD * derr;
            }
            s_pid[i].prev_err = err_f;

            /* PID 输出合成 */
            float output = p_out + i_out + d_out;

            /* 输出限幅 */
            if (output > PID_OUTPUT_MAX)  output = PID_OUTPUT_MAX;
            if (output < -PID_OUTPUT_MAX) output = -PID_OUTPUT_MAX;

            s_press_target[i] += output;

            s_press_prev_val[i] = raw_val;

            trigger_idx[update_count] = i;
            trigger_targets[update_count] = s_press_target[i];
            update_count++;
        }
    }

    /* 第2步：仅对触发的通道做同步控制 */
    if (update_count > 0) {
        motor_sync_selective_control(update_count, trigger_idx, trigger_targets);
    }
}