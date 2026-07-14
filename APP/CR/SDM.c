/**
 * @file SDM.c
 * @brief 分段微分模型 — 内部持有状态，对外只暴露 sdm_init/sdm_step。
 *
 * 控制流 (sdm_step):
 *   1. 力安全因子
 *   2. PCC 逆运动学 → 基准丝长 deltaL_base[6]
 *   3. 肌腱力误差 → k_ratio[6]
 *   4. deltaL_out[i] = deltaL_base[i] × k_ratio[i]
 *
 * @date 2026-06-19
 * @author blinest
 */

#include "SDM.h"
#include "CR.h"
#include "Motor/Motor.h"
#include "usart.h"
#include <math.h>
#include <string.h>

/* ==================== 内部常量 ==================== */
#define SDM_PI      3.14159265358979323846f
#define SDM_PI_3    1.0471975511965976f
#define SDM_2PI_3   2.0943951023931953f
#define SDM_4PI_3   4.1887902047863905f
#define SDM_5PI_3   5.235987755982988f
#define SDM_EPS     1e-9f
#define SDM_M_TO_MM 1000.0f
#define SDM_MM_TO_M 0.001f

/* ==================== 内部状态 ==================== */
typedef struct {
    float bending_stiffness;
    float force_peak_limit;
    float force_recovery;
    float tip_mass;
    float gravity;
    float R_mount[3][3];    /**< 臂体初始安装姿态 (世界→基座局部) */
    SDM_DynamicsConfig dynamics;
} SDM_InternalParams;

static SDM_InternalParams s_params = {0};
static bool s_initialized = false;

/* k_ratio 不对外暴露 */
static float s_k_ratio[SDM_WIRES];
static float s_force_target[SDM_WIRES];
static bool s_force_target_valid = false;

/* 段 → 驱动丝索引 */
static const int s_seg_to_wires[SDM_SEGMENTS][SDM_WIRES_PER_SEG] = {
    { 0, 2, 4 },   /* 段0: 丝0, 丝2, 丝4 */
    { 1, 3, 5 }    /* 段1: 丝1, 丝3, 丝5 */
};

/**
 * @brief 将浮点数限制在指定范围内（钳位函数）
 *
 * 如果输入值 v 小于下限 lo，则返回 lo；
 * 如果 v 大于上限 hi，则返回 hi；
 * 否则返回 v 本身。
 * 常用于防止数值溢出、越界或保证输出在有效区间内。
 *
 * @param v  输入值（待钳位的浮点数）
 * @param lo 下限（最小值）
 * @param hi 上限（最大值）
 * @return   钳位后的浮点数，范围 [lo, hi]
 */
static inline float _clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/**
 * @brief 从臂体初始方向向量构建安装姿态矩阵 R_mount
 *
 * 给定臂体中心线在世界坐标系中的方向 dir，
 * 构建 R_mount 使得局部Z轴 = dir 方向。
 *
 * @param dir  臂体初始方向 (世界坐标系, 单位向量, 长度3)
 * @param R    输出: 3×3 旋转矩阵
 */
static void _build_mount_matrix(const float dir[3], float R[3][3])
{
    /* 局部Z轴 = 臂体方向 */
    float z[3] = { dir[0], dir[1], dir[2] };

    /* 局部X轴: 取世界Z轴×z，若平行则取世界X轴×z */
    float ref[3] = { 0.0f, 0.0f, 1.0f };
    float cross = z[0]*ref[0] + z[1]*ref[1] + z[2]*ref[2];
    if (fabsf(cross) > 0.9f) {
        ref[0] = 1.0f; ref[1] = 0.0f; ref[2] = 0.0f;
    }
    float x[3] = {
        ref[1]*z[2] - ref[2]*z[1],
        ref[2]*z[0] - ref[0]*z[2],
        ref[0]*z[1] - ref[1]*z[0]
    };
    /* 归一化 */
    float len = sqrtf(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);
    if (len < SDM_EPS) len = 1.0f;
    x[0] /= len; x[1] /= len; x[2] /= len;

    /* 局部Y轴 = z × x (右手系) */
    float y[3] = {
        z[1]*x[2] - z[2]*x[1],
        z[2]*x[0] - z[0]*x[2],
        z[0]*x[1] - z[1]*x[0]
    };

    /* R_mount: 列 = [X轴, Y轴, Z轴] 在世界坐标系中的方向 */
    R[0][0] = x[0]; R[0][1] = y[0]; R[0][2] = z[0];
    R[1][0] = x[1]; R[1][1] = y[1]; R[1][2] = z[1];
    R[2][0] = x[2]; R[2][1] = y[2]; R[2][2] = z[2];
}

/* ==================== PCC 逆运动学 (内部) ==================== */
static void _calculate_L_m(float R, const float theta[SDM_SEGMENTS],
                           const float phi[SDM_SEGMENTS], float deltaL[SDM_WIRES])
{
    deltaL[0] = -R * theta[0] * cosf(phi[0]);
    deltaL[2] = -R * theta[0] * cosf(phi[0] + SDM_2PI_3);
    deltaL[4] = -R * theta[0] * cosf(phi[0] + SDM_4PI_3);

    deltaL[1] = -R * theta[0] * cosf(phi[0] + SDM_PI_3)
                + R * theta[1] * cosf(phi[1] + SDM_PI_3);
    deltaL[3] = -R * theta[0] * cosf(phi[0] + SDM_PI)
                - R * theta[1] * cosf(phi[1] + SDM_PI);
    deltaL[5] = -R * theta[0] * cosf(phi[0] + SDM_5PI_3)
                - R * theta[1] * cosf(phi[1] + SDM_5PI_3);
}

static void _calculate_L(float R, const float theta[SDM_SEGMENTS],
                         const float phi[SDM_SEGMENTS], float deltaL[SDM_WIRES])
{
    _calculate_L_m(R, theta, phi, deltaL);
    for (int i = 0; i < SDM_WIRES; i++) {
        deltaL[i] *= SDM_M_TO_MM;
    }
}

/* ==================== PCC 正解反推 (内部) ==================== */

/**
 * @brief 从实际丝长变化反解 θ, φ
 *
 * 三根丝 120° 等间隔分布的 PCC 反解公式:
 *   设 ψ = φ + α₀ (α₀ 为第一根丝的截面角)
 *   c₀ = θ cos(ψ)
 *   c₁ = θ cos(ψ + 2π/3)
 *   c₂ = θ cos(ψ + 4π/3)
 *
 *   θ = sqrt(c₀² + (c₂ - c₁)² / 3)
 *   ψ = atan2((c₂ - c₁) / √3, c₀)
 *   φ = ψ - α₀
 *
 * 段1 耦合段0, 先减去段0贡献再反解。
 *
 * @param deltaL_actual  实际 6 根丝长度变化 (mm)
 * @param R              驱动丝半径 (m)
 * @param theta_out      输出: 各段弯曲角 (rad, [2])
 * @param phi_out        输出: 各段弯曲方向角 (rad, [2])
 */
static void _inverse_kinematics(const float deltaL_actual[SDM_WIRES],
                                 float R,
                                 float theta_out[SDM_SEGMENTS],
                                 float phi_out[SDM_SEGMENTS])
{
    if (R < SDM_EPS) R = 0.03f;
    float sqrt3_inv = 1.0f / sqrtf(3.0f);

    /* Motor feedback and public SDM displacement values are millimetres. */
    float deltaL_m[SDM_WIRES];
    for (int i = 0; i < SDM_WIRES; i++) {
        deltaL_m[i] = deltaL_actual[i] * SDM_MM_TO_M;
    }

    /* ——— 段0: 丝0,2,4 角度 {0, 2π/3, 4π/3} ——— */
    float c0 = -deltaL_m[0] / R;
    float c1 = -deltaL_m[2] / R;
    float c2 = -deltaL_m[4] / R;

    float diff = c2 - c1;
    theta_out[0] = sqrtf(c0 * c0 + diff * diff / 3.0f);
    phi_out[0]   = atan2f(diff * sqrt3_inv, c0);

    /* ——— 段1: 丝1,3,5 角度 {π/3, π, 5π/3} ——— */
    /* 先减去段0对该段三根丝的贡献 */
    float s0_contribution_1 = -R * theta_out[0] * cosf(phi_out[0] + SDM_PI_3);
    float s0_contribution_3 = -R * theta_out[0] * cosf(phi_out[0] + SDM_PI);
    float s0_contribution_5 = -R * theta_out[0] * cosf(phi_out[0] + SDM_5PI_3);

    /* 段1 独立贡献 (注意丝3,5 的公式里段1项是负号) */
    float d0 = (deltaL_m[1] - s0_contribution_1) / R;   /* θ₁ cos(φ₁ + π/3) */
    float d1 = -(deltaL_m[3] - s0_contribution_3) / R;  /* θ₁ cos(φ₁ + π) */
    float d2 = -(deltaL_m[5] - s0_contribution_5) / R;  /* θ₁ cos(φ₁ + 5π/3) */

    diff = d2 - d1;
    theta_out[1] = sqrtf(d0 * d0 + diff * diff / 3.0f);
    float psi1   = atan2f(diff * sqrt3_inv, d0);
    phi_out[1]   = psi1 - SDM_PI_3;
}

/* ==================== 力安全因子 (内部) ==================== */
/**
 * @brief 力安全监控函数
 * 遍历所有传感器通道，找出最大力值，据此计算安全系数并判断是否过载。
 *
 * @param forces     输入：传感器力值数组
 * @param safety     输出：计算得到的安全系数，取值范围被钳制在
 *                   [s_params.force_recovery, 1.0] 之间。
 *                   1.0 表示完全安全，越接近 force_recovery 表示
 *                   力值越接近峰值限制，需要降低驱动输出。
 * @param over_peak  输出：过载标志。当任一通道力值达到或超过
 *                   s_params.force_peak_limit 时置为 true，
 *                   否则为 false。
 */
static void _force_safety(const float forces[SENSOR_NUM],
                           float *safety, bool *over_peak)
{
    float f_max = 0.0f;
    // 最大力值获取
    for (int i = 0; i < SENSOR_NUM; i++) {
        if (forces[i] > f_max) f_max = forces[i];
    }
    // 过载标志
    *over_peak = (f_max >= s_params.force_peak_limit);
    // 安全系数解算
    *safety = _clampf(1.0f - f_max / s_params.force_peak_limit,
                      s_params.force_recovery, 1.0f);
}

/* ==================== Cosserat/PCC quasi-static dynamics ==================== */

/* 各段每根丝的角度 (相对于局部X轴, 与 _calculate_L 公式一致) */
static const float s_wire_alpha[SDM_SEGMENTS][SDM_WIRES_PER_SEG] = {
    { 0.0f,    SDM_2PI_3, SDM_4PI_3 },   /* 段0: 丝0=0°, 丝2=120°, 丝4=240° */
    { SDM_PI_3, SDM_PI,   SDM_5PI_3 }    /* 段1: 丝1=60°, 丝3=180°, 丝5=300° */
};

static void _mat3_mul(const float A[3][3], const float B[3][3], float C[3][3])
{
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            C[r][c] = 0.0f;
            for (int k = 0; k < 3; k++) C[r][c] += A[r][k] * B[k][c];
        }
}

static void _mat3_vec(const float A[3][3], const float v[3], float out[3])
{
    for (int r = 0; r < 3; r++)
        out[r] = A[r][0] * v[0] + A[r][1] * v[1] + A[r][2] * v[2];
}

static void _mat3_t_vec(const float A[3][3], const float v[3], float out[3])
{
    for (int c = 0; c < 3; c++)
        out[c] = A[0][c] * v[0] + A[1][c] * v[1] + A[2][c] * v[2];
}

static void _cross3(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static void _pcc_pose(float theta, float phi, float length,
                      float p[3], float R_local[3][3])
{
    float cp = cosf(phi), sp = sinf(phi);
    float ct = cosf(theta), st = sinf(theta);
    R_local[0][0] = cp*cp*(ct-1.0f)+1.0f;
    R_local[0][1] = sp*cp*(ct-1.0f);
    R_local[0][2] = cp*st;
    R_local[1][0] = sp*cp*(ct-1.0f);
    R_local[1][1] = cp*cp*(1.0f-ct)+ct;
    R_local[1][2] = sp*st;
    R_local[2][0] = -cp*st;
    R_local[2][1] = -sp*st;
    R_local[2][2] = ct;
    if (fabsf(theta) < 1e-6f) {
        p[0] = 0.0f; p[1] = 0.0f; p[2] = length;
    } else {
        float radius = length / theta;
        p[0] = radius * (1.0f - ct) * cp;
        p[1] = radius * (1.0f - ct) * sp;
        p[2] = radius * st;
    }
}

static void _moment_to_tensions(int segment, const float moment_xy[2], float R,
                                float tensions[SDM_WIRES_PER_SEG])
{
    float min_t = 1e30f;
    float inv = 2.0f / (3.0f * fmaxf(R, 1e-6f));
    for (int j = 0; j < SDM_WIRES_PER_SEG; j++) {
        float alpha = s_wire_alpha[segment][j];
        tensions[j] = inv * (-sinf(alpha) * moment_xy[0]
                             + cosf(alpha) * moment_xy[1]);
        if (tensions[j] < min_t) min_t = tensions[j];
    }
    float common = s_params.dynamics.tendon_pretension;
    if (min_t < common) common -= min_t;
    else common = 0.0f;
    for (int j = 0; j < SDM_WIRES_PER_SEG; j++)
        tensions[j] = _clampf(tensions[j] + common, 0.0f,
                              s_params.force_peak_limit);
}

/** Section moment balance -> non-negative tendon target forces. */
static void _dynamics_model(const float theta_actual[SDM_SEGMENTS],
                            const float phi_actual[SDM_SEGMENTS],
                            float R,
                            float f_target_out[SDM_WIRES])
{
    float base_p[SDM_SEGMENTS][3] = {{0}};
    float com_p[SDM_SEGMENTS][3] = {{0}};
    float end_p[SDM_SEGMENTS][3] = {{0}};
    float base_R[SDM_SEGMENTS][3][3];
    float R_acc[3][3];
    float p_acc[3] = {0.0f, 0.0f, 0.0f};
    memcpy(R_acc, s_params.R_mount, sizeof(R_acc));

    /* PCC geometry supplies r(s) and R(s) for the section load balance. */
    for (int s = 0; s < SDM_SEGMENTS; s++) {
        float L = (float)CR.arm_params[s].L;
        if (L < SDM_EPS) L = 0.225f;
        memcpy(base_R[s], R_acc, sizeof(R_acc));
        memcpy(base_p[s], p_acc, sizeof(p_acc));
        float p_mid_l[3], p_end_l[3], R_mid[3][3], R_local[3][3];
        _pcc_pose(0.5f * theta_actual[s], phi_actual[s], 0.5f * L,
                  p_mid_l, R_mid);
        _pcc_pose(theta_actual[s], phi_actual[s], L, p_end_l, R_local);
        float p_mid_w[3], p_end_w[3];
        _mat3_vec(R_acc, p_mid_l, p_mid_w);
        _mat3_vec(R_acc, p_end_l, p_end_w);
        for (int k = 0; k < 3; k++) {
            com_p[s][k] = p_acc[k] + p_mid_w[k];
            end_p[s][k] = p_acc[k] + p_end_w[k];
            p_acc[k] = end_p[s][k];
        }
        float R_new[3][3];
        _mat3_mul(R_acc, R_local, R_new);
        memcpy(R_acc, R_new, sizeof(R_acc));
    }

    /* Backward integration of section moments: distal loads affect all bases. */
    float required[SDM_SEGMENTS][2] = {{0}};
    for (int s = SDM_SEGMENTS - 1; s >= 0; s--) {
        float Mg_world[3] = {0.0f, 0.0f, 0.0f};
        for (int load = s; load < SDM_SEGMENTS; load++) {
            float mass = fmaxf(s_params.dynamics.segment_mass[load], 0.0f);
            float arm[3], force[3] = {0.0f, 0.0f, -mass * s_params.gravity};
            float moment[3];
            for (int k = 0; k < 3; k++) arm[k] = com_p[load][k] - base_p[s][k];
            _cross3(arm, force, moment);
            for (int k = 0; k < 3; k++) Mg_world[k] += moment[k];
        }
        if (s_params.tip_mass > 0.0f) {
            float arm[3], force[3] = {0.0f, 0.0f,
                                      -s_params.tip_mass * s_params.gravity};
            float moment[3];
            for (int k = 0; k < 3; k++)
                arm[k] = end_p[SDM_SEGMENTS - 1][k] - base_p[s][k];
            _cross3(arm, force, moment);
            for (int k = 0; k < 3; k++) Mg_world[k] += moment[k];
        }

        float Mg_local[3];
        _mat3_t_vec(base_R[s], Mg_world, Mg_local);
        float L = (float)CR.arm_params[s].L;
        if (L < SDM_EPS) L = 0.225f;
        float kappa = fabsf(theta_actual[s]) / L;
        /* Stable constitutive law from the derivation: EI(0) > 0. */
        float EI = s_params.bending_stiffness * L
                   + s_params.dynamics.curvature_stiffness * kappa * kappa;
        EI = fmaxf(EI, SDM_EPS);
        float Cb = EI * kappa;
        float elastic_x = -Cb * sinf(phi_actual[s]);
        float elastic_y =  Cb * cosf(phi_actual[s]);
        required[s][0] = elastic_x - Mg_local[0];
        required[s][1] = elastic_y - Mg_local[1];
    }

    float distal[3], proximal[3];
    _moment_to_tensions(1, required[1], R, distal);
    float proximal_residual[2] = {required[0][0], required[0][1]};
    for (int j = 0; j < 3; j++) {
        float alpha = s_wire_alpha[1][j];
        proximal_residual[0] -= R * distal[j] * (-sinf(alpha));
        proximal_residual[1] -= R * distal[j] * cosf(alpha);
    }
    _moment_to_tensions(0, proximal_residual, R, proximal);

    float raw[SDM_WIRES] = {0};
    for (int j = 0; j < 3; j++) {
        raw[s_seg_to_wires[0][j]] = proximal[j];
        raw[s_seg_to_wires[1][j]] = distal[j];
    }
    float omega = _clampf(s_params.dynamics.force_relaxation, 0.1f, 1.0f);
    for (int i = 0; i < SDM_WIRES; i++) {
        if (!s_force_target_valid) s_force_target[i] = raw[i];
        else s_force_target[i] += omega * (raw[i] - s_force_target[i]);
        f_target_out[i] = s_force_target[i];
    }
    s_force_target_valid = true;
}

/* ==================== k_ratio 计算 (内部) ==================== */
static void _compute_k_ratio(const float forces[SENSOR_NUM],
                             const float theta_desired[SDM_SEGMENTS],
                             const float phi_desired[SDM_SEGMENTS],
                             const float deltaL_base[SDM_WIRES],
                             float R,
                             float K_force)
{
    /* 1. 动力学模型: 每根丝独立的目标拉力 */
    float f_target[SDM_WIRES];
    _dynamics_model(theta_desired, phi_desired, R, f_target);

    /* 2. 力误差 → k_ratio */
    for (int i = 0; i < SDM_WIRES; i++) {
        float f_err = forces[i] - f_target[i];
        float L_ref = fabsf(deltaL_base[i]) + 0.1f;
        s_k_ratio[i] = _clampf(1.0f + K_force * f_err / L_ref, 0.6f, 1.4f);
    }
}

/* ==================== 对外 API ==================== */

/**
 * @brief 初始化 SDM 模块。
 * @param bending_stiffness  弯曲刚度 (N·m/rad)
 * @param force_peak_limit   力峰值上限 (N)
 * @param force_recovery     恢复阈值 [0~1]
 * @param tip_mass           尖端质量 (kg), 用于重力补偿
 * @param mount_dir          臂体初始方向 (世界坐标系, 单位向量, [3])
 *                           水平安装传 [1,0,0] 或 [0,1,0]，竖直传 [0,0,1]
 *                           NULL 则默认竖直向上
 */
void sdm_init(float bending_stiffness,
              float force_peak_limit,
              float force_recovery,
              float tip_mass,
              const float mount_dir[3])
{
    s_params.bending_stiffness = bending_stiffness;
    s_params.force_peak_limit  = force_peak_limit;
    s_params.force_recovery    = force_recovery;
    s_params.tip_mass          = tip_mass;
    s_params.gravity           = 9.81f;
    sdm_configure_dynamics(NULL);

    /* 构建初始安装姿态矩阵 */
    if (mount_dir) {
        _build_mount_matrix(mount_dir, s_params.R_mount);
    } else {
        /* 默认竖直向上: 局部Z轴 = 世界Z轴 = [0,0,1] */
        float default_dir[3] = {0.0f, 0.0f, 1.0f};
        _build_mount_matrix(default_dir, s_params.R_mount);
    }

    for (int i = 0; i < SDM_WIRES; i++) {
        s_k_ratio[i] = 1.0f;
        s_force_target[i] = 0.0f;
    }
    s_force_target_valid = false;
    s_initialized = true;
}

void sdm_configure_dynamics(const SDM_DynamicsConfig *config)
{
    if (config == NULL) {
        for (int s = 0; s < SDM_SEGMENTS; s++)
            s_params.dynamics.segment_mass[s] = 0.0f;
        s_params.dynamics.curvature_stiffness = 0.0f;
        s_params.dynamics.tendon_pretension = 0.0f;
        s_params.dynamics.force_relaxation = 0.3f;
    } else {
        for (int s = 0; s < SDM_SEGMENTS; s++)
            s_params.dynamics.segment_mass[s] =
                fmaxf(config->segment_mass[s], 0.0f);
        s_params.dynamics.curvature_stiffness =
            fmaxf(config->curvature_stiffness, 0.0f);
        s_params.dynamics.tendon_pretension =
            fmaxf(config->tendon_pretension, 0.0f);
        s_params.dynamics.force_relaxation =
            _clampf(config->force_relaxation, 0.1f, 1.0f);
    }
    s_force_target_valid = false;
}

/**
 * @brief 获取力峰值上限 (N)
 */
float sdm_get_force_peak_limit(void) { return s_params.force_peak_limit; }

/**
 * @brief 一步 SDM 完整控制。
 *
 * 内部流程:
 *   1. 力安全 → theta_safe = theta_desired × safety
 *   2. PCC 逆运动学 → 基准丝长 ΔL_base
 *   3. 从 deltaL_actual 反解真实臂体几何 (θ_actual, φ_actual)
 *   4. 动力学模型基于真实几何计算每根丝目标拉力
 *   5. k_ratio = f(F_real, F_target, ΔL_base)
 *   6. ΔL_out[i] = ΔL_base[i] × k_ratio[i]
 *
 * @param forces         实时 6 路肌腱力 (N)
 * @param theta_desired  各段期望弯曲角 (rad, [2])
 * @param phi_desired    各段期望弯曲方向角 (rad, [2])
 * @param deltaL_actual  实际 6 根丝位移反馈 (mm), NULL 则用期望值近似
 * @param K_force        力控增益 (建议 0.01~0.1)
 * @param R              驱动丝半径 (m)
 * @param deltaL_out     输出: 6 根丝最终位移量 (mm)
 */
void sdm_step(const float forces[SENSOR_NUM],
              const float theta_desired[SDM_SEGMENTS],
              const float phi_desired[SDM_SEGMENTS],
              const float deltaL_actual[SDM_WIRES],
              float K_force,
              float R,
              float deltaL_out[SDM_WIRES])
{
    if (!s_initialized || forces == NULL || theta_desired == NULL ||
        phi_desired == NULL || deltaL_out == NULL) {
        return;
    }

    /* 1. 力安全 */
    float safety;
    bool  over_peak;
    _force_safety(forces, &safety, &over_peak);

    /* 力超限：硬停止，输出全部为 0 */
    if (over_peak) {
        for (int i = 0; i < SDM_WIRES; i++) deltaL_out[i] = 0.0f;
        return;
    }

    float theta_safe[SDM_SEGMENTS], phi_safe[SDM_SEGMENTS];
    for (int s = 0; s < SDM_SEGMENTS; s++) {
        theta_safe[s] = theta_desired[s] * safety;
        phi_safe[s]   = phi_desired[s];
    }

    /* 2. PCC → 基准驱动丝长度 */
    float deltaL_base[SDM_WIRES];
    _calculate_L(R, theta_safe, phi_safe, deltaL_base);

    /* 3. 从实际丝长反解真实臂体几何 */
    float theta_actual[SDM_SEGMENTS], phi_actual[SDM_SEGMENTS];
    if (deltaL_actual != NULL) {
        _inverse_kinematics(deltaL_actual, R, theta_actual, phi_actual);
    } else {
        /* 无反馈时用期望值近似 */
        for (int s = 0; s < SDM_SEGMENTS; s++) {
            theta_actual[s] = theta_safe[s];
            phi_actual[s]   = phi_safe[s];
        }
    }

    /* 4. 力控 k_ratio (动力学模型使用真实臂体几何) */
    _compute_k_ratio(forces, theta_actual, phi_actual, deltaL_base, R, K_force);

    /* 5. deltaL_out = deltaL_base × k_ratio */
    for (int i = 0; i < SDM_WIRES; i++) {
        deltaL_out[i] = deltaL_base[i] * s_k_ratio[i];
        if (isnan(deltaL_out[i]) || isinf(deltaL_out[i]))
            deltaL_out[i] = 0.0f;
    }

    /* 6. 直接驱动电机 */
    motor_sync_control(SDM_WIRES, 0, deltaL_out);
}

/**
 * @brief 纯运动学步进 — PCC 逆运动学 + 力安全 + 电机驱动（无动力学模型、无 k_ratio）
 *
 * 内部流程:
 *   1. 力安全阈值解算 → theta_safe = theta_desired × safety
 *   2. PCC 逆运动学 → deltaL_out
 *   3. 驱动电机
 *
 * @param forces         实时 6 路肌腱力 (N)，用于力安全
 * @param theta_desired  各段期望弯曲角 (rad, [2])
 * @param phi_desired    各段期望弯曲方向角 (rad, [2])
 * @param R              驱动丝半径 (m)
 * @param deltaL_out     输出: 6 根丝最终位移量 (mm)
 */
void sdm_kinematic_step(const float forces[SENSOR_NUM],
                         const float theta_desired[SDM_SEGMENTS],
                         const float phi_desired[SDM_SEGMENTS],
                         float R,
                         float deltaL_out[SDM_WIRES])
{
    if (!s_initialized || theta_desired == NULL ||
        phi_desired == NULL || deltaL_out == NULL) {
        return;
    }

    /* 1. 力安全阈值解算 */
    float safety;
    bool  over_peak;
    _force_safety(forces, &safety, &over_peak);

    /* 力超限：硬停止 */
    if (over_peak) {
        for (int i = 0; i < SDM_WIRES; i++) deltaL_out[i] = 0.0f;
        return;
    }

    float theta_safe[SDM_SEGMENTS], phi_safe[SDM_SEGMENTS];
    for (int s = 0; s < SDM_SEGMENTS; s++) {
        theta_safe[s] = theta_desired[s] * safety;
        phi_safe[s]   = phi_desired[s];
    }

    /* 2. 纯 PCC 逆运动学 → 直接输出（无动力学 / k_ratio 修正） */
    _calculate_L(R, theta_safe, phi_safe, deltaL_out);

    /* NaN/Inf 保护 */
    for (int i = 0; i < SDM_WIRES; i++) {
        if (isnan(deltaL_out[i]) || isinf(deltaL_out[i]))
            deltaL_out[i] = 0.0f;
    }

    /* 3. 直接驱动电机 */
    motor_sync_control(SDM_WIRES, 0, deltaL_out);
}

void sdm_auto_straight(void)
{
    float zero[SDM_WIRES] = {0};
    motor_sync_control(SDM_WIRES, 0, zero);
    HAL_Delay(500);
}
