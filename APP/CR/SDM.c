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
#include <math.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

/* ==================== 内部常量 ==================== */
#define SDM_PI_3    1.0471975511965976f
#define SDM_2PI_3   2.0943951023931953f
#define SDM_4PI_3   4.1887902047863905f
#define SDM_5PI_3   5.235987755982988f
#define SDM_EPS     1e-9f

/* ==================== 内部状态 ==================== */
typedef struct {
    float cable_radius[SDM_SEGMENTS];
    float bending_stiffness;
    float damping_coeff;
    float force_peak_limit;
    float force_recovery;
    float tip_mass;
    float gravity;
    float R_mount[3][3];    /**< 臂体当前安装姿态 (世界→基座局部) */
    float mount_dir[3];     /**< 臂体安装方向 (世界系单位向量) */

    /* 曲率分布模型参数 (由实测肌腱力正解 κ(s)) */
    float C0;               /**< 弯矩常数 (N·m²) = EI0/L_strut, 默认 SDM_CURV_DEFAULT_C0 */
    float L_strut;          /**< 晶胞斜杆长 (m), θ=|κ|·L_strut, 默认 SDM_CURV_L_STRUT */
} SDM_InternalParams;

static SDM_InternalParams s_params = {0};
static bool s_initialized = false;

/* k_ratio 不对外暴露 */
static float s_k_ratio[SDM_WIRES];

/* 重力补偿增益 (前馈), 0 = 关闭 */
static float s_grav_comp_gain = 0.0f;

/* 重力补偿弯曲角限幅 (rad): 防止参数不准时补偿量发散 */
#define SDM_GRAV_COMP_THETA_MAX  1.0471976f   /* 60° */

/* 段 → 驱动丝索引 */
static const int s_seg_to_wires[SDM_SEGMENTS][SDM_WIRES_PER_SEG] = {
    { 0, 2, 4 },   /* 段0: 丝0, 丝2, 丝4 */
    { 1, 3, 5 }    /* 段1: 丝1, 丝3, 丝5 */
};

/* ==================== 内部工具 ==================== */
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
static void _calculate_L(float R, const float theta[SDM_SEGMENTS],
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
    if (R < SDM_EPS) R = s_params.cable_radius[0];   /* 回退到初始化登记的驱动丝半径 */
    float sqrt3_inv = 1.0f / sqrtf(3.0f);

    /* ——— 段0: 丝0,2,4 角度 {0, 2π/3, 4π/3} ——— */
    float c0 = -deltaL_actual[0] / R;
    float c1 = -deltaL_actual[2] / R;
    float c2 = -deltaL_actual[4] / R;

    float diff = c2 - c1;
    theta_out[0] = sqrtf(c0 * c0 + diff * diff / 3.0f);
    phi_out[0]   = atan2f(diff * sqrt3_inv, c0);

    /* ——— 段1: 丝1,3,5 角度 {π/3, π, 5π/3} ——— */
    /* 先减去段0对该段三根丝的贡献 */
    float s0_contribution_1 = -R * theta_out[0] * cosf(phi_out[0] + SDM_PI_3);
    float s0_contribution_3 = -R * theta_out[0] * cosf(phi_out[0] + SDM_PI);
    float s0_contribution_5 = -R * theta_out[0] * cosf(phi_out[0] + SDM_5PI_3);

    /* 段1 独立贡献 (注意丝3,5 的公式里段1项是负号) */
    float d0 = (deltaL_actual[1] - s0_contribution_1) / R;   /* θ₁ cos(φ₁ + π/3) */
    float d1 = -(deltaL_actual[3] - s0_contribution_3) / R;  /* θ₁ cos(φ₁ + π) */
    float d2 = -(deltaL_actual[5] - s0_contribution_5) / R;  /* 新增传感器校准功能θ₁ cos(φ₁ + 5π/3) */

    diff = d2 - d1;
    theta_out[1] = sqrtf(d0 * d0 + diff * diff / 3.0f);
    float psi1   = atan2f(diff * sqrt3_inv, d0);
    phi_out[1]   = psi1 - SDM_PI_3;
}

/* ==================== 力安全因子 (内部) ==================== */
static float _force_safety(const float forces[SENSOR_NUM],
                           float *safety, bool *over_peak)
{
    float f_max = 0.0f;
    for (int i = 0; i < SENSOR_NUM; i++) {
        if (forces[i] > f_max) f_max = forces[i];
    }
    *over_peak = (f_max >= s_params.force_peak_limit);
    *safety = _clampf(1.0f - f_max / s_params.force_peak_limit,
                      s_params.force_recovery, 1.0f);
    return f_max;
}

/* ==================== 简化动力学模型 ==================== */

/* 各段每根丝的角度 (相对于局部X轴, 与 _calculate_L 公式一致) */
static const float s_wire_alpha[SDM_SEGMENTS][SDM_WIRES_PER_SEG] = {
    { 0.0f,    SDM_2PI_3, SDM_4PI_3 },   /* 段0: 丝0=0°, 丝2=120°, 丝4=240° */
    { SDM_PI_3, SDM_PI,   SDM_5PI_3 }    /* 段1: 丝1=60°, 丝3=180°, 丝5=300° */
};

/**
 * @brief 简化动力学模型: 弯曲角 → 每根丝实时目标拉力
 *
 * 两项叠加:
 *   f_target_i = f_geom + f_grav
 *
 *   f_geom  = (M / R) × |ΔL_i| / Σ|ΔL_j| ,  M = bending_stiffness·|θ|  (N·m)
 *             PCC 几何项: 丝长变化越大拉力越大; 量纲 = N·m / m = N
 *
 *   f_grav  = |g_proj_i| × (L_arm / L_total)
 *             重力补偿项，由虚拟中心杆实时姿态决定
 *
 * R_acc 累积旋转的执行顺序:
 *   段0 循环: R_acc=I → 算段0 的 f_target → R_acc = I×R_local_0
 *   段1 循环: R_acc=I×R_local_0 → 算段1 的 f_target → R_acc = I×R_local_0×R_local_1
 *   即: 段1 的重力投影已经包含了段0 弯曲后的姿态
 *
 * @param theta_actual   实际各段弯曲角 (rad, [2])
 * @param phi_actual     实际各段弯曲方向角 (rad, [2])
 * @param R              驱动丝半径 (m)
 * @param f_target_out   输出: 每根丝的目标拉力 (N, [6])
 */
static void _dynamics_model(const float theta_actual[SDM_SEGMENTS],
                            const float phi_actual[SDM_SEGMENTS],
                            float R,
                            float f_target_out[SDM_WIRES])
{
    for (int i = 0; i < SDM_WIRES; i++) {
        f_target_out[i] = 0.0f;
    }

    /* 尖端重量 (固定参数, 非自适应) */
    float W_tip = s_params.tip_mass * s_params.gravity;

    /* 总臂长: 从 CR 结构体读取, 不硬编码 */
    float L_total = 0.0f;
    for (int s = 0; s < SDM_SEGMENTS; s++) L_total += (float)CR.arm_params[s].L;
    if (L_total < SDM_EPS) L_total = 0.45f;

    /* 从基座安装姿态开始的累积旋转矩阵 R_acc */
    float R_acc[3][3];
    memcpy(R_acc, s_params.R_mount, sizeof(R_acc));

    for (int s = 0; s < SDM_SEGMENTS; s++) {
        float th = theta_actual[s];
        float ph = phi_actual[s];

        /* PCC 本段局部旋转矩阵 R_local */
        float cp = cosf(ph), sp = sinf(ph);
        float ct = cosf(th), st = sinf(th);
        float R_local[3][3] = {
            { cp*cp*(ct-1)+1,  sp*cp*(ct-1),     cp*st },
            { sp*cp*(ct-1),    cp*cp*(1-ct)+ct,   sp*st },
            { -cp*st,          -sp*st,            ct     }
        };

        /* 重力投影到当前段的截面坐标系
         * R_acc 此时反映从基座到本段之前的累积姿态
         * 段0: R_acc=I (基座坐标系, 重力沿Z)
         * 段1: R_acc=I×R_local_0 (包含段0弯曲后的姿态)
         */
        float g_lx = -W_tip * R_acc[2][0];
        float g_ly = -W_tip * R_acc[2][1];

        /* 力臂: 从当前段到尖端的距离 (从 CR 结构体读取) */
        float L_arm = 0.0f;
        for (int ss = s; ss < SDM_SEGMENTS; ss++) L_arm += (float)CR.arm_params[ss].L;
        float moment_ratio = L_arm / L_total;

        /* 每段弯矩 M = bending_stiffness·|θ| (N·m), 丝力尺度 = M/R (N)
         * 量纲: [N·m]/[m] = N  —— 旧式多除了 L_seg, 量纲为 N/m 且数值偏大 1/L_seg 倍 */
        float K = s_params.bending_stiffness * fabsf(th) / (R + SDM_EPS);

        int wires[3] = { s_seg_to_wires[s][0], s_seg_to_wires[s][1], s_seg_to_wires[s][2] };

        /* 先算每根丝的 |ΔL| 及总和，用于归一化 */
        float abs_dL[3];
        float sum_abs_dL = 0.0f;
        for (int j = 0; j < 3; j++) {
            abs_dL[j] = fabsf(-R * th * cosf(ph + s_wire_alpha[s][j]));
            sum_abs_dL += abs_dL[j];
        }
        if (sum_abs_dL < SDM_EPS) sum_abs_dL = SDM_EPS;

        for (int j = 0; j < 3; j++) {
            /* PCC 几何项 (归一化) */
            float f_geom = K * abs_dL[j] / sum_abs_dL;

            /* 重力补偿项 */
            float g_proj = g_lx * cosf(s_wire_alpha[s][j]) + g_ly * sinf(s_wire_alpha[s][j]);
            float f_grav = fabsf(g_proj) * moment_ratio;

            f_target_out[wires[j]] = f_geom + f_grav;
        }

        /* 累积旋转: R_acc = R_acc × R_local
         * 更新后供下一段使用，使下一段的重力投影包含本段的弯曲效果
         */
        float R_new[3][3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) {
                R_new[r][c] = 0;
                for (int k = 0; k < 3; k++)
                    R_new[r][c] += R_acc[r][k] * R_local[k][c];
            }
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                R_acc[r][c] = R_new[r][c];
    }
}

/* ==================== 重力模型 (内部) ==================== */

/**
 * @brief 逐步累积出「基座→各段截面」的姿态, 供重力投影使用
 * @param theta 各段弯曲角 (rad)
 * @param phi   各段方向角 (rad)
 * @param Racc  输出 [SDM_SEGMENTS][3][3]
 */
static void _accumulate_poses(const float theta[SDM_SEGMENTS],
                              const float phi[SDM_SEGMENTS],
                              float Racc[SDM_SEGMENTS][3][3])
{
    float acc[3][3];
    memcpy(acc, s_params.R_mount, sizeof(acc));
    memcpy(Racc[0], acc, sizeof(acc));

    for (int s = 0; s < SDM_SEGMENTS; s++) {
        const float th = theta[s], ph = phi[s];
        const float cp = cosf(ph), sp = sinf(ph);
        const float ct = cosf(th), st = sinf(th);
        const float Rl[3][3] = {
            { cp*cp*(ct-1)+1,  sp*cp*(ct-1),    cp*st },
            { sp*cp*(ct-1),    cp*cp*(1-ct)+ct, sp*st },
            { -cp*st,          -sp*st,           ct   }
        };
        float Rn[3][3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) {
                Rn[r][c] = 0.0f;
                for (int k = 0; k < 3; k++) Rn[r][c] += acc[r][k] * Rl[k][c];
            }
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) acc[r][c] = Rn[r][c];

        if (s + 1 < SDM_SEGMENTS) memcpy(Racc[s + 1], acc, sizeof(acc));
    }
}

static float _total_length(void)
{
    float L = 0.0f;
    for (int s = 0; s < SDM_SEGMENTS; s++) L += (float)CR.arm_params[s].L;
    return (L < SDM_EPS) ? 0.45f : L;
}

/**
 * @brief 重力在各丝上的张力 (仅重力项)
 *
 *   g_xy     = 尖端重力在当前段截面内的横向分量 (N)
 *   f_grav_j = |g_xy·(cosα_j, sinα_j)| × (该段到尖端的力臂 / 总长)
 */
static void _gravity_model(const float theta[SDM_SEGMENTS],
                           const float phi[SDM_SEGMENTS],
                           float f_grav[SDM_WIRES])
{
    for (int i = 0; i < SDM_WIRES; i++) f_grav[i] = 0.0f;

    const float W_tip   = s_params.tip_mass * s_params.gravity;
    const float L_total = _total_length();
    float Racc[SDM_SEGMENTS][3][3];
    _accumulate_poses(theta, phi, Racc);

    for (int s = 0; s < SDM_SEGMENTS; s++) {
        const float g_lx = -W_tip * Racc[s][2][0];
        const float g_ly = -W_tip * Racc[s][2][1];

        float L_arm = 0.0f;
        for (int ss = s; ss < SDM_SEGMENTS; ss++) L_arm += (float)CR.arm_params[ss].L;
        const float mr = L_arm / L_total;

        for (int j = 0; j < SDM_WIRES_PER_SEG; j++) {
            const float gp = g_lx * cosf(s_wire_alpha[s][j]) + g_ly * sinf(s_wire_alpha[s][j]);
            f_grav[s_seg_to_wires[s][j]] = fabsf(gp) * mr;
        }
    }
}

/**
 * @brief 重力补偿前馈: 生成抵消重力弯矩的丝位移偏置
 *
 *   该段重力弯矩        M   = L_arm × |g_xy|
 *   需要施加的反向弯曲  θ_c = gain · M / bending_stiffness
 *   方向                φ_c = 重力横向分量的反方向
 * 再按标准 PCC 映射成 6 路位移 (与 _calculate_L 同源, 保证一致)。
 */
static void _gravity_comp_bias(const float theta[SDM_SEGMENTS],
                               const float phi[SDM_SEGMENTS],
                               float R, float gain,
                               float deltaL_out[SDM_WIRES])
{
    const float W_tip   = s_params.tip_mass * s_params.gravity;
    float Racc[SDM_SEGMENTS][3][3];
    _accumulate_poses(theta, phi, Racc);

    float th_c[SDM_SEGMENTS], ph_c[SDM_SEGMENTS];
    for (int s = 0; s < SDM_SEGMENTS; s++) {
        const float g_lx = -W_tip * Racc[s][2][0];
        const float g_ly = -W_tip * Racc[s][2][1];

        float L_arm = 0.0f;
        for (int ss = s; ss < SDM_SEGMENTS; ss++) L_arm += (float)CR.arm_params[ss].L;

        const float M = L_arm * sqrtf(g_lx * g_lx + g_ly * g_ly);
        float th = gain * M / (s_params.bending_stiffness + SDM_EPS);
        if (th > SDM_GRAV_COMP_THETA_MAX) th = SDM_GRAV_COMP_THETA_MAX;
        if (th < 0.0f) th = 0.0f;

        th_c[s] = th;
        /* 重力把臂往下压 (方向 g_xy), 补偿要往反方向拉 */
        ph_c[s] = (M > SDM_EPS) ? atan2f(-g_ly, -g_lx) : 0.0f;
    }
    _calculate_L(R, th_c, ph_c, deltaL_out);
}

/* ==================== 张力份额 (比值) ==================== */

int sdm_segment_shares(int seg,
                       float phi_seg,
                       const float deltaL_base[SDM_WIRES],
                       float share_des_out[SDM_WIRES_PER_SEG])
{
    (void)phi_seg;

    if (seg < 0 || seg >= SDM_SEGMENTS || share_des_out == NULL) return 0;

    float w[SDM_WIRES_PER_SEG];
    float wsum = 0.0f;
    int   nact = 0;

    for (int j = 0; j < SDM_WIRES_PER_SEG; j++) {
        const int   wi   = s_seg_to_wires[seg][j];
        /* 拉动量 > 0 表示该丝被拉紧; 与 -ΔL = R·θ·cos(φ+α) 等价 */
        const float pull = (deltaL_base != NULL) ? -deltaL_base[wi] : 0.0f;

        if (pull > SDM_ACTIVE_DL_MM) {
            w[j] = pull;
            wsum += pull;
            nact++;
        } else {
            w[j] = 0.0f;   /* 松弛丝: 期望份额 0 (仅剩预紧力) */
        }
    }

    if (nact >= 1 && wsum > SDM_EPS) {
        for (int j = 0; j < SDM_WIRES_PER_SEG; j++) {
            share_des_out[j] = w[j] / wsum;
        }
        return nact;
    }

    /* 退化: 整段无被拉丝 (近直臂) → 均分, 供检测用; 返回 0 提示控制侧跳过 */
    for (int j = 0; j < SDM_WIRES_PER_SEG; j++) {
        share_des_out[j] = 1.0f / (float)SDM_WIRES_PER_SEG;
    }
    return 0;
}

/* ==================== k_ratio 计算 (内部) ==================== */

/**
 * @brief 比例式力平衡: 使段内实测张力份额逼近期望份额
 *
 *   r_j = τ_j / Στ        (实测份额, 无量纲)
 *   e_j = r_j - w*_j
 *   k_j = clamp(1 - K·e_j)     e_j>0(份额偏高) → k<1 → 少拉
 *
 * 全程不含绝对张力 ⇒ 对传感器增益/零点漂移、模型绝对力尺度误差免疫。
 */
static void _compute_k_ratio_share(const float forces[SDM_WIRES],
                                   const float phi_desired[SDM_SEGMENTS],
                                   const float deltaL_base[SDM_WIRES],
                                   float K_ratio)
{
    for (int i = 0; i < SDM_WIRES; i++) s_k_ratio[i] = 1.0f;

    for (int s = 0; s < SDM_SEGMENTS; s++) {
        float w[SDM_WIRES_PER_SEG];
        const int nact = sdm_segment_shares(s, phi_desired[s], deltaL_base, w);

        /* 只有 1 根被拉丝时无「份额」可平衡, 不做干预 */
        if (nact < 2) continue;

        /* 期望/实测份额均只在「被拉丝集合」内归一化:
         * 松弛丝的预紧力若计入分母会稀释份额, 产生虚假误差 */
        float wsum = 0.0f, tsum = 0.0f;
        for (int j = 0; j < SDM_WIRES_PER_SEG; j++) {
            if (w[j] <= 0.0f) continue;
            wsum += w[j];
            tsum += forces[s_seg_to_wires[s][j]];
        }
        if (wsum < SDM_EPS || tsum < SDM_RATIO_MIN_SUM_N) continue;

        for (int j = 0; j < SDM_WIRES_PER_SEG; j++) {
            if (w[j] <= 0.0f) continue;   /* 松弛丝不施加修正, 避免把它拉紧 */
            const int   wi     = s_seg_to_wires[s][j];
            const float r_meas = forces[wi] / tsum;
            const float r_des  = w[j] / wsum;
            const float e      = r_meas - r_des;
            s_k_ratio[wi] = _clampf(1.0f - K_ratio * e, SDM_KRATIO_MIN, SDM_KRATIO_MAX);
        }
    }
}

static void _compute_k_ratio(const float forces[SENSOR_NUM],
                             const float theta_desired[SDM_SEGMENTS],
                             const float phi_desired[SDM_SEGMENTS],
                             const float deltaL_base[SDM_WIRES],
                             float R,
                             float K_force)
{
#if (SDM_FORCE_MODE == 0)
    /* 纯运动学 */
    (void)forces; (void)theta_desired; (void)phi_desired;
    (void)deltaL_base; (void)R; (void)K_force;
    for (int i = 0; i < SDM_WIRES; i++) s_k_ratio[i] = 1.0f;

#elif (SDM_FORCE_MODE == 1)
    /* 比例式力平衡 (默认) */
    (void)theta_desired; (void)R;
    _compute_k_ratio_share(forces, phi_desired, deltaL_base, K_force);

#else
    /* legacy: 绝对力误差。符号已修正——实测高于目标 ⇒ 少拉。
     * 现场张力绝对量不可靠时误差大, 仅作对比用。 */
    float f_target[SDM_WIRES];
    _dynamics_model(theta_desired, phi_desired, R, f_target);
    for (int i = 0; i < SDM_WIRES; i++) {
        const float f_err = forces[i] - f_target[i];
        const float L_ref = fabsf(deltaL_base[i]) + 0.1f;
        s_k_ratio[i] = _clampf(1.0f - K_force * f_err / L_ref,
                               SDM_KRATIO_MIN, SDM_KRATIO_MAX);
    }
#endif
}

/* ==================== 对外 API ==================== */

void sdm_set_mount(const float dir[3])
{
    if (dir == NULL) return;

    float n = sqrtf(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
    if (n < SDM_EPS) return;                 /* 零向量: 忽略 */

    for (int i = 0; i < 3; i++) s_params.mount_dir[i] = dir[i] / n;
    _build_mount_matrix(s_params.mount_dir, s_params.R_mount);
}

void sdm_get_mount(float dir[3])
{
    if (dir == NULL) return;
    for (int i = 0; i < 3; i++) dir[i] = s_params.mount_dir[i];
}

void sdm_gravity_force(const float theta[SDM_SEGMENTS],
                       const float phi[SDM_SEGMENTS],
                       float g_out[SDM_WIRES])
{
    if (g_out == NULL) return;
    if (theta == NULL || phi == NULL) {
        for (int i = 0; i < SDM_WIRES; i++) g_out[i] = 0.0f;
        return;
    }
    _gravity_model(theta, phi, g_out);
}

void sdm_set_grav_comp_gain(float gain)
{
    if (gain < 0.0f) gain = 0.0f;
    if (gain > 2.0f) gain = 2.0f;
    s_grav_comp_gain = gain;
}

float sdm_get_grav_comp_gain(void)
{
    return s_grav_comp_gain;
}

void sdm_pcc_deltaL(const float theta[SDM_SEGMENTS],
                    const float phi[SDM_SEGMENTS],
                    float R,
                    float deltaL_out[SDM_WIRES])
{
    if (theta == NULL || phi == NULL || deltaL_out == NULL) return;
    if (R < SDM_EPS) R = s_params.cable_radius[0];
    _calculate_L(R, theta, phi, deltaL_out);
}

void sdm_force_target(const float theta[SDM_SEGMENTS],
                      const float phi[SDM_SEGMENTS],
                      float R,
                      float f_out[SDM_WIRES])
{
    if (!s_initialized || theta == NULL || phi == NULL || f_out == NULL) {
        if (f_out != NULL) {
            for (int i = 0; i < SDM_WIRES; i++) f_out[i] = 0.0f;
        }
        return;
    }
    _dynamics_model(theta, phi, R, f_out);
}

void sdm_init(const float cable_radius[SDM_SEGMENTS],
              float bending_stiffness,
              float force_peak_limit,
              float force_recovery,
              float tip_mass,
              const float mount_dir[3])
{
    if (cable_radius) memcpy(s_params.cable_radius, cable_radius, sizeof(s_params.cable_radius));
    s_params.bending_stiffness = bending_stiffness;
    s_params.damping_coeff     = 50.0f;
    s_params.force_peak_limit  = force_peak_limit;
    s_params.force_recovery    = force_recovery;
    s_params.tip_mass          = tip_mass;
    s_params.gravity           = 9.81f;

    /* 构建初始安装姿态矩阵并记录方向 */
    {
        float d[3] = { 0.0f, 0.0f, 1.0f };   /* 默认竖直向上 */
        if (mount_dir) { d[0] = mount_dir[0]; d[1] = mount_dir[1]; d[2] = mount_dir[2]; }

        float n = sqrtf(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
        if (n < SDM_EPS) { d[0] = 0.0f; d[1] = 0.0f; d[2] = 1.0f; n = 1.0f; }
        for (int i = 0; i < 3; i++) s_params.mount_dir[i] = d[i] / n;

        _build_mount_matrix(s_params.mount_dir, s_params.R_mount);
    }

    for (int i = 0; i < SDM_WIRES; i++) s_k_ratio[i] = 1.0f;

    /* 曲率分布模型默认参数 (弯矩 C=C0·sin(θ), θ=|κ|·L_strut; 割线 EI=C/κ→EI0) */
    s_params.C0        = SDM_CURV_DEFAULT_C0;
    s_params.L_strut   = SDM_CURV_L_STRUT;

    s_initialized = true;
}

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

    /* R 无效时回退到初始化登记的驱动丝半径, 避免 PCC 几何发散 */
    if (R < SDM_EPS) R = s_params.cable_radius[0];

    /* 1. 力安全 */
    float safety;
    bool  over_peak;
    _force_safety(forces, &safety, &over_peak);

    float theta_safe[SDM_SEGMENTS], phi_safe[SDM_SEGMENTS];
    for (int s = 0; s < SDM_SEGMENTS; s++) {
        theta_safe[s] = theta_desired[s] * safety;
        phi_safe[s]   = phi_desired[s];
    }

    /* 2. PCC → 基准驱动丝长度 */
    float deltaL_base[SDM_WIRES];
    _calculate_L(R, theta_safe, phi_safe, deltaL_base);

    /* 2b. 重力补偿前馈: 用驱动丝张力抵消臂体+尖端重力弯矩。
     *     在 deltaL 上叠加一个反向弯曲偏置, 由 gain 缩放 (0=关闭)。 */
    if (s_grav_comp_gain > 0.0f) {
        float dg[SDM_WIRES];
        _gravity_comp_bias(theta_safe, phi_safe, R, s_grav_comp_gain, dg);
        for (int i = 0; i < SDM_WIRES; i++) {
            deltaL_base[i] += dg[i];
        }
    }

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

    /* 4. 力控 k_ratio (deltaL_base 由 phi_safe 生成, 份额比对须同源) */
    _compute_k_ratio(forces, theta_actual, phi_safe, deltaL_base, R, K_force);

    /* 5. deltaL_out = deltaL_base × k_ratio */
    for (int i = 0; i < SDM_WIRES; i++) {
        deltaL_out[i] = deltaL_base[i] * s_k_ratio[i];
        if (isnan(deltaL_out[i]) || isinf(deltaL_out[i]))
            deltaL_out[i] = 0.0f;
    }
}

