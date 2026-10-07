/**
 * @file SDM_vc.h
 * @brief 完整 Cosserat 静力平衡 (VC 式) — 连续体柔性臂弯矩分布正解
 *
 * 输入: 6 路丝张力幅值 τ[N] (VC 式随从力; 方向由平衡自洽)。
 * 环境: 重力分布、末端外力 F_tip。
 * 求解: 单点打靶完整 Cosserat 平衡 → 解出沿臂弯矩分布 m(s)、形状 p(s)、
 *       内力 n(s)、曲率 u(s)。弯矩分布是"背骨刚度+重力+外力+丝张力"平衡结果,
 *       而非人为指定的两段常矩。
 *
 * 数学照搬: tdcr-modeling-master/matlab/Models/VC/
 *   (VC_solver / intermedquant / boundcond), 推广到 6 独立丝
 *   (段A 丝0,2,4 止于 L1; 段B 丝1,3,5 贯穿到末端)。
 *
 * 参考实现: tools/sdm_vc.py (已通过直臂/对称/单丝物理自检)。
 *
 * @date 2026-09-05
 */
#ifndef __SDM_VC_H
#define __SDM_VC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* VC 求解器几何/材料参数 */
typedef struct {
    double EI;             /* 弯曲刚度 Kbt_x=Kbt_y (N·m²) */
    double GJ;             /* 扭转刚度 Kbt_z (N·m²) */
    double GA;             /* 剪切刚度 Kse_x=Kse_y (N) */
    double EA;             /* 轴向刚度 Kse_z (N) */
    double r_disk;         /* 丝孔分布半径 (m) */
    double rhoA;           /* 每段背骨线密度 (kg/m, 用于重力) */
    double disk_mass;      /* 单盘质量 (kg, 重力均摊) */
    int   n_p;            /* 段A 子段数 */
    int   n_d;            /* 段B 子段数 */
} sdm_vc_params_t;

/* VC 求解结果 */
typedef struct {
    double v0[3];          /* 基座线应变速率 (体) */
    double u0[3];          /* 基座角速率/曲率 (体) */
    double res[6];         /* 末端残差 (力3+矩3, N/N·m) */
    double tip[3];         /* 末端位置 */
    double rms_res;        /* 残差 rms */
    uint8_t converged;    /* 打靶收敛 */
    int    n_iter;        /* 迭代次数 */
} sdm_vc_result_t;

/**
 * @brief 完整 Cosserat 静力平衡求解 (打靶 BVP).
 *
 * @param tau     6 路丝张力幅值 (N, 应 ≥0)
 * @param F_tip   末端外力 (N, 世界系, 3), 可 NULL=0
 * @param g_world 重力矢量 (m/s², 世界系, 3), 可 NULL=(0,0,-9.81)
 * @param R_mount 基座世界姿态 3×3 (列), 可 NULL=单位阵
 * @param par     VC 参数 (刚度/几何/子段)
 * @param out     结果 (v0/u0/res/tip/converged)
 * @return 0 成功 / 1 参数错
 */
uint8_t sdm_vc_solve_statics(const double tau[6],
                             const double F_tip[3],
                             const double g_world[3],
                             const double R_mount[3][3],
                             const sdm_vc_params_t *par,
                             sdm_vc_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __SDM_VC_H */
