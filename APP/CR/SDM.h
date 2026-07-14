/**
 * @file SDM.h
 * @brief 分段微分模型 (Segmented-Differential Model) — 力位混合控制
 *
 * 对外接口只暴露 sdm_init 和 sdm_step。
 * SDM 模块内部持有所有状态 (参数、k_ratio、控制状态)，
 * 外部 (CR.c) 只需设置期望值、调用 sdm_step、把结果发给电机。
 *
 * @date 2026-06-19
 * @author blinest
 */

#ifndef __SDM_H
#define __SDM_H

#include <stdbool.h>
#include "Sensor/Sensor.h"
#include "CR.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 驱动丝/段几何常量（借用 CR.h 中定义） */
#define SDM_SEGMENTS      SEGMENT_COUNT
#define SDM_WIRES         WIRE_COUNT
#define SDM_WIRES_PER_SEG WIRES_PER_SEG

/**
 * @brief SDM quasi-static dynamics parameters.
 *
 * The constitutive law used by the model is
 *   EI(kappa) = K0 * L_segment + a2 * kappa^2,
 * where K0 is the zero-curvature angular stiffness passed to sdm_init().
 */
typedef struct {
    float segment_mass[SDM_SEGMENTS]; /**< Lumped mass at each segment midpoint (kg). */
    float curvature_stiffness;        /**< Nonlinear coefficient a2 (N*m^4). */
    float tendon_pretension;          /**< Minimum common tendon pretension (N). */
    float force_relaxation;           /**< Target-force relaxation, range [0.1, 1]. */
} SDM_DynamicsConfig;

/* ==================== 对外 API ==================== */


void sdm_init(float bending_stiffness,
              float force_peak_limit,
              float force_recovery,
              float tip_mass,
              const float mount_dir[3]);


void sdm_kinematic_step(const float forces[SENSOR_NUM],
                         const float theta_desired[SDM_SEGMENTS],
                         const float phi_desired[SDM_SEGMENTS],
                         float R,
                         float deltaL_out[SDM_WIRES]);
static void _calculate_L(float R, const float theta[SDM_SEGMENTS],
                         const float phi[SDM_SEGMENTS], float deltaL[SDM_WIRES]);

/**
 * @brief Configure the distributed-mass and nonlinear constitutive model.
 * @note Passing NULL restores the safe defaults (zero segment mass, linear
 *       stiffness, zero pretension and a relaxation factor of 0.3).
 */
void sdm_configure_dynamics(const SDM_DynamicsConfig *config);

void sdm_step(const float forces[SENSOR_NUM],
              const float theta_desired[SDM_SEGMENTS],
              const float phi_desired[SDM_SEGMENTS],
              const float deltaL_actual[SDM_WIRES],
              float K_force,
              float R,
              float deltaL_out[SDM_WIRES]);


float sdm_get_force_peak_limit(void);

/** @brief 直驱所有电机回零（绕过 SDM 模型，发绝对位置 0） */
void sdm_auto_straight(void);

#ifdef __cplusplus
}
#endif

#endif /* __SDM_H */
