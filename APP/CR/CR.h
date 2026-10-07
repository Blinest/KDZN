#ifndef __CR_H
#define __CR_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
/**********************************************************
***	编写作者：blinest

***	qq：1071378062
**********************************************************/

// 数学常数
#define PI 3.14159265358979323846
#define EPS 1e-9

// 分段微分模型几何常量 (SDM.h 依赖本文件, 故在此定义)
#define SDM_SEGMENTS        2
#define SDM_WIRES           6
#define SDM_WIRES_PER_SEG   3

// 曲率分布查询配置 (3D 共背骨多段模型: 由实测肌腱力正解 κ(s))
// 臂体**无中心杆**, 承载/弯曲全由菱形晶胞骨架承担 ⇒ EI0 与 k_rod 同源。
// 弯曲本构 (笔记《曲率分布函数κ(s)》§4.2):
//   弯矩      C(κ) = C0·sin(θ),  θ=|κ|·L_strut,  C0 = EI0/L_strut
//   割线刚度  EI_eq(κ) = C(κ)/κ = EI0·sinc(θ)   ← κ→0 时 → EI0 (有限非零)
//   切向刚度  dC/dκ = EI0·cos(θ)
// L_strut=5mm, N_circ=3, 晶胞分布半径 r_i=30mm, N_long=21:
#define SDM_CELL_E             1e9f   /* 晶胞杆材料模量 E (Pa) */
#define SDM_CELL_A             1e-6f  /* 单根斜杆截面积 A (m², a=1mm 方形) */
#define SDM_CELL_ALPHA_RAD     0.2618f/* 斜杆与横向夹角 α=15° (rad) */
#define SDM_CELL_L_STRUT       0.005f /* 晶胞斜杆长 L_strut (m); θ=|κ|·L_strut */
#define SDM_CELL_N_CIRC        3.0f   /* 环向胞数 */
#define SDM_CELL_N_LONG        21.0f  /* 纵向胞数 (=臂长/节距) */
#define SDM_CELL_R_I           0.030f /* 骨架菱形晶胞分布半径 r_i (m): 仅用于 EI0 估算 */

/* ---------------------------------------------------------------------------
 * 驱动丝分布半径 r_t (m) —— 全工程唯一来源
 *   用于: PCC 几何 ΔL_i = -R·θ·cos(φ+α_i) / 力臂 / 目标张力 / 张力感知
 *   取值: 5 mm (实测)
 * 注意: 与上方 SDM_CELL_R_I (骨架晶胞半径, 用于 EI0) 是不同物理量, 切勿混用。
 * --------------------------------------------------------------------------*/
#define SDM_DRIVE_RADIUS_M     0.005f

// k_unit = 2·E·A·sin²α/L_strut ≈ 26795 N/m
#define SDM_CELL_K_UNIT  (2.0f*SDM_CELL_E*SDM_CELL_A \
                          *sinf(SDM_CELL_ALPHA_RAD)*sinf(SDM_CELL_ALPHA_RAD) \
                          /SDM_CELL_L_STRUT)
// EI0 = k_unit·l_ax·Σy², l_ax=2·L_strut·sinα, Σy²=2·N_circ·r_i²  ≈ 0.3745 N·m²
#define SDM_CURV_EI0     (SDM_CELL_K_UNIT \
                          *(2.0f*SDM_CELL_L_STRUT*sinf(SDM_CELL_ALPHA_RAD)) \
                          *(2.0f*SDM_CELL_N_CIRC*SDM_CELL_R_I*SDM_CELL_R_I))
#define SDM_CURV_L_STRUT SDM_CELL_L_STRUT
#define SDM_CURV_DEFAULT_C0  (SDM_CURV_EI0/SDM_CURV_L_STRUT) /* = EI0/L_strut ≈ 74.9 N·m² */
#define SDM_ROD_K_AXIAL  (SDM_CELL_K_UNIT/SDM_CELL_N_LONG)   /* k_rod ≈ 1276 N/m */

#define SDM_CURV_KAPPA_MAX     50.0f  /* 曲率输出绝对饱和 (m⁻¹), 防 NaN/发散 */
#define SDM_CURV_MOMENT_MIN    1e-3f  /* 盘弯矩低于此视为伸直阈值 (N·m) */
#define SDM_CURV_ARRAY_MAX     40     /* 兼容保留: 旧标量输出数组上限 */
#define SDM_MULTISEG_MAX       1024   /* 3D 多段模型的子段数上限 (调用方 n_sub <= 此值) */

/* 位移-力自洽层求解器参数 */
#define SDM_DISP_TOL_M      1e-8f  /* GN 收敛判据 ‖Jᵀr‖∞ (m) */
#define SDM_DISP_MAX_ITER   20     /* GN 最大迭代 */
#define SDM_DISP_RIDGE      1e-10f /* 岭正则, 防法方程奇异 */
#define SDM_DISP_OK_MM      0.05f  /* rms 残差 < 此值视为刚性可达 */

typedef struct CR_Parameter
{
	float r[3];
} CR_Parameter;

typedef struct JointSpace
{
	float target_phi[SDM_SEGMENTS];
	float target_theta[SDM_SEGMENTS];
	float total_target_theta;
	float current_phi;
	float current_theta[SDM_SEGMENTS];
	float deltaL[SDM_WIRES];
} JointSpace;

typedef struct OperationSpace
{
	float scale;
}OperationSpace;

typedef struct ArmParams
{
	double L;//每段长度
	double tendon_preload; // 预紧力
	double friction_coeff; // 摩擦系数

	double backbone_stiffness; //臂体弯曲刚度
	double material_damping; //材料阻尼系数

	double calibrate_offset[3]; // 肌腱零点偏移量
	double direction_gain[4]; //方向增益，对应(u,r,d,l)
} ArmParams;

typedef struct ContinuumRobot
{
	JointSpace joint_space;
	OperationSpace operation_space;
	CR_Parameter parameter;
	ArmParams arm_params[2];
	bool state;
} ContinuumRobot;

void CR_init(void);
uint8_t armBend(int seg, char direction, double val);
uint8_t armBend_edit(int seg, char direction, double val, double g_u, double g_r, double g_d, double g_l, double seg1_limit, double seg2_limit);
void deltaL_update(void);
void auto_straight(void);
void armRotate(float theta_deg, float step_deg);
void action_group_demo(void);
int direction_to_index(char direction);
double tendonCompensation(int seg, char direction, double angle_deg);

extern ContinuumRobot CR;
#endif
