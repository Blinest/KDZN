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

typedef struct {
    int n_seg;          // 段数（从 arm_params 数量推断，这里固定为1）
    double *L;          // 每段长度数组（指向 arm_params[0].L 等）
    int dof;            // 总自由度 = 1(d) + 2*n_seg + 1(ar)
} robot_params_t;

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
