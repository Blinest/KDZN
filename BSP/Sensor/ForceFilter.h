/**
 * @file ForceFilter.h
 * @brief 六路肌腱张力信号调理: 尖峰抑制 + 滑动中值 + 一阶低通
 *
 * 现场现象: 触碰变送器模块会导致张力示数剧烈跳变、模型推断力误差大。
 * 软件侧对策(按顺序):
 *   1) 尖峰门限 : 单次采样跳变 > FORCE_SPIKE_N 视为冲击, 用上次值替代
 *   2) 3 点中值 : 抑制孤立野点
 *   3) 一阶 IIR  : 平滑残余宽带噪声
 *
 * 分工: 本模块输出「滤波值」(global_sensor[].x), 原始值保留在 global_sensor[].y
 *       供上位机对比诊断。
 *
 * @note 若干扰表现为「触碰后读数整体平移并保持」, 属机械安装刚性/屏蔽接地/线缆
 *       固定的问题, 软件滤波无法消除, 必须从硬件侧解决。软件只能压制跳变与噪声。
 *
 * @date 2026-10-03
 */
#ifndef __FORCE_FILTER_H
#define __FORCE_FILTER_H

#include <stdint.h>
#include <stdbool.h>
#include "Sensor.h"   /* SENSOR_NUM */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 可调参数 ---- */
#ifndef FORCE_SPIKE_N
#define FORCE_SPIKE_N      15.0f   /* 单次跳变门限 (N): 超过则判为冲击并丢弃 */
#endif
#ifndef FORCE_IIR_ALPHA
#define FORCE_IIR_ALPHA     0.35f  /* IIR 系数 (0~1), 越大响应越快、平滑越弱 */
#endif
#define FORCE_MED_WIN        3     /* 中值窗口长度 */

/**
 * @brief 初始化(清零)全部通道滤波器
 */
void force_filter_init(void);

/**
 * @brief 送入一路新采样, 返回滤波值
 * @param ch  通道号 0..SENSOR_NUM-1
 * @param raw 原始张力 (N)
 * @return 滤波后的张力 (N); ch 越界时原样返回
 */
float force_filter_update(uint8_t ch, float raw);

/**
 * @brief 读取某通道当前滤波值 (不更新)
 */
float force_filter_get(uint8_t ch);

/**
 * @brief 读取全部通道当前滤波值
 * @param out 输出数组 [SENSOR_NUM]
 */
void force_filter_get_all(float out[SENSOR_NUM]);

/* ==================== 通道平衡系数 (比值标定) ====================
 * 6 根驱动丝等量同向移动时, 理论张力比应为 1:1:1:1:1:1。
 * 实测偏离反映「通道增益 + 装配/摩擦不对称」, 用 g_i = mean(τ)/τ_i 修正。
 * 修正后再整体归一化 ⇒ 不改变绝对尺度 (比值法只关心相对量)。
 *
 * 平衡系数在 force_filter_update()/get() 的输出端相乘,
 * 因此 global_sensor[].x 及所有下游(SDM/感知/上报)自动一致。
 * 初始值均为 1.0 (不修正)。
 */

/** 直接设置某通道平衡系数 (gain<=0 将被忽略) */
void  force_filter_set_balance(uint8_t ch, float gain);

/** 读取某通道平衡系数 (越界返回 1.0) */
float force_filter_get_balance(uint8_t ch);

/** 读取全部平衡系数 */
void  force_filter_get_balance_all(float out[SENSOR_NUM]);

/** 全部复位为 1.0 */
void  force_filter_reset_balance(void);

#ifdef __cplusplus
}
#endif

#endif /* __FORCE_FILTER_H */
