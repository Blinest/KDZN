/**
 * @file SensorDataTask.c
 * @brief 数据采集与上报任务
 *
 * 职责：
 *   1. 周期性采集外设状态：电机位置/速度、6 路压力传感器数据
 *   2. 周期性把系统状态打包成帧
 *   3. 通过 USART1 TX 上报给上位机
 *
 * 说明：
 *   外设反馈字节的「指令解析」已拆分至 DataParserTask.c，
 *   本任务不再从队列中解析指令，只做采集 + 打包 + 发送。
 *
 * @date 2026-03-30
 * @author Psyduck
 */

#include <stdio.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

#include "Motor/Motor.h"
#include "Sensor/Sensor.h"
#include "CR/CR.h"
#include "CR/Perception.h"
#include "CR/ForceBalance.h"
#include "Communicate/cmd_packer.h"

/* ==================== 周期配置 (ms) ====================
 * 串口分配: USART1 @115200 = 上位机链路 ; USART2 @9600 = CMCU-06 传感器总线 */
#define MOTOR_CHECK_PERIOD_MS   200   /* 电机状态采集周期 */
#define SENSOR_POLL_PERIOD_MS    15   /* 压力传感器轮询步进(6 路串行, 全部应答时 ≈90ms/轮 → 每路 ≈11Hz) */
#define PERCEPT_PERIOD_MS        20   /* 六路张力感知更新周期 (50Hz, 目前未启用) */

void StartSensorDataTask(void *argument)
{
    uint32_t last_motor_check_time = 0;
    uint32_t last_sensor_read_time = 0;
    uint32_t last_percept_time     = 0;

    (void)argument;

    /* 启动上电自动通道平衡标定 (非阻塞, 由下面的 tick 推进) */
    force_balance_start();

    for (;;)
    {
        const uint32_t current_time = osKernelGetTickCount();

        /* 标定状态机: 6 路同向等量 → 采样 → 生成通道平衡系数 */
        //force_balance_tick(current_time);

        /* 每 200ms 读取一次电机状态（位置、速度、使能标志） */
        if ((current_time - last_motor_check_time) >= MOTOR_CHECK_PERIOD_MS)
        {
            motor_status_check();
            /* 汇总 6 路实际使能状态 → CR.state (全部使能=1, 有任一路未使能=0) */
            motor_update_enable_state();
            last_motor_check_time = current_time;
        }

        /* 压力传感器轮询: 6 路共用 USART2 总线, 串行推进(每步至多发一条请求) */
        if ((current_time - last_sensor_read_time) >= SENSOR_POLL_PERIOD_MS)
        {
            sensor_poll_step(SENSOR_RESP_TIMEOUT_MS);
            last_sensor_read_time = current_time;
        }

        /* 六路张力感知: 残差 / 接触 / 肌腱健康
         * 标定期间跳过, 避免把标定动作(6路等量回缩)误判为干扰 */
        if (!force_balance_is_busy() &&
            (current_time - last_percept_time) >= PERCEPT_PERIOD_MS)
        {
            //perception_update_from_global();
            last_percept_time = current_time;
        }

        osDelay(10); /* 降低 CPU 占用 */
    }
}
