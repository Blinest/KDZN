/**
 * @file InitTask.c
 * @brief 上电初始化任务：归中 + 传感器归零 + 压力标定
 *
 * 重写 StartDefaultTask（弱函数），在 DataTask 完成一轮数据采集后
 * 执行电机归中、传感器去皮、压力灵敏度标定。
 *
 * 注意：defaultTask 堆栈在 freertos.c 中定义（128*4=512），
 *       这里调用 cr_kinematic_step 和 sdm_step 需要更大栈空间，
 *       但通过 Cubemx 修改默认堆栈或在任务属性中定义更大的栈。
 *
 * @date 2026-07-05
 * @author blin
 */

#include "cmsis_os2.h"
#include "CR/CR.h"
#include "Motor/Motor.h"
#include "Sensor/Sensor.h"
/* ==================== 重写 defaultTask ==================== */

void StartDefaultTask(void *argument)
{
    /* 等待 CAN 总线稳定 + DataTask 完成多轮数据采集 */
    osDelay(3000);

    /* 1. 读取电机状态（等待 DataTask 完成多轮采集，确保 CAN 回复已更新） */
    motor_status_check();
    osDelay(2000);

    /* 如果还是没读到数据，再补一次 */
    bool pos_ready = false;
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (fabsf(global_motor[i].stepper_motor.current_pos) > 0.1f) {
            pos_ready = true;
            break;
        }
    }
    if (!pos_ready) {
        motor_status_check();
        osDelay(2000);
    }

    /* 2. 臂体归中（无论是否读到位置，都发送归零指令） */
    auto_straight();
    osDelay(500);

    /* 3. 传感器归零（去皮） */
    sensor_reset();
    osDelay(300);

    /* 4. 压力灵敏度自动标定 */
    CR_calibrate_pressure_sensitivity();

    /* 初始化完成，后续循环保持最低 CPU 占用 */
    for (;;) {
        osDelay(1);
    }
}
