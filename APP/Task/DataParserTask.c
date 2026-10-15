/**
 * @file DataParserTask.c
 * @brief 数据解析任务：消费外设反馈字节并解析
 *
 * 数据来源：SensorDataQueue (由 USART2 中断接收回调入队)
 *   - USART2 RX 队列包含两类数据：
 *       1) CMCU-06 压力传感器响应（Modbus-RTU 协议）
 *       2) WT_IMU 数据帧（预留）
 *
 * 职责边界：
 *   - 本任务只负责「收字节 -> 解析 -> 更新全局状态」
 *   - 数据采集 / 打包 / 上报上位机 由 SensorDataTask 负责
 *
 * 上报约定 (2026-10-15 起)：
 *   - **只上报一帧** —— 0x55 01 00 状态帧 (78 字节), 200ms 周期;
 *   - 状态帧只含 6 路电机 + 6 路传感器 + CR 的 scale/armbend/state,
 *     张力感知数据与独立感知帧均已取消 (见 cmd_packer.h 状态帧布局)。
 *
 * @note 原实现位于 SensorDataTask.c 的「数据解析」段落，2026-10-01 拆分至此。
 *
 * @date 2026-10-01
 */


#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "Motor/Motor.h"
#include "CR/CR.h"
#include "Sensor/Sensor.h"
#include "Sensor/CMCU-06.h"
#include "Communicate/cmd_packer.h"
/* ==================== 周期配置 (ms) ==================== */
#define SEND_PERIOD_MS          200   /* 状态帧上报周期 (唯一上报帧) */


void StartDataParserTask(void *argument)
{
    uint8_t rx_byte = 0;
    uint32_t last_send_time = osKernelGetTickCount();  // 用启动时刻初始化
    (void)argument;

    for (;;)
    {
        /* 1. 收数据：带超时，超时也往下走 */
        if (osMessageQueueGet(SensorDataQueueHandle, &rx_byte, NULL, 10) == osOK)
        {
            uint32_t drain_cnt = 0;
            do {
                //CMCU_06_Parse_Byte(rx_byte);
                if (++drain_cnt >= 64) { taskYIELD(); drain_cnt = 0; }
            } while (osMessageQueueGet(SensorDataQueueHandle, &rx_byte, NULL, 0) == osOK);
        }

        /* 2. 周期上报: 状态帧, 200ms */
        const uint32_t now = osKernelGetTickCount();
        if ((now - last_send_time) >= SEND_PERIOD_MS)
        {
            static uint8_t packed_frame[256];
            const uint16_t frame_len =
                cmd_packer_pack_status_frame(packed_frame, global_motor,
                                             global_sensor, &CR, (uint8_t)CR.state);
            if (frame_len > 0 && frame_len <= sizeof(packed_frame))
            {
                Usart_SendString(&huart1, packed_frame, frame_len);   /* 上位机链路 = USART1 */
            }
            last_send_time = now;
        }

    }
}