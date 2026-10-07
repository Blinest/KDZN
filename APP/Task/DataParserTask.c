/**
 * @file DataParserTask.c
 * @brief 数据解析任务：消费外设反馈字节并解析
 *
 * 数据来源：SensorDataQueue (由 USART1 中断接收回调入队)
 *   - USART1 RX 队列包含两类数据：
 *       1) CMCU-06 压力传感器响应（Modbus-RTU 协议）
 *       2) WT_IMU 数据帧（预留）
 *
 * 职责边界：
 *   - 本任务只负责「收字节 -> 解析 -> 更新全局状态」
 *   - 数据采集 / 打包 / 上报上位机 由 SensorDataTask 负责
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
#include "CR/Perception.h"
/* ==================== 周期配置 (ms) ==================== */
#define SEND_PERIOD_MS          200   /* 上报上位机周期 */
#define PERCEPT_SEND_PERIOD_MS  500   /* 张力感知帧上报周期 */
#define PERCEPT_REPORT_ENABLE     1   /* 1=上报 0x03 感知帧; 0=关闭(不影响既有 0x02 帧) */
void StartDataParserTask(void *argument)
{
    uint8_t rx_byte = 0;
    uint32_t last_send_time = osKernelGetTickCount();  // 用启动时刻初始化
    uint32_t last_percept_send_time = last_send_time;
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

        /* 2. 周期上报：独立判断 */
        const uint32_t now = osKernelGetTickCount();
        if ((now - last_send_time) >= SEND_PERIOD_MS)
        {
            static uint8_t packed_frame[256];
            const uint16_t frame_len =
                cmd_packer_pack_status_frame(packed_frame, global_motor,
                                             global_sensor, &CR, (uint8_t)CR.state);
            if (frame_len > 0 && frame_len <= sizeof(packed_frame))
            {
                Usart_SendString(&huart1, packed_frame, frame_len);
            }
            last_send_time = now;
        }

#if PERCEPT_REPORT_ENABLE
        /* 3. 张力感知帧上报 (较慢周期, 独立于 0x02 状态帧) */
        if ((now - last_percept_send_time) >= PERCEPT_SEND_PERIOD_MS)
        {
            static uint8_t percept_frame[64];
            const uint16_t plen = cmd_packer_pack_percept_frame(percept_frame, perception_get());
            if (plen > 0 && plen <= sizeof(percept_frame))
            {
                Usart_SendString(&huart2, percept_frame, plen);
            }
            last_percept_send_time = now;
        }
#endif
    }
}