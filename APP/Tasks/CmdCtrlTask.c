/**
*  @file CmdCtrlTask.c
 * @brief 指令控制任务：从队列取串口数据，交给 cmd_parse 解析并执行
 * @author blin
 *
 * 指令解析与电机/传感器控制逻辑在 Common/cmd_parse 中实现。
 */

#include "cmsis_os2.h"
#include "main.h"
#include "string.h"
#include "Common/can_driver.h"

#include "stdio.h"
#include "usart.h"

#include "Common/pc_cmd_parser.h"

extern osMessageQueueId_t CmdCtrlQueueHandle;

#define RX_BUF_SIZE 256


bool is_connected = false;   // false:未连接 true:已连接

void StartCmdCtrlTask(void *argument)
{
	uint8_t rx_buffer[RX_BUF_SIZE];
	uint16_t rx_len = 0;
    // 测试串口用
    uint8_t test_msg[] = "send to usart1\r\n";
    uint8_t receive;

    // ==================== 指令解析任务 ====================
    for (;;)
    {
	    // Bus Off 延迟恢复（检查标志并在任务上下文中恢复CAN）
	    CAN_BusOff_Recovery();

	    // 从队列接收上位机指令
	    if (osMessageQueueGet(CmdCtrlQueueHandle, &receive, NULL, 0) == osOK) {
	        // 优先回显，提高响应性
	        if (rx_len < RX_BUF_SIZE) {
	            rx_buffer[rx_len++] = receive;
	        	// 进入指令解析函数，指令解析函数负责指令解析，而后再将解析好的指令传给电机进行解析
	        	pc_cmd_parser_feed_byte(receive);
	        } else {
                rx_len = 0; // 缓冲区溢出重置
            }
	    }

		osDelay(10); // 降低 CPU 占用
	}
}