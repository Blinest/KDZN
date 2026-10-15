/**
*  @file PeriphCtrlTask.c
 * @brief 指令控制任务：从队列取串口数据，交给 cmd_parse 解析并执行
 * @author blin
 *
 * 指令解析与电机/传感器控制逻辑在 Common/cmd_parse 中实现。
 */

#include "cmsis_os2.h"
#include "main.h"
#include "string.h"
#include "Communicate/can_driver.h"

#include "stdio.h"
#include "usart.h"

#include "Communicate/pc_cmd_parser.h"

bool is_connected = false;   // false:未连接 true:已连接

void StartCmdCtrlTask(void *argument)
{
	uint8_t receive;
	(void)argument;

	// ==================== 指令解析任务 ====================
	for (;;)
	{
		// Bus Off 延迟恢复（检查标志并在任务上下文中恢复CAN）
		CAN_BusOff_Recovery();

		// 从队列接收上位机指令，逐字节送入解析状态机
		/* 先阻塞等 10ms 拿第一个字节，再一次性排空队列：
		   1 字节/10ms 的处理速度会让长帧在 16 字节队列里溢出丢字节 */
		if (osMessageQueueGet(CmdQueueHandle, &receive, NULL, 10) == osOK)
		{
			pc_cmd_parser_feed_byte(receive);

			while (osMessageQueueGet(CmdQueueHandle, &receive, NULL, 0) == osOK)
			{
				pc_cmd_parser_feed_byte(receive);
			}
		}
	}
}
