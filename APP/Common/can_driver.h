//
// Created by rosding on 2026/4/2.
//

#ifndef CONTROLSYSTEM_CAN_DRIVER_H
#define CONTROLSYSTEM_CAN_DRIVER_H

#include "main.h"
#include "fdcan.h"
#include "cmsis_os2.h"


// CAN消息结构
typedef struct {
	uint32_t id;        // CAN ID
	uint8_t data[8];    // 数据
	uint8_t len;        // 数据长度
	uint8_t format;     // 0:标准帧, 1:扩展帧
	uint8_t type;       // 0:数据帧, 1:远程帧
} CAN_Message_t;

// CAN设备地址定义
#define CAN_ID_MOTOR_BASE     0x100   // 电机基地址
#define CAN_ID_SENSOR_BASE    0x200   // 传感器基地址
#define CAN_ID_BROADCAST      0x7FF   // 广播地址

// 功能码定义
#define CAN_FUNC_MOTOR_CTRL   0x01    // 电机控制
#define CAN_FUNC_MOTOR_FB     0x02    // 电机反馈
#define CAN_FUNC_SENSOR_READ  0x03    // 传感器读取
#define CAN_FUNC_SENSOR_FB    0x04    // 传感器反馈
#define CAN_FUNC_SYSTEM_CTRL  0x05    // 系统控制

// 初始化函数
void CAN_Driver_Init(void);
void CAN_Driver_Start(void);

// 发送函数
void CAN_SendCmd(FDCAN_HandleTypeDef *hcan, uint8_t *cmd, uint8_t len);


// 接收函数
uint8_t CAN_Driver_Receive(CAN_Message_t* msg);

// Bus Off 恢复函数（在任务主循环中调用）
void CAN_BusOff_Recovery(void);




// 队列句柄
extern osMessageQueueId_t CAN_RxQueueHandle;
extern FDCAN_TxHeaderTypeDef TxHeader;
#endif //CONTROLSYSTEM_CAN_DRIVER_H
