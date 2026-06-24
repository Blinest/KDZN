//
// Created by blin on 2026/5/2.
//

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "main.h"
#include "SCSerial.h"

extern osMessageQueueId_t MotorQueueHandle;

uint8_t wBuf[256];
uint8_t wLen = 0;



//UART 接收数据接口
int readSCS(unsigned char *nDat, int nLen)
{
	int received = 0;
	uint8_t byte;
	// 设置超时时间：200ms (可根据舵机响应速度调整)
	uint32_t timeout_ms = 200;
	
	for (int i = 0; i < nLen; i++) {
		// 修复：添加超时机制，防止无限等待导致卡死
		// osWaitForever 改为 200ms 超时
		if (osMessageQueueGet(MotorQueueHandle, &byte, NULL, timeout_ms) == osOK) {
			nDat[i] = byte;
			received++;
		} else {
			// 超时，返回已读取的字节数（可能小于 nLen）
			// 打印调试信息（可选）
			// printf("[ERR] readSCS timeout at byte %d/%d\n", i, nLen);
			break;
		}
	}
	return received;
}

//UART 发送数据接口
int writeSCS(unsigned char *nDat, int nLen)
{
	int written = 0;
	while(nLen--){
		if(wLen < sizeof(wBuf)){
			wBuf[wLen] = *nDat;
			wLen++;
			nDat++;
			written++;
		} else {
			// 修复：缓冲区满时返回错误，而不是静默丢弃
			return -1;  // 返回 -1 表示缓冲区溢出
		}
	}
	return written;  // 返回成功写入的字节数
}

int writeByteSCS(unsigned char bDat)
{
	if(wLen < sizeof(wBuf)){
		wBuf[wLen] = bDat;
		wLen++;
	} else {
		// 修复：缓冲区满时返回 -1
		return -1;
	}
	return wLen;
}

//接收缓冲区刷新
void rFlushSCS() {
	uint8_t dummy;
	uint32_t max_attempts = 10;  // 最多尝试 10 次
	while (max_attempts-- > 0 && osMessageQueueGet(MotorQueueHandle, &dummy, NULL, 0) == osOK);

}

//发送缓冲区刷新
void wFlushSCS() {
	if (wLen) {
		// 直接调用你的串口发送函数
		Usart_SendString(&huart1, wBuf, wLen);
		wLen = 0;
	}
}
