#include "can_driver.h"
#include <string.h>
#include "cmsis_os2.h"
#include "usart.h"
#include "Motor/Motor.h"
#include "XV2_cmd_parser.h"
// CAN接收队列
osMessageQueueId_t CAN_RxQueueHandle;
FDCAN_TxHeaderTypeDef TxHeader;
/**
 * @brief CAN驱动初始化
 */
void CAN_Driver_Init(void)
{
    // 创建CAN接收队列
    CAN_RxQueueHandle = osMessageQueueNew(32, sizeof(CAN_Message_t), NULL);

    // 启动CAN
    HAL_FDCAN_Start(&hfdcan1);

	/* 1. 配置全局过滤 —— 接收所有标准帧和扩展帧 */
	FDCAN_FilterTypeDef sFilterConfig = {0};
	sFilterConfig.IdType       = FDCAN_STANDARD_ID;        // 先配置标准ID过滤
	sFilterConfig.FilterIndex  = 0;                        // 过滤器索引
	sFilterConfig.FilterType   = FDCAN_FILTER_MASK;        // 掩码模式
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  // 匹配的报文存入 RX FIFO0
	sFilterConfig.FilterID1    = 0;                        // 标准 ID 全为 0
	sFilterConfig.FilterID2    = 0;                        // 掩码全 0 → 任意 ID 都通过
	HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

	// 扩展帧也要配一次
	sFilterConfig.IdType       = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex  = 1;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1    = 0;
	sFilterConfig.FilterID2    = 0;
	HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

	/* 2. 激活中断 */
	HAL_FDCAN_ActivateNotification(&hfdcan1,
								   FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan1,
								   FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan1,
								   FDCAN_IT_TX_COMPLETE, 0);

	// 激活 Bus Off 错误中断，用于自动恢复
	HAL_FDCAN_ActivateNotification(&hfdcan1,
								   FDCAN_IT_BUS_OFF, 0);
}

/**
	* @brief   CAN发送多个字节
	* @param   无
	* @retval  无
	*/

void CAN_SendCmd(FDCAN_HandleTypeDef *hfdcan, uint8_t *cmd, uint8_t len)
{
	if (cmd == NULL || len < 2) return;

	uint8_t addr = cmd[0];          // 设备地址
	uint8_t func = cmd[1];          // 功能码
	uint8_t data_len = len - 2;     // 实际数据长度（不含地址和功能码）
	uint8_t *data = &cmd[2];        // 数据起始指针

	FDCAN_TxHeaderTypeDef TxHeader = {0};  // 缺省清零，防止未初始化字段

	// ---- FDCAN 特有的帧格式设置 ----
	TxHeader.IdType = FDCAN_EXTENDED_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;   // 数据帧
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;     // 用经典 CAN 帧（非 FD 长帧）
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF; //不切换速率
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;     // 显性：节点主动错

	uint8_t offset = 0;
	uint8_t pack_idx = 0;
	uint8_t tx_data[8];

	while (offset < data_len)
	{
		uint8_t remain = data_len - offset;
		uint8_t send_data_len = (remain > 7) ? 7 : remain;   // 每帧最多7字节数据
		TxHeader.DataLength = send_data_len + 1;  // FDCAN 用 DataLength 字段
		TxHeader.Identifier = ((uint32_t)addr << 8) | pack_idx;   // ID = (地址 << 8) | 包序号

		tx_data[0] = func;   // 每帧第一字节固定为功能码
		for (uint8_t i = 0; i < send_data_len; i++) {
			tx_data[1 + i] = data[offset + i];
		}

		// 等待 TX FIFO 有空间
		uint32_t timeout = 5000;
		while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0 && timeout-- > 0) {
			for (volatile int d = 0; d < 480; d++);  // ~1us per iteration @480MHz
		}

		if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, tx_data) != HAL_OK) {
			HAL_FDCAN_Stop(hfdcan);
			osDelay(20);  // 等待总线错误帧消散（CAN协议要求 128×11 隐性位）
			HAL_FDCAN_Start(hfdcan);
			// 重新激活中断通知
			HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
			HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_COMPLETE, 0);
			HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, tx_data);
		}

		// 帧间延时：给总线时间发送，避免 burst 堵塞
		for (volatile int d = 0; d < 24000; d++);  // ~50us

		offset += send_data_len;
		pack_idx++;
	}
}


/**
	* @brief   FDCAN_RX0接收中断
	* @param   无
	* @retval  无
	*/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t rx_data[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, rx_data) == HAL_OK)
    {
        if (RxHeader.IdType == FDCAN_EXTENDED_ID)
        {
            uint32_t ext_id = RxHeader.Identifier;
            for (int i = 0; i < MOTOR_NUM; i++)
            {
                if (global_motor[i].id == (ext_id >> 8))
                {
                    X_V2_parse_can(&global_motor[i], ext_id, rx_data, RxHeader.DataLength, true);
                    break;
                }
            }

        }
    }
}

/**
 * @brief FDCAN Bus Off 错误回调 — 自动恢复
 *
 * 当发送错误计数器(TEC)达到256时，FDCAN进入Bus Off状态。
 * 此回调在中断上下文中执行，仅设置标志位，
 * 由 CAN_BusOff_Recovery() 在任务上下文中执行实际恢复。
 */
volatile uint8_t fdcan_bus_off_flag = 0;

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance->PSR & FDCAN_PSR_BO)  // Bus Off 状态位
    {
        fdcan_bus_off_flag = 1;  // 仅设置标志，不在ISR中做恢复
    }
}

/**
 * @brief Bus Off 延迟恢复函数 — 在任务主循环中调用
 *
 * 检测到Bus Off标志后，执行停止→等待→重启→重配流程。
 * 应在主任务循环中周期性调用。
 */
void CAN_BusOff_Recovery(void)
{
    if (fdcan_bus_off_flag == 0) return;
    fdcan_bus_off_flag = 0;

    HAL_FDCAN_Stop(&hfdcan1);
    // CAN协议要求 128×11=1408 个隐性位才能退出Bus Off
    // 500kbps下约 2.8ms，留余量等待 20ms
    osDelay(20);
    HAL_FDCAN_Start(&hfdcan1);

    // 重新配置过滤器（Stop/Start后可能丢失）
    FDCAN_FilterTypeDef sFilterConfig = {0};
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex  = 0;
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0;
    sFilterConfig.FilterID2    = 0;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

    sFilterConfig.IdType       = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex  = 1;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1    = 0;
    sFilterConfig.FilterID2    = 0;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

    // 重新激活中断通知
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);
}

/**
 * @brief 从队列接收CAN消息
 */
uint8_t CAN_Driver_Receive(CAN_Message_t* msg)
{
    if (osMessageQueueGet(CAN_RxQueueHandle, msg, NULL, 0) == osOK) {
        return 1;
    }
    return 0;
}



