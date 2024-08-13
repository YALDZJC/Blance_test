#pragma once
#include "RM_stm32fxxx_hal.h"
#include "stm32h7xx_hal_fdcan.h"

void RM_FDCAN_Filter_Init()
{
	//通用初始化
	FDCAN_FilterTypeDef Filter0;
	Filter0.IdType=FDCAN_STANDARD_ID;                       //标准ID
    Filter0.FilterIndex=0;                                  //滤波器索引                   
    Filter0.FilterType=FDCAN_FILTER_MASK;                   //滤波器类型
    Filter0.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
    Filter0.FilterID1=0x0000;                               
    Filter0.FilterID2=0x0000;                               
	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &Filter0) != HAL_OK)
	{
		Error_Handler();
	}	
	
	FDCAN_FilterTypeDef Filter1;
	Filter1.IdType=FDCAN_STANDARD_ID;                       //标准ID
    Filter1.FilterIndex=1;                                  //滤波器索引                   
    Filter1.FilterType=FDCAN_FILTER_MASK;                   //滤波器类型
    Filter1.FilterConfig=FDCAN_FILTER_TO_RXFIFO1;           //过滤器1关联到FIFO1
    Filter1.FilterID1=0x0000;                               
    Filter1.FilterID2=0x0000;                               
    HAL_FDCAN_ConfigFilter(&hfdcan2,&Filter1); 							//滤波器初始化
	  HAL_FDCAN_ConfigFilter(&hfdcan3,&Filter1); 							//滤波器初始化

}

void RM_FDCan_Init()
{
	RM_FDCAN_Filter_Init();
	//开启can1
    HAL_FDCAN_Start(&hfdcan1);                              			  //开启FDCAN
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	//开启can2
    HAL_FDCAN_Start(&hfdcan2);                              			  //开启FDCAN
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
	//开启can3
    HAL_FDCAN_Start(&hfdcan3);                              			  //开启FDCAN
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}

void RM_FDCan_Send(FDCAN_HandleTypeDef* han,uint32_t StdId,uint8_t* s_data)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.Identifier = StdId; // 标准ID
    TxHeader.IdType = FDCAN_STANDARD_ID; // 标准ID类型
    TxHeader.TxFrameType = FDCAN_DATA_FRAME; // 数据帧
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; // 数据长度代码，8字节
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 错误状态指示器
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // 关闭位速率切换
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN; // 经典CAN格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 不使用事件FIFO
    TxHeader.MessageMarker = 0; // 消息标记
	
    // 发送消息
    if (HAL_FDCAN_AddMessageToTxFifoQ(han, &TxHeader, s_data) != HAL_OK)
    {
        // 处理错误
    }
}
