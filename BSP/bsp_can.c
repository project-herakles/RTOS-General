#include "bsp_can.h"

extern CAN_HandleTypeDef hcan1;	// this is defined in main
//uint8_t canRxMsg[8] = {0};
CAN_TxHeaderTypeDef can1TxHeader0;
CAN_TxHeaderTypeDef can1TxHeader1;
//CAN_RxHeaderTypeDef can1RxHeader;
CAN_FilterTypeDef can1Filter;
//uint32_t can_count=0;

uint8_t canTxMsg0[8] = {0};
uint8_t canTxMsg1[8] = {0};

// CAN_Initialize should be in main, placed after MX_CAN1_Init
void CAN_Initialize(void)
{
	hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	can1TxHeader0.IDE = CAN_ID_STD;
	can1TxHeader0.StdId = 0x200;
	can1TxHeader0.DLC = 8;
	
	can1TxHeader1.IDE = CAN_ID_STD;
	can1TxHeader1.StdId = 0x1FF;
	can1TxHeader1.RTR = CAN_RTR_DATA;
	can1TxHeader1.DLC = 8;
	
	
	can1Filter.FilterActivation = ENABLE;
	can1Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can1Filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can1Filter.FilterFIFOAssignment = CAN_FilterFIFO0;
	can1Filter.FilterIdHigh = 0x0000;
	can1Filter.FilterIdLow = 0x0000;
	can1Filter.FilterBank = 0;
	HAL_CAN_ConfigFilter(&hcan1,&can1Filter);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_FULL);
	HAL_CAN_Start(&hcan1);
}

// CAN transmit message
void CAN_SendMsg(CAN_HandleTypeDef* hcan,CAN_TxHeaderTypeDef *canTxHeader,uint8_t* canMsg)
{
	HAL_CAN_AddTxMessage(hcan,canTxHeader,canMsg,(void*)CAN_TX_MAILBOX0);
}
