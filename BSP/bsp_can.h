#ifndef _PID_H_
#define _PID_H_

#include "main.h"
#include "stm32f4xx_hal.h"

void CAN_Initialize(void);
void CAN_SendMsg(CAN_HandleTypeDef* hcan,CAN_TxHeaderTypeDef *canTxHeader,uint8_t* canMsg);

extern void _Error_Handler(char *, int);

#endif
