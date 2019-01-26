#include "canTask.h"
#include "bsp_can.h"
#include "bsp_motor.h"
#include "encoder.h"
#include "freertos.h"
#include "task.h"
#include "cmsis_os.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef can1TxHeader0;
CAN_RxHeaderTypeDef can1RxHeader;
uint8_t canRxMsg[8] = {0};
uint32_t can_count = 0;

extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder; 

osThreadId canHandle;

void CanReceiveMsgProcess(CAN_RxHeaderTypeDef *rxHeader,uint8_t* msg)
{   
		can_count++;
		switch(rxHeader->StdId)
		{
				case CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM1Encoder ,rxHeader,msg):EncoderProcess(&CM1Encoder ,msg);					
				}break;
				case CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM2Encoder ,rxHeader,msg):EncoderProcess(&CM2Encoder ,msg);
				}break;
				case CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM3Encoder ,rxHeader,msg):EncoderProcess(&CM3Encoder ,msg);
				}break;
				case CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID:
				{
					(can_count<=50) ? GetEncoderBias(&CM4Encoder ,rxHeader,msg):EncoderProcess(&CM4Encoder ,msg);
				}break;
				case CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID:
				{
				}break;
				
				case CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID:
				{	
				}break;		
				case CAN_BUS2_MOTOR7_FEEDBACK_MSG_ID:
				{
				}break;
				case CAN_BUS2_MOTOR8_FEEDBACK_MSG_ID:
				{
				}break;
				
		}

}

void can_task(void const *argument)
{
	uint32_t can_wake_time = osKernelSysTick();
	for(;;)
  {
		if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) >= 1)
		{
			HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1RxHeader, canRxMsg);
			CanReceiveMsgProcess(&can1RxHeader,canRxMsg);
		}
    osDelayUntil(&can_wake_time, 5);
  }
}

void create_CAN_Task(void)
{
	osThreadDef(canTask, can_task, osPriorityRealtime, 0, 256);
	canHandle = osThreadCreate(osThread(canTask), NULL);
}
