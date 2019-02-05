#include "freertos.h"
#include "task.h"
#include "cmsis_os.h"
#include "encoder.h"
#include "pid.h"
#include "bsp_motor.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "chassisTask.h"
#include "Control.h"

volatile Encoder CM1Encoder;
volatile Encoder CM2Encoder;
volatile Encoder CM3Encoder;
volatile Encoder CM4Encoder;  
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef can1TxHeader0;

extern rc_info_t RcSig;
extern ctrl_info_t CtrlSig;

osThreadId chassisHandle;

void send_Chassis_Msg(CAN_HandleTypeDef* hcan, int16_t cm1_iq,int16_t cm2_iq,int16_t cm3_iq,int16_t cm4_iq)
{
		uint8_t canTxMsg0[8];
    canTxMsg0[0] = (uint8_t)(cm1_iq >> 8);
    canTxMsg0[1] = (uint8_t)cm1_iq;
    canTxMsg0[2] = (uint8_t)(cm2_iq >> 8);
    canTxMsg0[3] = (uint8_t)cm2_iq;
    canTxMsg0[4] = (uint8_t)(cm3_iq >> 8);
    canTxMsg0[5] = (uint8_t)cm3_iq;
    canTxMsg0[6] = (uint8_t)(cm4_iq >> 8);
    canTxMsg0[7] = (uint8_t)cm4_iq;
    CAN_TxHeaderTypeDef msgHeader = can1TxHeader0;
		HAL_CAN_AddTxMessage(hcan,&msgHeader,canTxMsg0,(void*)CAN_TX_MAILBOX0);
}

void chassis_remoteControl(chassis_ctrl_t * chassis_ref, rc_info_t * rc)
{
	osDelay(1);
	CM1SpeedPID.ref =  (-chassis_ref->forward_back_speed_ref*0.075 + chassis_ref->left_right_speed_ref*0.075 + chassis_ref->rotation_speed_ref*0.075)*18;
	CM2SpeedPID.ref = (chassis_ref->forward_back_speed_ref*0.075 + chassis_ref->left_right_speed_ref*0.075 + chassis_ref->rotation_speed_ref*0.075)*18;
	CM3SpeedPID.ref = (chassis_ref->forward_back_speed_ref*0.075 - chassis_ref->left_right_speed_ref*0.075 + chassis_ref->rotation_speed_ref*0.075)*18;
	CM4SpeedPID.ref = (-chassis_ref->forward_back_speed_ref*0.075 - chassis_ref->left_right_speed_ref*0.075 + chassis_ref->rotation_speed_ref*0.075)*18; 
  CM1SpeedPID.fdb = CM1Encoder.filter_rate;
	CM2SpeedPID.fdb = CM2Encoder.filter_rate;
	CM3SpeedPID.fdb = CM3Encoder.filter_rate;
	CM4SpeedPID.fdb = CM4Encoder.filter_rate;
  PID_Calc(&CM1SpeedPID);
	PID_Calc(&CM2SpeedPID);
	PID_Calc(&CM3SpeedPID);
	PID_Calc(&CM4SpeedPID);
	send_Chassis_Msg(&hcan1, (CM1SpeedPID.output*SPEED_OUTPUT_ATTENUATION),CM2SpeedPID.output*SPEED_OUTPUT_ATTENUATION,CM3SpeedPID.output*SPEED_OUTPUT_ATTENUATION,(CM4SpeedPID.output*SPEED_OUTPUT_ATTENUATION));
	if(rc->sw1==2 && rc->sw2==2)
	{
	   CM1SpeedPID.ref = 0;
	   CM2SpeedPID.ref = 0;
	   CM3SpeedPID.ref = 0;
	   CM4SpeedPID.ref = 0;
	}
}

// BELOW FUNCTION IS FOR DEBUGGING
void set_Chassis_Pid_Speed(int cm1, int cm2, int cm3, int cm4)
{
	osDelay(1);
	CM1SpeedPID.ref = cm1;
	CM2SpeedPID.ref = cm2;
	CM3SpeedPID.ref = cm3;
	CM4SpeedPID.ref = cm4; 
  CM1SpeedPID.fdb = CM1Encoder.filter_rate;
	CM2SpeedPID.fdb = CM2Encoder.filter_rate;
	CM3SpeedPID.fdb = CM3Encoder.filter_rate;
	CM4SpeedPID.fdb = CM4Encoder.filter_rate;
  PID_Calc(&CM1SpeedPID);
	PID_Calc(&CM2SpeedPID);
	PID_Calc(&CM3SpeedPID);
	PID_Calc(&CM4SpeedPID);
	send_Chassis_Msg(&hcan1, (CM1SpeedPID.output*SPEED_OUTPUT_ATTENUATION),CM2SpeedPID.output*SPEED_OUTPUT_ATTENUATION,CM3SpeedPID.output*SPEED_OUTPUT_ATTENUATION,(CM4SpeedPID.output*SPEED_OUTPUT_ATTENUATION));
}

void chassis_task(void const *argument)
{
	uint32_t chassis_wake_time = osKernelSysTick();
	for(;;)
  {
		chassis_remoteControl(CtrlSig.chassis_ctrl_ptr, &RcSig);
    osDelayUntil(&chassis_wake_time, 5);
  }
}

void create_chassis_Task(void)
{
	osThreadDef(chassisTask, chassis_task, osPriorityRealtime, 0, 256);
	chassisHandle = osThreadCreate(osThread(chassisTask), NULL);
}
