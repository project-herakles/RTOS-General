#include "motorControlTask.h"
#include "Control.h"
#include "bool.h"

#include "freertos.h"
#include "task.h"
#include "cmsis_os.h"


int16_t             angle_inbet;  //Angle in between

rc_info_t           RcSig;       //= (rc_info_t)RcInit;
chassis_ctrl_t      ChassisSig;  //= (chassis_ctrl_t)chassisInit;
gimbal_ctrl_t       GimbalSig;   //= (gimbal_ctrl_t)gimbalInit;
func_t              funcSig;     //= (func_t)FuncInit;
ctrl_info_t         CtrlSig;     //= {.state={0,0,0,0},.gimbal_ctrl_ptr=&GimbalSig,.chassis_ctrl_ptr=&ChassisSig, .func_ptr=&funcSig};

int16_t feedback; //gimbal pitch motor feedback signal
									//Need to be changed to extern after locating the CAN feedback signal.

osThreadId motorControlHandle;

void motorControl_task(void const *argument)
{
	uint32_t motorControl_wake_time = osKernelSysTick();
	for(;;)
  {
		rcDealler(&CtrlSig,&feedback,&RcSig,&angle_inbet);
		/*Set the gimbal pitch motor feedback to be the angle_in_between*/
		refCalc(&RcSig, &CtrlSig, angle_inbet);
		
    osDelayUntil(&motorControl_wake_time, 15);
  }
}

void create_motorControl_Task(void)
{
	osThreadDef(motorControlTask, motorControl_task, osPriorityRealtime, 0, 256);
	motorControlHandle = osThreadCreate(osThread(motorControlTask), NULL);
}
