#include "userRtos.h"
#include "chassisTask.h"

void RTOS_Initialize(void)
{
	// place tasks here
	create_CAN_Task();
	create_chassis_Task();
}
