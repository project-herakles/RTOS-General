#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "bsp_imu.h"
#include "main.h"
typedef struct
{
	int16_t pit_offset;
	int16_t yaw_offset;
	int16_t rol_offset;
} imu_offset;

void imu_init(void);
void imu_getdata(void);
void create_IMU_Task(void);
	
#endif
