#include "imu_task.h"
#include "spi.h"
#include "freertos.h"
#include "task.h"
#include "cmsis_os.h"
extern imu_t imu;
imu_offset offset;
osThreadId imuHandle;

void imu_init(void)
{
	MX_SPI5_Init();
	mpu_device_init();
	init_quaternion();
	HAL_Delay(100);
	for(int i=0;i<100;i++)
	{
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update();
		offset.yaw_offset+=imu.yaw;
		offset.pit_offset+=imu.pit;
		offset.rol_offset+=imu.rol;
	}
  offset.yaw_offset= offset.yaw_offset/100;
	offset.pit_offset= offset.pit_offset/100;
	offset.rol_offset= offset.rol_offset/100;
}

void imu_getdata(void)
{
	  mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update();
}

void imu_task(void const *argument)
{
	uint32_t imu_wake_time = osKernelSysTick();
	for(;;)
  {
		imu_getdata();
    osDelayUntil(&imu_wake_time, 2);  //imu's data update every 2ms 
  }
}

void create_IMU_Task(void)
{
	osThreadDef(canTask, imu_task, osPriorityRealtime, 0, 256);
	imuHandle = osThreadCreate(osThread(canTask), NULL);
}
	

	
