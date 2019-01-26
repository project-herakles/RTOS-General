#ifndef _CONTROLTASK_H_
#define _CONTROLTASK_H_
#include "stm32f4xx.h"
typedef enum
{
	WAKE_STATE,
	PREPARE_STATE,
	NORMAL_STATE,
	COLLECT_STATE,
	STOP_STATE,
}workState_e;

typedef enum
{
	ACUTE,
	RIGHT,
	OBTUSE,
}ArmMode_e;

typedef enum
{
	CLAW_ENABLE,
	CLAW_DISABLE,
}ClawMode_e;

typedef enum
{
	REST,
	HOLD,
	DUMP,
}DumperMode_e;

typedef struct CollectMode
{
	ArmMode_e armMode;
	ClawMode_e clawMode;
	DumperMode_e dumperMode;
}CollectMode;

#define PREPARE_TIME_TICK_MS 4000
#define CHASSIS_SPEED_ATTENUATION (1.0f)
#define SPEED_OUTPUT_ATTENUATION (1.0f)


void CM_Control(void);
void Collect_Control(void);
void workStateFSM(void);
void setWorkState(workState_e workState);
workState_e getWorkState(void);
void contorlTaskInit(void);
void CollectModeSwitch(void);
void Rescue_Control(void);
void Control_Loop(void);
void code2mode(uint8_t code,CollectMode *collectMode);
uint32_t getCurrentTimeTick(void);
#endif
