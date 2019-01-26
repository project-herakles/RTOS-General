#ifndef _PID_H_
#define _PID_H_
#include "stm32f4xx.h"

typedef struct PID_Regulator_t
{
	float ref;
	float fdb;
	float err[2];
	float output;
	float last_output;
	float kp;
	float ki;
	float kd;
	float KpComponent;
	float KiComponent;
	float KdComponent;
	float output_limit;
	float windup_limit;
	float max_step;
	void (*Calc)(struct PID_Regulator_t*);
	void (*Reset)(struct PID_Regulator_t*);
	float Ki_Limit;
}PID_Regulator_t;

typedef struct
{
	float smart_buffer[5];
	float smart_max;
  float smart_min;
  uint32_t smart_timer;
  uint8_t smart_counter;
}Smart_PID_t;

void PID_Calc(PID_Regulator_t *pid);
void PID_Calc_Debug(PID_Regulator_t *pid,float kp,float ki,float kd);
void PID_Calc_Windup(PID_Regulator_t *pid);
void PID_Calc_Step(PID_Regulator_t *pid);
void PID_Calc_Arm(PID_Regulator_t *pid);
void PID_Calc_GM(PID_Regulator_t *pid);
void PID_Calc_GM_PS(PID_Regulator_t *pid);
void PID_Calc_GM_PP(PID_Regulator_t *pid);
void PID_Calc_GM_YP(PID_Regulator_t *pid);
void PID_Calc_GM_YS(PID_Regulator_t *pid);
void PID_Reset(PID_Regulator_t *pid);
void PID_Smart(PID_Regulator_t *pid, float impluse);

#define YAW_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	0.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	1000,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}\

#define YAW_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	0.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	1000,\
	0,\
	&PID_Calc_Windup,\
	&PID_Reset,\
}\

#define PITCH_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	0.5f,\
	0.002f,\
	0.0f,\
	0,\
	9,\
	0,\
	5000,\
	5,\
	0,\
	&PID_Calc_Windup,\
	&PID_Reset,\
	20,\
}\

#define PITCH_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	70.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	1000,\
	0,\
	&PID_Calc_Windup,\
	&PID_Reset,\
}\

#define YAW_ENCODER_DEFAULT \
{\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	6369,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
}\

#define ARM_ENCODER_DEFAULT \
{\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	4385,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
}\


#define PITCH_ENCODER_DEFAULT \
{\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	1645,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
}\

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	80.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	0,\
	800,\
	&PID_Calc_Step,\
	&PID_Reset,\
}\

#define ROTATE_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	0.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	0,\
	800,\
	&PID_Calc,\
	&PID_Reset,\
}\


#define SHOOT_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	0.5f,\
	0.0f,\
	0.0f,\
	0,\
	9,\
	0,\
	5000,\
	12,\
	0,\
	&PID_Calc_Windup,\
	&PID_Reset,\
}\

#define SHOOT_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	0.5f,\
	0.0f,\
	0.0f,\
	0,\
	9,\
	0,\
	5000,\
	5,\
	0,\
	&PID_Calc_Windup,\
	&PID_Reset,\
}\

#define SHOOT_ENCODER_DEFAULT \
{\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	7123,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
	0,\
}\

#define ARM_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	70.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	1000,\
	0,\
	&PID_Calc_Arm,\
	&PID_Reset,\
}\

#define ARM_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	0,\
	0,\
	70.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	5000,\
	1000,\
	0,\
	&PID_Calc_Arm,\
	&PID_Reset,\
}\

#endif
