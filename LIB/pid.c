#include "pid.h"
#include "stm32f4xx.h"
#include "math.h"
#include "controlTask.h"

static float pitch_output = 0;
static float yaw_output = 0;
Smart_PID_t sPID;

void PID_Reset(PID_Regulator_t *pid)
{
	pid->ref = 0;
	pid->fdb = 0;
	pid->output = 0;
}
void PID_Calc(PID_Regulator_t *pid)
{
	pid->err[1] = pid->err[0];
	pid->err[0] = (pid->ref - pid->fdb);

	//calculate PID
	pid->KpComponent = pid->kp * pid->err[0];
	pid->KiComponent += pid->ki * pid->err[0];
	pid->KdComponent = pid->kd * (pid->err[0] - pid->err[1]);
	pid->output = pid->KpComponent + pid->KiComponent+ pid->KdComponent;
	
	//output value limit
	if(fabs(pid->output) > pid->output_limit)
		(pid->output>0) ? (pid->output=pid->output_limit) : (pid->output = -pid->output_limit);
}

void PID_Calc_Debug(PID_Regulator_t *pid,float kp,float ki,float kd)
{
	pid->err[1] = pid->err[0];
	pid->err[0] = (pid->ref - pid->fdb);

	//calculate PID
	pid->KpComponent = kp * pid->err[0];
	
	//if(pid->last_output > 750.0f/70.f && pid->last_output < 1500.0f/70.0f)
	//if(fabs(pid->ref-pid->fdb)<5)
	pid->KiComponent += ki * pid->err[0];
	//if(fabs(pid->KpComponent) < 1) 
		
	pid->KdComponent = kd * (pid->err[0] - pid->err[1]);
	pid->output = pid->KpComponent + pid->KiComponent+ pid->KdComponent;
	
	//if(fabs(pid->KiComponent)>16.5)
		//(pid->KiComponent>0) ? (pid->KiComponent=16.5) : (pid->KiComponent=-16.5);
	//output value limit
	if(fabs(pid->output) > pid->output_limit)
		(pid->output>0) ? (pid->output=pid->output_limit) : (pid->output = -pid->output_limit);
	pid->last_output = pid->output;
}
void PID_Calc_Step(PID_Regulator_t *pid)
{
	pid->err[1] = pid->err[0];
	pid->err[0] = (pid->ref - pid->fdb);

		
	pid->KpComponent = pid->kp * pid->err[0];
	pid->KiComponent += pid->ki * pid->err[0];
	pid->KdComponent = pid->kd * (pid->err[0] - pid->err[1]);
	pid->output = pid->KpComponent + pid->KiComponent+ pid->KdComponent;
	if(fabs(pid->output) > pid->output_limit)
		(pid->output>0) ? (pid->output=pid->output_limit) : (pid->output = -pid->output_limit);
	
	//restrict maximum step
	if(pid->output - pid->last_output > 800)
		pid->output = pid->last_output + 800;
	else if(pid->output - pid->last_output < -800)
		pid->output = pid->last_output - 800;
	
	pid->last_output = pid->output;
}

void PID_Calc_Windup(PID_Regulator_t *pid)
{
	
	pid->err[1] = pid->err[0];
	pid->err[0] = (pid->ref - pid->fdb);
	
	pid->KpComponent = pid->kp * pid->err[0];
	pid->KdComponent = pid->kd * (pid->err[0]-pid->err[1]);

//Integral wind up
	/*
	if(fabs(pid->ref-pid->fdb) < pid->windup_limit)
	{
			pid->KiComponent += pid->ki*pid->err[0];
	}
	*/
	pid->KiComponent += pid->ki*pid->err[0];
	if(fabs(pid->KiComponent) > pid->Ki_Limit)
		(pid->KiComponent>0) ? (pid->KiComponent = pid->Ki_Limit) : (pid->KiComponent = -pid->Ki_Limit);
		

	pid->output = pid->KpComponent + pid->KiComponent + pid->KdComponent;
	if(fabs(pid->output) > pid->output_limit)
		(pid->output>0) ? (pid->output=pid->output_limit) : (pid->output = -pid->output_limit);
	
	pid->last_output = pid->output;
}


void PID_Calc_GM(PID_Regulator_t *pid)
{
	pid->kp = 50;
	pid->ki = 0;
	pid->kd = 0;
	
	pid->err[1] = pid->err[0];
	pid->err[0] = (pid->ref - pid->fdb);
	
	pid->KpComponent = pid->kp * pid->err[0];
	pid->KdComponent = pid->kd * (pid->err[0]-pid->err[1]);

//Integral wind up	
	if(pid->last_output < -4800)
	{
		if(pid->err[0]>0)
			pid->KiComponent += pid->ki*pid->err[0];
	}
	else if(pid->last_output > 4800)
	{
		if(pid->err[0]<0)
			pid->KiComponent += pid->ki*pid->err[0];
	}
	else
		pid->KiComponent += pid->ki*pid->err[0];
		

	pid->output = pid->KpComponent + pid->KiComponent + pid->KdComponent;
	
//specify the range of output
	if(pid->output > 5000)
		pid->output = 5000;
	else if(pid->output < -5000)
		pid->output = -5000;
	
	pid->last_output = pid->output;
	
}

void PID_Calc_GM_PP(PID_Regulator_t *pid)
{
	pid->kp = 2;//2
	pid->ki = 0.01;//0.03
	pid->kd = 0;
	
	pid->err[1] = pid->err[0];
	pid->err[0] = (pid->ref - pid->fdb);
	
	pid->KpComponent = pid->kp * pid->err[0];
	pid->KdComponent = pid->kd * (pid->err[0]-pid->err[1]);

//Integral wind up	
	if(pitch_output < -3000)
	{
		if(pid->err[0]>0)
			pid->KiComponent += pid->ki*pid->err[0];
	}
	else if(pitch_output > 3000)
	{
		if(pid->err[0]<0)
			pid->KiComponent += pid->ki*pid->err[0];
	}
	else
		pid->KiComponent += pid->ki*pid->err[0];
		

	pid->output = pid->KpComponent + pid->KiComponent + pid->KdComponent;
	
//specify the range of output
	if(pid->output > 5000)
		pid->output = 5000;
	else if(pid->output < -5000)
		pid->output = -5000;
	
	pid->last_output = pid->output;
	
}

void PID_Calc_GM_PS(PID_Regulator_t *pid)
{
	pid->kp = 70;//50
	pid->ki = 0;
	pid->kd = 0;
	
	pid->err[1] = pid->err[0];
	pid->err[0] = (pid->ref - pid->fdb);
	
	pid->KpComponent = pid->kp * pid->err[0];
	pid->KdComponent = pid->kd * (pid->err[0]-pid->err[1]);

//Integral wind up	
	if(pid->last_output < -4800)
	{
		if(pid->err[0]>0)
			pid->KiComponent += pid->ki*pid->err[0];
	}
	else if(pid->last_output > 4800)
	{
		if(pid->err[0]<0)
			pid->KiComponent += pid->ki*pid->err[0];
	}
	else
		pid->KiComponent += pid->ki*pid->err[0];
		

	pid->output = pid->KpComponent + pid->KiComponent + pid->KdComponent;
	
//specify the range of output
	if(pid->output > 5000)
		pid->output = 5000;
	else if(pid->output < -5000)
		pid->output = -5000;
	
	pid->last_output = pid->output;
	pitch_output = pid->output;
}


void PID_Calc_GM_YP(PID_Regulator_t *pid)
{
	pid->kp = 1;
	pid->ki = 0;
	pid->kd = 0; //PID for YAW position 
	
	pid->err[1] = pid->err[0];
	pid->err[0] = (pid->ref - pid->fdb);
	
	pid->KpComponent = pid->kp * pid->err[0];
	pid->KdComponent = pid->kd * (pid->err[0]-pid->err[1]);

//Integral wind up	
	if(yaw_output < -3000)
	{
		if(pid->err[0]>0)
			pid->KiComponent += pid->ki*pid->err[0];
	}
	else if(yaw_output > 3000)
	{
		if(pid->err[0]<0)
			pid->KiComponent += pid->ki*pid->err[0];
	}
	else
		pid->KiComponent += pid->ki*pid->err[0];
		

	pid->output = pid->KpComponent + pid->KiComponent + pid->KdComponent;
	
//specify the range of output
	if(pid->output > 5000)
		pid->output = 5000;
	else if(pid->output < -5000)
		pid->output = -5000;
	
	pid->last_output = pid->output;
	
}


void PID_Calc_GM_YS(PID_Regulator_t *pid)
{
	pid->kp = 100;
	pid->ki = 0;
	pid->kd = 0; //PID for YAW speed
	pid->err[1] = pid->err[0];
	pid->err[0] = (pid->ref - pid->fdb);
	
	pid->KpComponent = pid->kp * pid->err[0];
	pid->KdComponent = pid->kd * (pid->err[0]-pid->err[1]);

//Integral wind up	
	if(pid->last_output < -3000)
	{
		if(pid->err[0]>0)
			pid->KiComponent += pid->ki*pid->err[0];
	}
	else if(pid->last_output > 3000)
	{
		if(pid->err[0]<0)
			pid->KiComponent += pid->ki*pid->err[0];
	}
	else
		pid->KiComponent += pid->ki*pid->err[0];
		

	pid->output = pid->KpComponent + pid->KiComponent + pid->KdComponent;
	
//specify the range of output
	if(pid->output > 5000)
		pid->output = 5000;
	else if(pid->output < -5000)
		pid->output = -5000;
	
	pid->last_output = pid->output;
	yaw_output = pid->output;
}
static uint8_t smart = 0;
/*
void PID_Smart(PID_Regulator_t *pid,float impulse)
{
	if(getCurrentTimeTick() - sPID.smart_timer > 10) // sample every 10ms
	{
		// update smart_buffer
		sPID.smart_timer = getCurrentTimeTick();
		sPID.smart_counter %= 5; // determine by buffer_size
		sPID.smart_buffer[sPID.smart_counter++] = pid->err[0];
		
		// sort out maximum and minimum error
		sPID.smart_max = 0;
		sPID.smart_min = 360;
		for(int i=0;i<5;i++)
		{
			if(sPID.smart_max < fabs(sPID.smart_buffer[i])) sPID.smart_max = sPID.smart_buffer[i];
			if(sPID.smart_min > fabs(sPID.smart_buffer[i])) sPID.smart_min = sPID.smart_buffer[i];
		}
	}
	// if error rate stays at around zero and error persists, increase output to cope with non-linear interval
	if((fabs(sPID.smart_max-sPID.smart_min)<1.0f && fabs(pid->ref-pid->fdb)>5.0f)) //minimum change rate and converge interval defined here
	{
		if(pid->ref-pid->fdb>0) pid->output = impulse;
		else pid->output = -impulse; // additional help (for GMYPositionPID)
		smart = 1;
	}
	else
	{
		smart = 0; // do nothing
	}
}
*/
void PID_Calc_Arm(PID_Regulator_t *PID_Regulator)
{
	PID_Regulator->err[1] = PID_Regulator->err[0];
	PID_Regulator->err[0] = (PID_Regulator->ref - PID_Regulator->fdb);

		
	PID_Regulator->KpComponent = PID_Regulator->kp * PID_Regulator->err[0];
	PID_Regulator->KiComponent += PID_Regulator->ki *PID_Regulator->err[0];
	PID_Regulator->KdComponent = PID_Regulator->kd * (PID_Regulator->err[0] - PID_Regulator->err[1]);
	PID_Regulator->output = PID_Regulator->KpComponent + PID_Regulator->KiComponent+PID_Regulator->KdComponent;
	
	if(PID_Regulator->output - PID_Regulator->last_output > 8000)
		PID_Regulator->output = PID_Regulator->last_output + 8000;
	else if(PID_Regulator->output - PID_Regulator->last_output < -8000)
		PID_Regulator->output = PID_Regulator->last_output - 8000;
	
	PID_Regulator->last_output = PID_Regulator->output;
}
