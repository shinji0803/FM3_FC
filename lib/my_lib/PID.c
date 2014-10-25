
#include "PID.h"

void PID_Init(PID_Info *pid, float kp, float ki, float kd, float i_max)
{
	/* Load PID Parameters */
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->i_max = i_max;
	
	pid->integrator = 0;
	pid->old_error = 0;
}

void PID_set_gain(PID_Info *pid, float kp, float ki, float kd, float i_max)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->i_max = i_max;
}

float PID_calcD(PID_Info *pid, float error, float dt, float rate)
{
	float output;
	
	/* Integrator calculation */
	pid->integrator += error * dt;
	
	/* If i_max is 0, integrator limit is disable */
	if(pid->i_max <= 0){
		if(pid->integrator > pid->i_max) pid->integrator = pid->i_max;
		if(pid->integrator < -pid->i_max) pid->integrator = -pid->i_max;
	}
	
	output = pid->kp * error + pid->ki * pid->integrator - pid->kd * rate;
	pid->old_error = error;
	
	return output;
}

float PID_calc(PID_Info *pid, float error, float dt)
{
	return PID_calcD(pid, error, dt, (error - pid->old_error) / dt);
}

