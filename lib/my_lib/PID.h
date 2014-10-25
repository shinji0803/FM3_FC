
#ifndef PID_H
#define PID_H

/* PID Controller library */

#include "hw_config.h"

typedef struct _pid_info
{
	float kp;
	float ki;
	float kd;
	float i_max;
	
	float integrator;
	float old_error;
	
} PID_Info;

/* PID Struct Initialize */
void PID_Init(PID_Info *pid, float kp, float ki, float kd, float i_max);

/* Set Gain to PID Struct */
void PID_set_gain(PID_Info *pid, float kp, float ki, float kd, float i_max);

/* Calculate PI-D Controller Output */ 
float PID_CalcD(PID_Info *pid, float error, float dt, float rate);

/* Calculate PID Controller Output */
float PID_Calc(PID_Info *pid, float error, float dt);
	
#endif