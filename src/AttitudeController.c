
#include "AttitudeController.h"

#include "rc_io.h"
#include "parameters.h"
#include "PID.h"

typedef struct _attitude_controller
{
	/* RC Input Data */
	RC_CONFIG rc;
	
	/* SetPoint */
	float setpoint;
	float rate_setpoint;
	
	/* PID Controller Output */
	float servo_out;
	
	PID_Info att;
	PID_Info rate;
} AttitudeController;


AttitudeController _roll;
AttitudeController _pitch;
AttitudeController _yaw;


/* Controller Initialize and Load Controller Gain */
void AttitudeController_Init(void)
{
	/* PID Controller Initialize */
	PID_Init(&(_roll.att), 0, 0, 0, 0);
	PID_Init(&(_roll.rate), 0, 0, 0, 0);
	
	PID_Init(&(_pitch.att), 0, 0, 0, 0);
	PID_Init(&(_pitch.rate), 0, 0, 0, 0);
	
	PID_Init(&(_yaw.att), 0, 0, 0, 0);
	PID_Init(&(_yaw.rate), 0, 0, 0, 0);
	
	/* Load Conttoller Gain */
	_roll.att.kp 		= storage_get_param(STAB_ROLL_P);
	_roll.att.ki 		= storage_get_param(STAB_ROLL_I);
	_roll.att.kd 		= storage_get_param(STAB_ROLL_D);
	_roll.att.i_max		= storage_get_param(STAB_ROLL_IMAX);
	
	_pitch.att.kp 		= storage_get_param(STAB_PITCH_P);
	_pitch.att.ki 		= storage_get_param(STAB_PITCH_I);
	_pitch.att.kd 		= storage_get_param(STAB_PITCH_D);
	_pitch.att.i_max	= storage_get_param(STAB_PITCH_IMAX);
	
	_yaw.att.kp 		= storage_get_param(STAB_YAW_P);
	_yaw.att.ki 		= storage_get_param(STAB_YAW_I);
	_yaw.att.kd 		= storage_get_param(STAB_YAW_D);
	_yaw.att.i_max		= storage_get_param(STAB_YAW_IMAX);
	
	/* Set RC Config Data */
	_roll.rc.rc_max		= 1940;
	_roll.rc.rc_min		= 1100;
	_roll.rc.rc_trim	= 1520;
	_roll.rc.rc_dz		= 0;
	
	_pitch.rc.rc_max	= 1940;
	_pitch.rc.rc_min	= 1100;
	_pitch.rc.rc_trim	= 1520;
	_pitch.rc.rc_dz		= 0;
	
	_yaw.rc.rc_max		= 1940;
	_yaw.rc.rc_min		= 1100;
	_yaw.rc.rc_trim		= 1520;
	_yaw.rc.rc_dz		= 0;
	
}

void AttitudeController_calcSetpoint(void)
{
	
}


