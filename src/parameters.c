
#include "parameters.h"
#include "fram.h"

const Gain_t AHRS_Gain[] = {
	{AHRS_ROLLPITCH_P, 	1.4f,		0	},
	{AHRS_ROLLPITCH_I, 	0.0001f,	4	},
	{AHRS_YAW_P,		3.0f,		8	},
	{AHRS_YAW_I, 		0.001f,		12	}
};

const Gain_t Att_Gain[] = {
	/* Stabilize Roll PID */
	{STAB_ROLL_P,		0.0f,	16	},
	{STAB_ROLL_I,		0.0f,	20	},
	{STAB_ROLL_D,		0.0f,	24	},
	{STAB_ROLL_IMAX,	0.0f,	28	},
	
	/* Stabilize Pitch PID */
	{STAB_PITCH_P, 		0.0f,	32	},
	{STAB_PITCH_I, 		0.0f,	36	},
	{STAB_PITCH_D, 		0.0f,	40	},
	{STAB_PITCH_IMAX,	0.0f,	44	},
	
	/* Stabilize Yaw PID */
	{STAB_YAW_P,		0.0f,	48	},
	{STAB_YAW_I,		0.0f,	52	},
	{STAB_YAW_D,		0.0f,	56	},
	{STAB_YAW_IMAX,		0.0f,	60	},
	
	/* Rate Roll PID */
	{RATE_ROLL_P, 		0.0f,	64	},
	{RATE_ROLL_I,		0.0f,	68	},
	{RATE_ROLL_D, 		0.0f,	72	},
	{RATE_ROLL_IMAX,	0.0f,	76	},
	
	/* Rate Pitch PID */
	{RATE_PITCH_P,	 	0.0f,	80	},
	{RATE_PITCH_I,		0.0f,	84	},
	{RATE_PITCH_D, 		0.0f,	88	},
	{RATE_PITCH_IMAX,	0.0f,	92	},
	
	/* Rate Yaw PID */
	{RATE_YAW_P, 		0.0f,	96	},
	{RATE_YAW_I,		0.0f,	100	},
	{RATE_YAW_D,	 	0.0f,	104	},
	{RATE_YAW_IMAX,		0.0f,	108	}
};


float storage_get_param(ParamIndex_t index)
{
	uint16_t temp_add;
	if(index < AHRS_GAIN_END) temp_add = AHRS_Gain[index].add;
	else temp_add = AHRS_Gain[index - 4 - 1].add;

	return read_float(temp_add);
}

void storage_set_param(ParamIndex_t index, float v)
{
	uint16_t temp_add;
	
	if(index < AHRS_GAIN_END) temp_add = AHRS_Gain[index].add;
	else temp_add = AHRS_Gain[index - 4 - 1].add;
	
	write_float(temp_add, v);
}
	
	
	
	



