
#ifndef PARAMETERS_H
#define PARAMETERS_H

/* Parameter management valiable and function */

#include "hw_config.h"

typedef union __generic_16bit{
	uint8_t b[2];
	int16_t i;
} generic_16bit;

typedef union __generic_32bit{
	uint8_t b[4];
	float f;
	int32_t i;
} generic_32bit;

typedef union __generic_64bit{
	uint8_t b[8];
	int64_t ll;
} generic_64bit;

/* Vector and Matrix Paraemters */
typedef struct __Vector3f{
	float x, y, z;
} Vector3f;

typedef struct __Vector3d{
	uint16_t x, y, z;
} Vector3d;

typedef struct __Matrix3f{
	Vector3f a;
	Vector3f b;
	Vector3f c;	
} Matrix3f;


/* Radio Config Parameters */
typedef struct{
	uint16_t input[8];
	uint16_t output[8];
	uint16_t trim[8];
} Radio;

typedef struct {
	uint16_t frame_count;
	int16_t pixel_flow_x_sum;
	int16_t pixel_flow_y_sum;
	int16_t flow_comp_m_x;
	int16_t flow_comp_m_y;
	int16_t qual;
	int16_t gyro_x_rate;
	int16_t gyro_y_rate;
	int16_t gyro_z_rate;
	uint8_t gyro_range;
	uint8_t sonar_timestamp;
	int16_t ground_distance;
	
	uint16_t qual_th;
	float x;
	float y;
} flow_data;

typedef enum{
	ROLL,
	PITCH,
	YAW,
	RATE_ROLL,
	RATE_PITCH,
	RATE_YAW
} axis_t;

/* Paraemters list */
typedef enum{
	AHRS_ROLLPITCH_P = 0,
	AHRS_ROLLPITCH_I,
	
	AHRS_YAW_P,
	AHRS_YAW_I,
	
	AHRS_GAIN_END,
	
	STAB_ROLL_P,
	STAB_ROLL_I,
	STAB_ROLL_D,
	
	STAB_PITCH_P,
	STAB_PITCH_I,
	STAB_PITCH_D,
	
	STAB_YAW_P,
	STAB_YAW_I,
	STAB_YAW_D,
	
	RATE_ROLL_P,
	RATE_ROLL_I,
	RATE_ROLL_D,
	
	RATE_PITCH_P,
	RATE_PITCH_I,
	RATE_PITCH_D,
	
	RATE_YAW_P,
	RATE_YAW_I,
	RATE_YAW_D,
	
	ATT_GAIN_END
} ParamIndex_t;


/* Gain Parameters */
typedef struct __gain{
	ParamIndex_t index;
	float value; // Default parameter
	uint16_t add; // FRAM address for save
} Gain_t;

/* Get parameter from FRAM */
float storage_get_param(ParamIndex_t index);
/* Set parameter to FRAM */
void storage_set_param(ParamIndex_t index, float v);

#endif



