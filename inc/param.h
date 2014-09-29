
#ifndef PARAM_H
#define PARAM_H

#include "mcu.h"

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

typedef struct __gain{
	float p_gain;
	float i_gain;
	float d_gain;
	
	uint16_t p_add;
	uint16_t i_add;
	uint16_t d_add;
} Gain;

//AHRS gain
#define RP_P_ADD 12
#define RP_I_ADD 16
#define Y_P_ADD 20
#define Y_I_ADD 24

typedef struct{
	uint16_t input[8];
	uint16_t output[8];
	uint16_t trim[8];
} Radio;

typedef struct{
	uint16_t frame_count;		// counts created I2C frames
	int16_t pixel_flow_x_sum;	// accumulated x flow in pixels*10 since last I2C frame
	int16_t pixel_flow_y_sum;	// accumulated y flow in pixels*10 since last I2C frame
	int16_t flow_comp_m_x;		// x velocity*1000 in meters / timestep
	int16_t flow_comp_m_y;		// y velocity*1000 in meters / timestep
	int16_t qual;				// Optical flow quality / confidence 0: bad, 255: maximum quality
	int16_t gyro_x_rate; 		//gyro x rate
	int16_t gyro_y_rate; 		//gyro y rate
	int16_t gyro_z_rate; 		//gyro z rate
	uint8_t gyro_range; 		// gyro range
	uint8_t sonar_timestamp;	// timestep in milliseconds between I2C frames
	int16_t ground_distance;	// Ground distance in meters*1000. Positive value: distance known. Negative value: Unknown distance
	
	uint16_t qual_th; 			// Valid threshold optical flow quality
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

#endif
