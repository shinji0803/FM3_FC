
#ifndef AHRS_H
#define AHRS_H

/* Attitude Heading Reference System */

#include <Math.h>
#include "hw_config.h"

#include "myMath.h"
#include "param.h"
#include "timer.h"
#include "i2c.h"
#include "fram.h"

#include "PX4FLOW.h"
#include "LSM303DLH.h"
#include "WMP.h"

#define GRAVITY 9.767f

#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

#define ToRad(x) (x * DEG_TO_RAD)
#define ToDeg(x) (x * RAD_TO_DEG)

extern volatile timeFlg time;

void AHRS_Init(void);

void AHRS_read_imu(void); // read all sensor(Gyro, Acc, Mag)

// DCM algorithm
void AHRS_dcm_update(float dt); // update DCM
void AHRS_dcm_normalize(void); // normalize DCM
void AHRS_drift_correction(void); // gyro drift correction
// get AHRS gain
void AHRS_get_gain(Gain *RandP, Gain *Y);
// set AHRS gain
void AHRS_set_gain(Gain *RandP, Gain *Y);
// get AHRS gain from FRAM
void AHRS_load_gain(void);

// Gyro function
void AHRS_get_gyro(Vector3f *g); // get scaled gyro value
void AHRS_get_raw_gyro(Vector3f *g); // get raw gyro value
void AHRS_get_omega(Vector3f *g); // Drift corrected gyro value

// Acc function
void AHRS_get_acc(Vector3f *a); // get scaled acc value
void AHRS_get_raw_acc(Vector3f *a); // get raw acc value

// Mag function
void AHRS_get_mag(Vector3f *m); // get scaled mag value
void AHRS_get_raw_mag(Vector3f *m); // get raw mag value

void AHRS_get_euler(Vector3f *att); // get euler angle from DCM

float AHRS_heading(Vector3f from); // get heading(Yaw) that corrected leaning

// Default DCM parameters
#define Kp_ROLLPITCH 1.4f
#define Ki_ROLLPITCH 0.0001f
#define Kp_YAW 3.0f
#define Ki_YAW 0.001f

/* Calibration Data */
/* Mag calibration data */
static const Vector3f m_max = { 400.0f, 400.0f, 400.0f}; 
static const Vector3f m_min = { -400.0f, -400.0f, -400.0f};

/* Acc calibration data */
static const Vector3f a_max = { 16800.0f, 15500.0f, 16000.0f};
static const Vector3f a_min = { -16400.0f, -16800.0f, -17050.0f};

/* Gyro calibration data */
/* Gyro valid range at rest when use WiiMotionPlus */
#define GYRO_CALIB_LOW	7800
#define GYRO_CALIB_HIGH 8400


/* Sensor scale data */
//#define GYRO_SCALE 13.768f // 13.768 unit/(deg/sec) when use WiiMotionPlus
// default value is 13.768, but it's too small. 
// Reference: http://hikakeya3.blog68.fc2.com/blog-category-14.html
#define GYRO_SCALE 19.9f
//#define GYRO_SCALE 20.0f

#define ACCEL_X_OFFSET ((a_min.x + a_max.x) / 2.0f)
#define ACCEL_Y_OFFSET ((a_min.y + a_max.y) / 2.0f)
#define ACCEL_Z_OFFSET ((a_min.z + a_max.z) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (a_max.x - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (a_max.y - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (a_max.z - ACCEL_Z_OFFSET))


#endif