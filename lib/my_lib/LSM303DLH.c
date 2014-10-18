
#include "LSM303DLH.h"

#include <math.h>

#include "hw_config.h"
#include "i2c.h"
#include "parameters.h"
#include "myMath.h"


// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address, 
// and sets the last bit correctly based on reads and writes
#define ACC_ADDRESS (0x30 >> 1)
#define MAG_ADDRESS (0x3C >> 1)

// Public Methods //////////////////////////////////////////////////////////////

// Turns on the LSM303DLH's accelerometer and magnetometers and places them in normal
// mode.
void LSM303DLH_Init(void)
{	
	// Enable Accelerometer
	// 0x27 = 0b00100111
	// Normal power mode, all axes enabled
	//writeAccReg(LSM303DLH_CTRL_REG1_A, 0x27);
	writeAccReg(LSM303DLH_CTRL_REG1_A, 0x37);
	
	// Enable Magnetometer
	// 0x00 = 0b00000000
	// Continuous conversion mode
	writeMagReg(LSM303DLH_MR_REG_M, 0x00);
	// Data output rate = 75Hz
	writeMagReg(LSM303DLH_CRA_REG_M, 0x18);
}

// Writes an accelerometer register
void writeAccReg(uint8_t reg, uint8_t value)
{
	int32_t size;
	uint8_t data[2] = { reg, value};
	
	i2c->Cfg.SlaveAddr = ACC_ADDRESS;
	size = 2;
	i2c->DataTx(&data, &size);
}

// Reads an accelerometer register
uint8_t readAccReg(uint8_t reg)
{
	int32_t size;
	uint8_t data = reg;
	
	i2c->Cfg.SlaveAddr = ACC_ADDRESS;
	size = 1;
	i2c->DataTx(&data, &size);
	
	size = 1;
	i2c->DataRx(&data, &size);
	
	return data;
}

// Writes a magnetometer register
void writeMagReg(uint8_t reg, uint8_t value)
{
	int32_t size;
	uint8_t data[2] = { reg, value};
	
	i2c->Cfg.SlaveAddr = MAG_ADDRESS;
	size = 2;
	i2c->DataTx(&data, &size);
}

// Reads a magnetometer register
uint8_t readMagReg(uint8_t reg)
{
	int32_t size;
	uint8_t data = reg;
	
	i2c->Cfg.SlaveAddr = MAG_ADDRESS;
	size = 1;
	i2c->DataTx(&data, &size);
	
	size = 1;
	i2c->DataRx(&data, &size);
	
	return data;
}

// Reads the 3 accelerometer channels and stores them in vector a
void readAcc(Vector3f *a)
{
	int32_t size;
	uint8_t data[8];
	
	i2c->Cfg.SlaveAddr = ACC_ADDRESS;
	// assert the MSB of the address to get the accelerometer 
	// to do slave-transmit subaddress updating.
	size = 1;
	data[0] = LSM303DLH_OUT_X_L_A | (1 << 7);
	i2c->DataTx( &data, &size); 
	
	size = 6;
	i2c->DataRx( &data, &size); 
	
	uint8_t xha = data[1];
	uint8_t xla = data[0];
	uint8_t yha = data[3];
	uint8_t yla = data[2];
	uint8_t zha = data[5];
	uint8_t zla = data[4];
	
	a->x = (int16_t)((xha << 8) | xla);
	a->y = (int16_t)((yha << 8) | yla);
	a->z = (int16_t)((zha << 8) | zla);
}

// Reads the 3 magnetometer channels and stores them in vector m
void readMag(Vector3f *m)
{
	int32_t size;
	uint8_t data[8];
	
	i2c->Cfg.SlaveAddr = MAG_ADDRESS;
	size = 1;
	data[0] = LSM303DLH_OUT_X_H_M;
	i2c->DataTx( &data, &size); 
	
	size = 6;
	i2c->DataRx( &data, &size); 


	uint8_t xhm = data[0];
	uint8_t xlm = data[1];
	uint8_t yhm = data[2];
	uint8_t ylm = data[3];
	uint8_t zhm = data[4];
	uint8_t zlm = data[5];

	m->x = (int16_t)(xhm << 8 | xlm);
	m->y = (int16_t)(yhm << 8 | ylm);
	m->z = (int16_t)(zhm << 8 | zlm);
}

// Reads all 6 channels of the LSM303DLH and stores them in the object variables
void LSM303DLH_read(Vector3f *a, Vector3f *m)
{
	readAcc(a);
	readMag(m);
}


void mag_calibrate_out(Vector3f *min, Vector3f *max)
{
	Vector3f temp_m;
	readMag(&temp_m);
	
	if(temp_m.x > max->x) max->x = temp_m.x;
	if(temp_m.x < min->x) min->x = temp_m.x;
	if(temp_m.y > max->y) max->y = temp_m.y;
	if(temp_m.y < min->y) min->y = temp_m.y;
	if(temp_m.z > max->z) max->z = temp_m.z;
	if(temp_m.z < min->z) min->z = temp_m.z;
}



