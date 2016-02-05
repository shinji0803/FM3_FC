
#include "MPU9250.h"

#include <string.h>

#include "i2c.h"
#include "timer.h"

static uint8_t _data[16];

void MPU9250_init(void)
{
	/* MPU9250 Initialize. */
	int32_t size;
	int32_t ret;
	uint8_t temp[4];
	
	wait(100);
	
	i2c->Cfg.SlaveAddr = MPU9250_ADDRESS; 
	
	if(MPU9250_Who() == 0x71) printf("MPU9250 Communication OK.\r\n");
	else{
		printf("MPU9250 Communication Failed.")
		while(1);
	}
	
	//Gyro Config
	temp[0] = GYRO_CONFIG;
	temp[1] = 0x18; //Full Scale = 2000 DPS
	MPU9250_Write(temp, 2);
	
	//Accel Config
	temp[0] = ACCEL_CONFIG;
	temp[1] = 0x10; //Full Scale = 8 G
	MPU9250_Write(temp, 2);
	
	//Power Management Config 
	temp[0] = PWR_MGMT_1;
	temp[1] = 0x00; //
	MPU9250_Write(temp, 2);
	
	//Power Management Config 
	temp[0] = INT_PIN_CFG;
	temp[1] = 0x02; //
	MPU9250_Write(temp, 2);	
	
	
	if(ret != SUCCESS){
		printf("WiiMotion+ Initialize Failed\r\n");
		while(1);
	}
	printf("WiiMotion+ Initialized\r\n");
	
	wait(200);
}

static uint8_t MPU9250_Who(void)
{
	int32_t size;
	uint8_t temp[4];
	
	temp[0] = WHO_AM_I_MPU9250;
	MPU9250_Write(temp, 1);
	MPU9250_Read(temp,1);
	
	return temp[0];
}

static int32_t MPU9250_Write(uint8_t *data, int32_t size)
{
	int32_t tx_size = size;
	
	i2c->Cfg.SlaveAddr = MPU9250_ADDRESS;
	i2c->DataTx(data, &tx_size);
	
	if(tx_size == size) return 1;
	else return -1;
}

static int32_t MPU9250_Read(uint8_t *data, int32_t size)
{
	int32_t rx_size = size;
	
	i2c->Cfg.SlaveAddr = MPU9250_ADDRESS;
	i2c->DataRx(data, &rx_size);
	
	if(rx_size == size) return 1;
	else return -1;
}
	