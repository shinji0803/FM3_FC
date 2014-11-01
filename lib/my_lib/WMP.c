
#include "WMP.h"

#include <string.h>

#include "i2c.h"
#include "timer.h"

static uint8_t _wmp_data[32];

void WMP_init(void)
{
	/* Wii Motion Plus Initialize. */
	int32_t size;
	int32_t ret;
	
	wait(200);
	
	i2c->Cfg.SlaveAddr = WMP_CONF_ADD; 
	
	_wmp_data[0] = 0xFE;
	_wmp_data[1] = 0x04;
	size = 2;
	ret = i2c->DataTx(_wmp_data, &size);
	
	if(ret != SUCCESS){
		printf("WiiMotion+ Initialize Failed\r\n");
		while(1);
	}
	printf("WiiMotion+ Initialized\r\n");
	
	wait(200);
}

void WMP_update(void)
{
	int32_t size;
	uint8_t data_tx = 0x00;
	
	i2c->Cfg.SlaveAddr = WMP_ADD; 	
	size = 1;
	i2c->DataTx(&data_tx, &size);
	size= 6;
	i2c->DataRx(_wmp_data, &size);
}

void WMP_get_raw_gyro(Vector3f *g)
{
	WMP_update();
	
	g->x = (int16_t)((_wmp_data[4] >> 2) << 8) + _wmp_data[1];
	g->y = (int16_t)((_wmp_data[5] >> 2) << 8) + _wmp_data[2];
	g->z = (int16_t)((_wmp_data[3] >> 2) << 8) + _wmp_data[0];

}

void WMP_get_gyro_mode(uint8_t *mode)
{
	*mode = 0;
	*mode |= ((_wmp_data[4] >> 1) & 0x01) << 2; 	// Roll slow mode
	*mode |= (_wmp_data[3] & 0x01) << 1; 			// Pitch slow mode
	*mode |= ((_wmp_data[3] >> 1) & 0x01); 		// Yaw slow mode
}
