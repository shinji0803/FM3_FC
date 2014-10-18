
#include "WMP.h"

#include <string.h>

#include "i2c.h"

static uint8_t wmp_data[32];

void WMP_init(void)
{
	/* Wii Motion Plus Initialize. */
	int32_t size;
	
	i2c->Cfg.SlaveAddr = WMP_CONF_ADD; 

	wmp_data[0] = 0xFE;
	wmp_data[1] = 0x04;
	size = 2;
	i2c->DataTx(wmp_data, &size);
	
	printf("WiiMotion+ Initialized\r\n");
}

void WMP_update(void)
{
	int32_t size;
	uint8_t data_tx = 0x00;
	
	i2c->Cfg.SlaveAddr = WMP_ADD; 	
	size = 1;
	i2c->DataTx(&data_tx, &size);
	size= 6;
	i2c->DataRx(wmp_data, &size);
}

void WMP_get_raw_gyro(Vector3f *g)
{
	WMP_update();
	
	g->x = (int16_t)((wmp_data[4] >> 2) << 8) + wmp_data[1];
	g->y = (int16_t)((wmp_data[5] >> 2) << 8) + wmp_data[2];
	g->z = (int16_t)((wmp_data[3] >> 2) << 8) + wmp_data[0];

}

void WMP_get_gyro_mode(uint8_t *mode)
{
	*mode = 0;
	*mode |= ((wmp_data[4] >> 1) & 0x01) << 2; // Roll slow mode
	*mode |= (wmp_data[3] & 0x01) << 1; 		// Pitch slow mode
	*mode |= ((wmp_data[3] >> 1) & 0x01); 		// Yaw slow mode
}
