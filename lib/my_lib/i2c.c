
#include "i2c.h"

#include <stdio.h>

#include "hw_config.h"

extern I2cDev_IOB I2cDev[];
I2cDev_IOB *i2c;

static void init_i2c_port(void);

void Init_i2c(){
	int32_t ret = SUCCESS;
	
	init_i2c_port();
	
	i2c = &I2cDev[3];
	i2c->Cfg.BaudRate = 300000; 
	i2c->Cfg.Mode = I2cDev_MODE_MASTER;
	
	ret = i2c->Init();
	if(ret == SUCCESS) printf("I2C Initialized\r\n");
}

static void init_i2c_port(){
	FM3_GPIO->ADE = 0x00000000;
	FM3_GPIO->PFR7 = FM3_GPIO->PFR7 | 0xC0;
	FM3_GPIO->EPFR07 = FM3_GPIO->EPFR07 | 0x05000000;
	
	//Pullup resister setting
	//FM3_GPIO->PCR7 = FM3_GPIO->PCR7 | 0xC0;
}
