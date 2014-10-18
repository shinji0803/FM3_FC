
#include "fram.h"

#include "hw_config.h"
#include "parameters.h"

extern I2cDev_IOB I2cDev[];
I2cDev_IOB *fram;

static void initPort(void);
static uint32_t calc_slave_addr(uint16_t address);
static void write_multibyte(uint16_t address, uint8_t byte_size, uint8_t *data);
static void read_multibyte(uint16_t address, uint8_t byte_size, uint8_t *data);

void Init_fram(){
	int32_t ret;
	
	initPort();
		
	fram = &I2cDev[1];
	fram->Cfg.SlaveAddr = 0x50;  //1010000b: 7bit slave address
	fram->Cfg.BaudRate = 400000; 
	fram->Cfg.Mode = I2cDev_MODE_MASTER;
	
	ret = fram->Init();
	if(ret == SUCCESS) printf("FRAM Initialized\r\n");
}


static void initPort(){
	FM3_GPIO->ADE = 0x00000000;
	FM3_GPIO->PFR5 = FM3_GPIO->PFR5 | 0x180;
	FM3_GPIO->EPFR07 = FM3_GPIO->EPFR07 | 0x05000;
}

//スレーブアドレス計算
static uint32_t calc_slave_addr(uint16_t address){
	uint16_t slave_addr;
	slave_addr = (address >> 8) & 0x07;
	return (uint32_t)slave_addr | 0x50; //1010000b + (アドレス上位3bit）
}

//多バイト書き込み
static void write_multibyte(uint16_t address, uint8_t byte_size, uint8_t *data){
	uint8_t *data_p; 
	uint8_t i = 0;
	uint8_t data_tx[10];
	int32_t size;
	
	data_p = data;
	
	fram->Cfg.SlaveAddr = calc_slave_addr(address);
	
	data_tx[0] = (uint8_t)(address & 0xFF); //アドレス下位8bit
	for( i = 0; i < byte_size; i++)	data_tx[i + 1] = *(data_p + i);
	
	size = (uint32_t)(byte_size + 1);
	fram->DataTx(data_tx, &size);
}

//////////////////////
// 各型の書き込み関数 //
//////////////////////
void write_byte(uint16_t address, uint8_t data){
	write_multibyte(address, 1, &data);
}

void write_int16(uint16_t address, int16_t data){
	uint8_t data_tx[2];
	
	data_tx[0] = (uint8_t)(data & 0xFF);
	data_tx[1] = (uint8_t)((data >>4) & 0xFF);
	write_multibyte(address, 2, data_tx);
}

void write_uint16(uint16_t address, uint16_t data){
	uint8_t data_tx[2];
	
	data_tx[0] = (uint8_t)(data & 0xFF);
	data_tx[1] = (uint8_t)((data >>8) & 0xFF);
	write_multibyte(address, 2, data_tx);
}

void write_int32(uint16_t address, int32_t data){
	uint8_t data_tx[4];
	
	data_tx[0] = (uint8_t)(data & 0xFF);
	data_tx[1] = (uint8_t)((data >> 8) & 0xFF);
	data_tx[2] = (uint8_t)((data >> 16) & 0xFF);
	data_tx[3] = (uint8_t)((data >> 24) & 0xFF);
	write_multibyte(address, 4, data_tx);
}

void write_uint32(uint16_t address, uint32_t data){
	uint8_t data_tx[4];
	
	data_tx[0] = (uint8_t)(data & 0xFF);
	data_tx[1] = (uint8_t)((data >> 8) & 0xFF);
	data_tx[2] = (uint8_t)((data >> 16) & 0xFF);
	data_tx[3] = (uint8_t)((data >> 24) & 0xFF);
	write_multibyte(address, 4, data_tx);
}

void write_float(uint16_t address, float data){
	uint8_t data_tx[4];
	generic_32bit r;
	r.f = data;
	
	data_tx[0] = r.b[0];
	data_tx[1] = r.b[1];
	data_tx[2] = r.b[2];
	data_tx[3] = r.b[3];
	write_multibyte(address, 4, data_tx);
}
/////////////////////////////
// 各型の書き込み関数　ここまで //
/////////////////////////////

//多バイト読込
static void read_multibyte(uint16_t address, uint8_t byte_size, uint8_t *data){
	uint8_t data_tx[2];
	uint8_t data_rx[10];
	uint8_t i;
	int32_t size;
	
	for(i = 0; i < byte_size ; i++){
		fram->Cfg.SlaveAddr = calc_slave_addr(address + (uint16_t)i);
	
		data_tx[0] = (uint8_t)((address + (uint16_t)i) & 0xFF);
		size = 1;
		fram->DataTx(data_tx, &size);
		size = 1;
		fram->DataRx(data_rx, &size);
		*(data + i) = data_rx[0];
	}
	
	return;
}

//////////////////
// 各型の読込関数 //
//////////////////
uint8_t read_byte(uint16_t address){
	uint8_t data_rx;
	read_multibyte(address, 1, &data_rx);
	return data_rx;
}

int16_t read_int16(uint16_t address){
	uint8_t data_rx[2];
	generic_16bit r;
	
	read_multibyte(address, 2, data_rx);
	r.b[0] = data_rx[0];
	r.b[1] = data_rx[1];
	return (int16_t)r.i;
}

uint16_t read_uint16(uint16_t address){
	uint8_t data_rx[2];
	generic_16bit r;
	
	read_multibyte(address, 2, data_rx);
	r.b[0] = data_rx[0];
	r.b[1] = data_rx[1];
	return (uint16_t)r.i;
}

int32_t read_int32(uint16_t address){
	uint8_t data_rx[4];
	generic_32bit r;
	
	read_multibyte(address, 4, data_rx);
	r.b[0] = data_rx[0];
	r.b[1] = data_rx[1];
	r.b[2] = data_rx[2];
	r.b[3] = data_rx[3];
	return (int32_t)r.i;
}

uint32_t read_uint32(uint16_t address){
	uint8_t data_rx[4];
	generic_32bit r;
	
	read_multibyte(address, 4, data_rx);
	r.b[0] = data_rx[0];
	r.b[1] = data_rx[1];
	r.b[2] = data_rx[2];
	r.b[3] = data_rx[3];
	return (uint32_t)r.i;
}

float read_float(uint16_t address){
	uint8_t data_rx[4];
	generic_32bit r;
	
	read_multibyte(address, 4, data_rx);
	r.b[0] = data_rx[0];
	r.b[1] = data_rx[1];
	r.b[2] = data_rx[2];
	r.b[3] = data_rx[3];
	return (float)r.f;
}

//////////////////////////
// 各型の読込関数　ここまで //
//////////////////////////

