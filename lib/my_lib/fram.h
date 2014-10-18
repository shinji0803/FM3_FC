
#ifndef FRAM_H
#define FRAM_H

//FRAM用の関数
//MB85RC16用　容量2048×8bit

#include "I2cDev.h"

extern I2cDev_IOB I2cDev[];
extern I2cDev_IOB *fram;

//初期化　FRAMとつながるI2Cポートを設定
void Init_fram(void);

//書き込み関数
void write_byte(uint16_t address, uint8_t data);
void write_int16(uint16_t address, int16_t data);
void write_uint16(uint16_t address, uint16_t data);
void write_int32(uint16_t address, int32_t data);
void write_uint32(uint16_t address, uint32_t data);
void write_float(uint16_t address, float data);

//読み込み関数
uint8_t read_byte(uint16_t address);
int16_t read_int16(uint16_t address);
uint16_t read_uint16(uint16_t address);
int32_t read_int32(uint16_t address);
uint32_t read_uint32(uint16_t address);
float read_float(uint16_t address);


#endif
