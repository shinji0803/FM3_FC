
#ifndef FRAM_H
#define FRAM_H

// FRAM read and write funtions
// for MB85RC16 Memory = 2048~8bit

#include "I2cDev.h"

extern I2cDev_IOB I2cDev[];
extern I2cDev_IOB *fram;

//Initialize
void Init_fram(void);

//Write Function
void write_byte(uint16_t address, uint8_t data);
void write_int16(uint16_t address, int16_t data);
void write_uint16(uint16_t address, uint16_t data);
void write_int32(uint16_t address, int32_t data);
void write_uint32(uint16_t address, uint32_t data);
void write_float(uint16_t address, float data);

//Read Function
uint8_t read_byte(uint16_t address);
int16_t read_int16(uint16_t address);
uint16_t read_uint16(uint16_t address);
int32_t read_int32(uint16_t address);
uint32_t read_uint32(uint16_t address);
float read_float(uint16_t address);


#endif
