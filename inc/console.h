
#ifndef CONSOLE_H
#define CONSOLE_H

//コンソールとか　いわゆるCLI

#include "uart_support.h"
#include "param.h"
#include "fram.h"

#include "AHRS.h"

typedef struct __menu{
	uint8_t command;
	uint8_t brief[64];
	void (*run)(void);
} Menu;

extern uint8_t p_flg;

void console_init(void);
void console_run(void);

#endif
