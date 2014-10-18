
#ifndef CONSOLE_H
#define CONSOLE_H

//Console control functions

#include "mcu.h"
#include "timer.h"

typedef struct __menu{
	uint8_t command;
	uint8_t brief[64];
	void (*run)(void);
} Menu;

extern volatile timeFlg time;

void console_init(void);
void console_run(void);

#endif
