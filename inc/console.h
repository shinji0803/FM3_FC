
#ifndef CONSOLE_H
#define CONSOLE_H

//コンソールとか　いわゆるCLI

#include "uart_support.h"
#include "param.h"
#include "fram.h"

#include "AHRS.h"

//Command type
#define	AHRS			'a'
#define	RADIO			'r'
#define	FLOW			'f'
#define	RADIO_FLOW		'w'
#define	SET_GAIN 		'g'
#define	DISPLAY			'd'
#define	SET_P			'p'
#define	SET_I			'i'
#define	SET_D			'd'
#define SET_Q_TH		'q'
#define	SAVE			's'
#define	RESTORE			'r'
#define EXIT			'e'

#define SET_RP_P		'1'
#define SET_RP_I		'2'
#define SET_Y_P			'3'
#define SET_Y_I			'4'

extern uint8_t p_flg;
extern uint8_t menu_flg;
extern uint8_t input_detect;

void top_menu(void); //print top menu
	
void gain_menu(void); //print gain set menu

void gain_menu_branch(uint8_t com_type);

void top_menu_branch(uint8_t com_type);

#endif
