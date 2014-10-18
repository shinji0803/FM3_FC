/********************************************************************************/
/*!
	@file			uart_support.h
	@author         Nemui Trinomius (http://nemuisan.blog.bai.ne.jp)
    @version        1.00
    @date           2012.05.10
	@brief          Based on ChaN's FatFs Sample Thanks!

    @section HISTORY
		2012.05.10	V1.00	Start Here.

    @section LICENSE
		BSD License. See Copyright.txt
*/
/********************************************************************************/
#ifndef __UART_SUPPORT_H
#define __UART_SUPPORT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* General Inclusion */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>

#include "mcu.h"
#include "UartDev.h"
#include "clock_def.h"


/* USART Definition */
#define UART_BUFSIZE			256			/* UART TX&RX BufferSize in byte */
#define UART_BAUDLATE			57600UL		/* UART Baudrate */
#define UART_MFS				0			/* MFS channel to be used as UART (0-3,4-7) */
#define UART_LOC				0			/* Pin relocation number (0-2) */


#if   UART_MFS == 0
#if   UART_LOC == 0
#define ATTACH_MFS() { FM3_GPIO->ADE = FM3_GPIO->ADE & ~(1<<31); FM3_GPIO->EPFR07 = (FM3_GPIO->EPFR07 & ~(15 << 4)) | (4 << 4);  FM3_GPIO->PFR2 |= 3 << 1; }
#elif UART_LOC == 1
#define ATTACH_MFS() { FM3_GPIO->EPFR07 = (FM3_GPIO->EPFR07 & ~(15 << 4)) | (10 << 4); FM3_GPIO->PFR1 |= 3 << 4; }
#elif UART_LOC == 2
#define ATTACH_MFS() { FM3_GPIO->EPFR07 = (FM3_GPIO->EPFR07 & ~(15 << 4)) | (15 << 4); FM3_GPIO->PFRB |= 3 << 4; }
#endif

#elif UART_MFS == 1
#if   UART_LOC == 0
#define ATTACH_MFS() { FM3_GPIO->EPFR07 = (FM3_GPIO->EPFR07 & ~(15 << 10)) | (4 << 10);  FM3_GPIO->PFR5 |= 3 << 6; }
#elif UART_LOC == 1
#define ATTACH_MFS() { FM3_GPIO->EPFR07 = (FM3_GPIO->EPFR07 & ~(15 << 10)) | (10 << 10); FM3_GPIO->PFR1 |= 3 << 1; }
#elif UART_LOC == 2
#define ATTACH_MFS() { FM3_GPIO->EPFR07 = (FM3_GPIO->EPFR07 & ~(15 << 10)) | (15 << 10); FM3_GPIO->PFRF |= 3 << 0; }
#endif

#elif UART_MFS == 2
#if   UART_LOC == 0
#define ATTACH_MFS() { FM3_GPIO->EPFR07 = (FM3_GPIO->EPFR07 & ~(15 << 16)) | (4 << 16);  FM3_GPIO->PFR7 |= 3 << 2; }
#elif UART_LOC == 1
#define ATTACH_MFS() { FM3_GPIO->EPFR07 = (FM3_GPIO->EPFR07 & ~(15 << 16)) | (10 << 16); FM3_GPIO->PFR2 |= 3 << 4; }
#elif UART_LOC == 2
#define ATTACH_MFS() { FM3_GPIO->EPFR07 = (FM3_GPIO->EPFR07 & ~(15 << 16)) | (15 << 16); FM3_GPIO->PFR1 |= 3 << 7; }
#endif

#elif UART_MFS == 3
#if   UART_LOC == 0
#define ATTACH_MFS() { FM3_GPIO->EPFR07 = (FM3_GPIO->EPFR07 & ~(15 << 22)) | (4 << 22);  FM3_GPIO->PFR7 |= 3 << 5; }
#elif UART_LOC == 1
#define ATTACH_MFS() { FM3_GPIO->EPFR07 = (FM3_GPIO->EPFR07 & ~(15 << 22)) | (10 << 22); FM3_GPIO->PFR5 |= 3 << 0; }
#elif UART_LOC == 2
#define ATTACH_MFS() { FM3_GPIO->EPFR07 = (FM3_GPIO->EPFR07 & ~(15 << 22)) | (15 << 22); FM3_GPIO->PFR4 |= 3 << 8; }
#endif

#elif UART_MFS == 4
#if   UART_LOC == 0
#define ATTACH_MFS() { FM3_GPIO->EPFR08 = (FM3_GPIO->EPFR08 & ~(15 << 4)) | (4 << 4);  FM3_GPIO->PFRD |= 3 << 1; }
#elif UART_LOC == 1
#define ATTACH_MFS() { FM3_GPIO->EPFR08 = (FM3_GPIO->EPFR08 & ~(15 << 4)) | (10 << 4); FM3_GPIO->PFR1 |= 3 << 10; }
#elif UART_LOC == 2
#define ATTACH_MFS() { FM3_GPIO->EPFR08 = (FM3_GPIO->EPFR08 & ~(15 << 4)) | (15 << 4); FM3_GPIO->PFR0 |= 3 << 5; }
#endif

#elif UART_MFS == 5
#if   UART_LOC == 0
#define ATTACH_MFS() { FM3_GPIO->EPFR08 = (FM3_GPIO->EPFR08 & ~(15 << 10)) | (4 << 10);  FM3_GPIO->PFR6 |= 3 << 0; }
#elif UART_LOC == 1
#define ATTACH_MFS() { FM3_GPIO->EPFR08 = (FM3_GPIO->EPFR08 & ~(15 << 10)) | (10 << 10); FM3_GPIO->PFR9 |= 3 << 2; }
#elif UART_LOC == 2
#define ATTACH_MFS() { FM3_GPIO->EPFR08 = (FM3_GPIO->EPFR08 & ~(15 << 10)) | (15 << 10); FM3_GPIO->PFR3 |= 3 << 6; }
#endif

#elif UART_MFS == 6
#if   UART_LOC == 0
#define ATTACH_MFS() { FM3_GPIO->EPFR08 = (FM3_GPIO->EPFR08 & ~(15 << 16)) | (4 << 16);  FM3_GPIO->PFR5 |= 3 << 3; }
#elif UART_LOC == 1
#define ATTACH_MFS() { FM3_GPIO->EPFR08 = (FM3_GPIO->EPFR08 & ~(15 << 16)) | (10 << 16); FM3_GPIO->PFR3 |= 3 << 2; }
#elif UART_LOC == 2
#define ATTACH_MFS() { FM3_GPIO->EPFR08 = (FM3_GPIO->EPFR08 & ~(15 << 16)) | (15 << 16); FM3_GPIO->PFRF |= 3 << 3; }
#endif

#elif UART_MFS == 7
#if   UART_LOC == 0
#define ATTACH_MFS() { FM3_GPIO->EPFR08 = (FM3_GPIO->EPFR08 & ~(15 << 22)) | (4 << 22);  FM3_GPIO->PFR5 |= 3 << 9; }
#elif UART_LOC == 1
#define ATTACH_MFS() { FM3_GPIO->EPFR08 = (FM3_GPIO->EPFR08 & ~(15 << 22)) | (10 << 22); FM3_GPIO->PFR4 |= 3 << 13; }
#elif UART_LOC == 2
#define ATTACH_MFS() { FM3_GPIO->EPFR08 = (FM3_GPIO->EPFR08 & ~(15 << 22)) | (15 << 22); FM3_GPIO->PFRB |= 3 << 0; }
#endif

#endif


extern UartDev_IOB UartDev[];


/* Funcion Prototypes */
extern void conio_init(uint32_t baudrate);
extern int32_t conio_available(void);

extern void putch(uint8_t c);
extern uint8_t getch(void);
extern uint8_t keypressed(void);
extern void cputs(char *s);
extern void cgets(char *s, int bufsize);

/* Externs */


#ifdef __cplusplus
}
#endif

#endif	/* __UART_SUPPORT_H */
