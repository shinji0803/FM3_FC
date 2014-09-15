/********************************************************************************/
/*!
	@file			hw_config.h
	@author         Nemui Trinomius (http://nemuisan.blog.bai.ne.jp)
    @version        1.00
    @date           2012.02.02
	@brief          For FUJITSU MB9BF618T JunkBoard Versaion.

    @section HISTORY
		2012.02.02	V1.00	Start Here.

    @section LICENSE
		BSD License. See Copyright.txt
*/
/********************************************************************************/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H 0x0100

#ifdef __cplusplus
 extern "C" {
#endif

/* General Inclusion */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

/* Function Inclusion */
#include "mcu.h"
//#include "systick.h"
#include "uart_support.h"

/* High Level Function */
/*
#include "xprintf.h"
#include "diskio.h"
#include "ff.h"
#include "ff_support.h"
#include "term_io.h"
#include "display_if.h"
#include "display_if_support.h"
#include "ts_fileloads.h"
*/

/* Macros */
#define countof(a)   (sizeof(a) / sizeof(*(a)))
/* See systick.h */
#define Cyclic_Exec()	{			\
		}

/* Externals */
extern void initialise_monitor_handles(void);

#ifdef __cplusplus
}
#endif

#endif  /* __HW_CONFIG_H */
