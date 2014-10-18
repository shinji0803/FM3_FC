
#ifndef TIMER_H
#define TIMER_H

#include "hw_config.h"

//SysTickタイマの設定（時間計測用）
void Init_timer(void);

//タイマ値を返す
uint32_t get_micros(void);
uint32_t get_millis(void);

//Wait関数（引数usec）
void wait_usec(uint32_t length);

void wait(uint32_t length);

//デュアルタイマの設定
void Init_DT(void);

//デュアルタイマハンドラ内で実行する関数
extern void loop_1hz(void);
extern void loop_20hz(void);
extern void loop_50hz(void);
extern void loop_100hz(void);
extern void loop_200hz(void);

//各周期用のフラグ
typedef struct timeFlg{
	unsigned int flg_1hz   :	1;
	unsigned int flg_10hz  :	1;
	unsigned int flg_20hz  :	1;
	unsigned int flg_50hz  :	1;
	unsigned int flg_100hz :	1;
	unsigned int flg_200hz :	1;
	unsigned int reserve1  :	1;
	unsigned int calibrate :	1;
} timeFlg;


#endif

