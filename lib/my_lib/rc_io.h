
#ifndef RC_IO_H
#define RC_IO_H

//RC入出力関数
//PWMを読む，PWMを出力する etc...

#include "hw_config.h"

#define USE_PPM

#define PWM_FREQ 100.f
#define INTERRUPT_FREQ 4500000.f

#define USEC_PER_TICK (1000000.f / INTERRUPT_FREQ)
#define TICK_PER_USEC (1.0f / USEC_PER_TICK)

#define PWM_MAX_PULSE_WIDTH 2100
#define PWM_MIN_PULSE_WIDTH 900

#define MAX_CH 4

//PWM読込設定関数
void rcin_enable(uint8_t ch);

#define IN1 FM3_BT0_PWC
#define IN2 FM3_BT1_PWC
#define IN3 FM3_BT2_PWC
#define IN4 FM3_BT3_PWC
#define IN5 FM3_BT4_PWC
#define IN6 FM3_BT5_PWC
#define IN7 FM3_BT6_PWC
#define IN8 FM3_BT7_PWC


//PWM出力設定関数
void rcout_enable(uint8_t ch);

#define OUT1 FM3_BT8_PWM
#define OUT2 FM3_BT9_PWM
#define OUT3 FM3_BT10_PWM
#define OUT4 FM3_BT11_PWM
#define OUT5 FM3_BT12_PWM
#define OUT6 FM3_BT13_PWM
#define OUT7 FM3_BT14_PWM
#define OUT8 FM3_BT15_PWM


//指定Chに指定パルス幅でPWM出力(ch = 1~8)
void rc_write(uint8_t ch, uint16_t pulse_usec);

//指定ChのPWM入力パルス幅取得(ch = 1~8)
uint16_t rc_read(uint8_t ch);

//8ch分の入力パルス幅取得
void rc_multiread(uint16_t *data);

//指定ChのPWM出力パルス幅取得(ch = 1~8)
uint16_t rcout_read(uint8_t ch);

#endif
