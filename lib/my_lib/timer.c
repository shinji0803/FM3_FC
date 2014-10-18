
#include "timer.h"

static volatile uint32_t usec = 0;
static volatile uint32_t usec_delay_counter = 0;

static inline void TimingDelay_Decrement(void);

volatile timeFlg time = { 0, 0, 0, 0, 0, 0, 0, 0};

void Init_timer(){
	
	uint32_t period;
	period = SystemCoreClock / 1000000;
	
	SysTick_Config(period); //1usecで割り込み設定
}

void Init_DT(void){
	///タイマクロック(TIMCLK)=APBバスクロック=72MHz
	
	FM3_DTIM->TIMER1CONTROL_f.TIMEREN = 0; //Timer1停止
	FM3_DTIM->TIMER1CONTROL_f.TIMERMODE = 1; //周期モード(ワンショットだと関係なし）
	FM3_DTIM->TIMER1CONTROL_f.ONESHOT = 1; //ワンショット
	FM3_DTIM->TIMER1CONTROL_f.INTENABLE = 1; //割り込み許可
	FM3_DTIM->TIMER1CONTROL_f.TIMERPRE0 = 1; //クロック16分周
	FM3_DTIM->TIMER1CONTROL_f.TIMERPRE1 = 0;
	FM3_DTIM->TIMER1CONTROL_f.TIMERSIZE = 0; //16bitカウンタ
	
	NVIC_EnableIRQ(DTIM_QDU_IRQn);	//DualTimer割り込み起動
	//カウンタ初期値設定 0.001 * (1 / (72M / 16)) = 
	FM3_DTIM->TIMER1LOAD = 0x1194;
	FM3_DTIM->TIMER1CONTROL_f.TIMEREN = 1; //Timer1起動
}

void SysTick_Handler(void){
	usec ++;
	
	TimingDelay_Decrement();
}

void DT_Handler(void){	//DT割込みハンドラ: 0.001sec毎
	static uint16_t counter_1hz = 0, counter_20hz = 0, counter_50hz = 0, counter_100hz = 0, counter_200hz = 0;

	if(counter_200hz >= 5){	//200Hz
		time.flg_200hz = 1;
		loop_200hz();
		counter_200hz = 0;
	}
	
	if(counter_100hz >= 10){	//100Hz
		time.flg_100hz = 1;
		loop_100hz();
		counter_100hz = 0;
	}
	
	if(counter_50hz >= 20){	//50Hz
		time.flg_50hz = 1;
		loop_50hz();
		counter_50hz = 0;
	}
	
	if(counter_20hz >= 50){	//20Hz : 表示周期
		time.flg_20hz = 1;
		time.print = 1;
		loop_20hz();
		counter_20hz = 0;
	}

	if(counter_1hz >= 1000){	//1Hz : LED点滅周期
		time.flg_1hz = 1;
		loop_1hz();
		counter_1hz  = 0;
	}

	counter_1hz ++;
	counter_20hz ++;
	counter_50hz ++;
	counter_100hz ++;
	counter_200hz ++;
	
	FM3_DTIM->TIMER1LOAD = 0x1194;
	FM3_DTIM->TIMER1INTCLR = 0x01; //なんでもいいから書き込むと割込みクリア？
}


uint32_t get_micros(){
	return usec;
}

uint32_t get_millis(){
	return (uint32_t)(usec / 1000);
}

void wait_usec(__IO uint32_t length){
	
	usec_delay_counter = length;
	
	while(usec_delay_counter != 0);
}

void wait(__IO uint32_t length){
	
	usec_delay_counter = length * 1000;
	
	while(usec_delay_counter != 0);
}

static inline void TimingDelay_Decrement(){
	if (usec_delay_counter != 0x00){
		usec_delay_counter --;
	}
}
