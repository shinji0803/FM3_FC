
#include "timer.h"

static volatile uint32_t _msec = 0;

volatile timeFlg time = { 0, 0, 0, 0, 0, 0, 0, 0};

void Init_timer(void)
{
	uint32_t period;
	
	/* Systick interrupt priority setting */
	NVIC_SetPriority(SysTick_IRQn, 0); // set to highest priority
	
	period = SystemCoreClock / 1000UL;
	
	SysTick_Config(period); // Interuppt every 1msec
}

void Init_DT(void)
{
	///�^�C�}�N���b�N(TIMCLK)=APB�o�X�N���b�N=72MHz
	NVIC_DisableIRQ(DTIM_QDU_IRQn);
	
	FM3_DTIM->TIMER1CONTROL_f.TIMEREN = 0; //Timer1��~
	FM3_DTIM->TIMER1CONTROL_f.TIMERMODE = 1; //�������[�h(�����V���b�g���Ɗ֌W�Ȃ��j
	FM3_DTIM->TIMER1CONTROL_f.ONESHOT = 1; //�����V���b�g
	FM3_DTIM->TIMER1CONTROL_f.INTENABLE = 1; //���荞�݋���
	FM3_DTIM->TIMER1CONTROL_f.TIMERPRE0 = 1; //�N���b�N16����
	FM3_DTIM->TIMER1CONTROL_f.TIMERPRE1 = 0;
	FM3_DTIM->TIMER1CONTROL_f.TIMERSIZE = 0; //16bit�J�E���^
	
	NVIC_EnableIRQ(DTIM_QDU_IRQn);	//DualTimer���荞�݋N��
	//�J�E���^�����l�ݒ� 0.001 * (1 / (72M / 16)) = 
	FM3_DTIM->TIMER1LOAD = 0x1194;
	FM3_DTIM->TIMER1CONTROL_f.TIMEREN = 1; //Timer1�N��
}

#define DT2_USEC_PER_TICK  (1000000.f / 4500000.f)
void Init_DT2(void)
{
	///TimerClock(TIMCLK)=APB Bus Clock = 72MHz
	NVIC_DisableIRQ(DTIM_QDU_IRQn);
	
	FM3_DTIM->TIMER2CONTROL_f.TIMEREN = 0; // Timer disable
	FM3_DTIM->TIMER2CONTROL_f.TIMERMODE = 0; // FreeRun mode
	FM3_DTIM->TIMER2CONTROL_f.ONESHOT = 0; // 
	FM3_DTIM->TIMER2CONTROL_f.INTENABLE = 1;
	FM3_DTIM->TIMER2CONTROL_f.TIMERPRE0 = 1;
	FM3_DTIM->TIMER2CONTROL_f.TIMERPRE1 = 0;
	FM3_DTIM->TIMER2CONTROL_f.TIMERSIZE = 1;
	
	NVIC_EnableIRQ(DTIM_QDU_IRQn);
}

void Start_DT2(void)
{
	FM3_DTIM->TIMER2LOAD = 0xFFFFFFFF;
	FM3_DTIM->TIMER2CONTROL_f.TIMEREN = 1;
}

uint32_t Stop_DT2(void)
{
	uint32_t count = 0;
	
	FM3_DTIM->TIMER2CONTROL_f.TIMEREN = 0;
	count = FM3_DTIM->TIMER2VALUE;
	
	return (uint32_t)((float)(0xFFFFFFFF - count) * DT2_USEC_PER_TICK);
}

void SysTick_Handler(void)
{
	_msec ++;
}

void DT_Handler(void)
{	//DT�����݃n���h��: 0.001sec��
	static uint16_t counter_1hz = 0, counter_20hz = 0, counter_50hz = 0, counter_100hz = 0;
	
	uint8_t irq;
	irq = (uint8_t)(FM3_INTREQ->IRQ06MON & 0x00000003U);
	
	if((irq & 0x01) != 0){	
		if(counter_100hz >= 10){//100Hz
			time.flg_100hz = 1;
			loop_100hz();
			counter_100hz = 0;
		}
	
		if(counter_50hz >= 20){	//50Hz
			time.flg_50hz = 1;
			loop_50hz();
			counter_50hz = 0;
		}
	
		if(counter_20hz >= 50){	//20Hz : �\������
			time.flg_20hz = 1;
			time.print = 1;
			loop_20hz();
			counter_20hz = 0;
		}

		if(counter_1hz >= 1000){	//1Hz : LED�_�Ŏ���
			time.flg_1hz = 1;
			loop_1hz();
			counter_1hz  = 0;
		}

		counter_1hz ++;
		counter_20hz ++;
		counter_50hz ++;
		counter_100hz ++;
		
		FM3_DTIM->TIMER1LOAD = 0x1194;
		FM3_DTIM->TIMER1INTCLR = 0x01; //�Ȃ�ł��������珑�����ނƊ����݃N���A�H
	}
	
	if((irq & 0x02) != 0){
		FM3_DTIM->TIMER2INTCLR = 0x01;
	}
}


uint32_t get_micros()
{
	register uint32_t millis, systickCount;
	
	do{
		millis = _msec;
		systickCount = SysTick->VAL;
	} while(millis != _msec);
	
	return (millis * 1000) + (144000 - systickCount) / 144;
}

uint32_t get_millis()
{
	return _msec;
}

void wait_usec(__IO uint32_t length)
{
	uint32_t now;
	now = get_micros();
	
	while(get_micros() < (now + length));
}

void wait(__IO uint32_t length)
{
	uint32_t now;
	now = _msec;
	
	while(_msec < (now + length));
}

