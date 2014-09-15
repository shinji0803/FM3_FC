
#include "rc_io.h"

static FM3_BT_PWC_TypeDef *rcin_list[]		= { IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8};
static FM3_BT_PWM_TypeDef *rcout_list[]		= { OUT1, OUT2, OUT3, OUT4, OUT5, OUT6, OUT7, OUT8};

static uint16_t rc_in[8]	= { 0, 0, 0, 0, 0, 0, 0, 0};
static uint16_t rc_out[8]	= { 0x4AB5, 0x4AB5, 0x4AB5, 0x4AB5, 0x4AB5, 0x4AB5, 0x4AB5, 0x4AB5};

static void Init_rcin_port(uint8_t ch);
static void Init_rcout_port(uint8_t ch);
	
static uint16_t pwm_period = (uint16_t)(INTERRUPT_FREQ / PWM_FREQ);


static void Init_rcin_port(uint8_t ch){
	
	switch(ch){
		case 0:
		//Ch.1 BT0 TIOB00_0
		FM3_GPIO->PFR4_f.P9 = 1; //P49をPWM入力
		FM3_GPIO->EPFR04_f.TIOB0S0 = 0; //TIOB00_0
		FM3_GPIO->EPFR04_f.TIOB0S1 = 0;
		break;
		
		case 1:
		//Ch.2 BT1 TIOB01_0
		FM3_GPIO->PFR3_f.P1 = 1; //P31をPWM入力
		FM3_GPIO->EPFR04_f.TIOB1S0 = 0; //TIOB01_0
		FM3_GPIO->EPFR04_f.TIOB1S1 = 1;
		break;
		
		case 2:
		//Ch.3 BT2 TIOB02_1
		FM3_GPIO->PFR3_f.P2 = 1; //P32をPWM入力
		FM3_GPIO->EPFR04_f.TIOB2S0 = 0; //TIOB02_1
		FM3_GPIO->EPFR04_f.TIOB2S1 = 1;
		break;
		
		case 3:
		//Ch.4 BT3 TIOB03_1
		FM3_GPIO->PFR3_f.P3 = 1; //P33をPWM入力
		FM3_GPIO->EPFR04_f.TIOB3S0 = 0; //TIOB03_1
		FM3_GPIO->EPFR04_f.TIOB3S1 = 1;
		break;
	}
}

//PWM出力ポート設定
static void Init_rcout_port(uint8_t ch){
	
	switch(ch){
		case 0:
		//Ch.1 BT8 TIOA08_0
		FM3_GPIO->PFRA_f.P0 = 1; //PA0をPWM出力
		FM3_GPIO->EPFR12_f.TIOA8E0 = 1; //TIOA08_0
		FM3_GPIO->EPFR12_f.TIOA8E1 = 0;
		break;
		
		case 1:
		//Ch.2 BT9 TIOA09_0
		FM3_GPIO->PFRA_f.P1 = 1; //PA1をPWM出力
		FM3_GPIO->EPFR12_f.TIOA9E0 = 1; //TIOA09_0
		FM3_GPIO->EPFR12_f.TIOA9E1 = 0;
		break;
		
		case 2:
		//Ch.3 BT10 TIOA10_0
		FM3_GPIO->PFRB_f.P2 = 1; //PB2をPWM出力
		FM3_GPIO->EPFR12_f.TIOA10E0 = 0; //TIOA10_1
		FM3_GPIO->EPFR12_f.TIOA10E1 = 1;
		break;
		
		case 3:
		//Ch.4 BT11 TIOA11_0
		FM3_GPIO->PFRA_f.P3 = 1; //PA3をPWM出力
		FM3_GPIO->EPFR12_f.TIOA11E0 = 0; //TIOA11_0
		FM3_GPIO->EPFR12_f.TIOA11E1 = 0;
		break;
	}
}


void rcin_enable(uint8_t ch){
	FM3_BT_PWC_TypeDef *rcin;
	uint8_t in = ch;
	if(in >= MAX_CH) in = MAX_CH - 1;
	
	rcin = rcin_list[in];
	
	//Port setting
	Init_rcin_port(in);	
	
	//Timer setting
	rcin->TMCR_f.CTEN = 0; //カウンタ停止
	rcin->TMCR_f.FMD = 0x04; //16bitPWCタイマ選択
	rcin->TMCR_f.T32 = 0;
	rcin->TMCR_f.CKS0 = 0; //カウンタクロック選択
	rcin->TMCR_f.CKS1 = 1; //72MHz/16 = 4.5MHz
	rcin->TMCR_f.CKS2 = 0; //
	rcin->TMCR2_f.CKS3 = 0;
#ifndef USE_PPM
	rcin->TMCR_f.EGS0 = 0; //Hパルス幅測定
	rcin->TMCR_f.EGS1 = 0;
	rcin->TMCR_f.EGS2 = 0;
#else
	rcin->TMCR_f.EGS0 = 1; //立ち上がり周期測定
	rcin->TMCR_f.EGS1 = 0;
	rcin->TMCR_f.EGS2 = 0;
#endif
	rcin->TMCR_f.MDSE = 0; //測定
	
	rcin->STC_f.EDIE = 1; //測定終了割り込み有効
	rcin->STC_f.OVIE = 0; //オーバフロー割り込み無効
	
	NVIC_EnableIRQ(BTIM0_7_IRQn); 
	rcin->TMCR_f.CTEN = 1; //ダウンカウンタ起動
	
}	

void rcout_enable(uint8_t ch){
	FM3_BT_PWM_TypeDef *rcout;
	uint8_t out = ch;
	if(out >= MAX_CH) out = MAX_CH - 1;
	
	rcout = rcout_list[out];
	
	Init_rcout_port(out);
	
	rcout->TMCR_f.CTEN = 0; //カウンタ停止
	rcout->TMCR_f.STRG = 0; //ソフトウェアトリガ無効
	rcout->TMCR_f.FMD = 0x01; //16bitPWMタイマ選択 FMDはビット単位で変更できないくさい
	rcout->TMCR_f.CKS0 = 0; //カウンタクロック選択
	rcout->TMCR_f.CKS1 = 1; //36MHz/16 = 2.25MHz
	rcout->TMCR_f.CKS2 = 0; //
	rcout->TMCR2_f.CKS3 = 0;
	rcout->TMCR_f.PMSK = 0; //PWM通常出力
	rcout->TMCR_f.RTGEN = 0; //再起動禁止
	rcout->TMCR_f.EGS0 = 0; //測定エッジ選択
	rcout->TMCR_f.EGS1 = 0; //トリガ入力無効
	rcout->TMCR_f.OSEL = 1; //H出力から開始 
	rcout->TMCR_f.MDSE = 1; //リロードorワンショット　(0:リロード）
	
	rcout->STC_f.TGIE = 0; //トリガ割り込み無効
	rcout->STC_f.DTIE = 0; //デューティ一致割り込み無効
	rcout->STC_f.UDIE = 1; //アンダフロー割り込み有効
	rcout->STC_f.TGIR = 0; //トリガ割り込みフラグクリア
	rcout->STC_f.DTIR = 0; //デューティフラグクリア
	rcout->STC_f.UDIR = 0; //アンダフローフラグクリア
	
	//RCサーボ用
	rcout->PCSR = pwm_period; //PWM周期設定=0.01sec周期
	rcout->PDUT = 1520 * TICK_PER_USEC;  //デューティ比(オフ幅）　
	//center 1.5msec = 4AB5
	//max 2.0msec = 4650
	//min 1.0msec = 4F1A
	
	NVIC_EnableIRQ(BTIM8_15_IRQn); //割り込み起動
	rcout->TMCR_f.CTEN = 1; //ダウンカウンタ起動
	rcout->TMCR_f.STRG = 1; 
}


void rc_write(uint8_t ch, uint16_t pulse_usec){
	uint16_t pulse = pulse_usec;
	uint16_t duty;
	uint8_t out = ch;
	
	if(pulse > PWM_MAX_PULSE_WIDTH) pulse =  PWM_MAX_PULSE_WIDTH;
	if(pulse < PWM_MIN_PULSE_WIDTH) pulse =  PWM_MIN_PULSE_WIDTH;
	
	if(out >= MAX_CH) out = MAX_CH - 1;
	
	//PWM出力はパルスのOFF幅指定
	duty = pwm_period - (uint16_t)(pulse * TICK_PER_USEC);
	
	rc_out[ch] = duty;
}

uint16_t rc_read(uint8_t ch){
	return rc_in[ch];
}

uint16_t rcout_read(uint8_t ch){
	return (uint16_t)((pwm_period - rc_out[ch]) * USEC_PER_TICK);
}

void rc_multiread(uint16_t *data){
	uint8_t i = 0;
	for(i = 0; i < 8; i++) data[i] = rc_read(i);
}

void BT0_7_IRQHandler(void){
	static uint16_t pulse = 0;
#ifdef USE_PPM
	static uint8_t invalid = 0;
	static int8_t count = 0;
#endif
	
	if(FM3_BT0_PWC->STC_f.EDIR == 1){
		pulse = FM3_BT0_PWC->DTBF;

#ifdef USE_PPM
		//無効区間はパルス幅が10msecくらい
		//5msec以上で無効幅と判断
		if(pulse > (5000 * TICK_PER_USEC)){
			invalid = 1;
			count = -1;
		}
		
		if(invalid == 1){
			//1100~1940に変換
			if(count >= 0) 	rc_in[count] = (uint16_t)(pulse * USEC_PER_TICK);
			count ++;
			
			if(count == 8){
				invalid = 0;
				count = -1;
			}
		}
#else
		rc_in[0] = (uint16_t)(pulse * USEC_PER_TICK);
#endif
		
	}
	
	if(FM3_BT1_PWC->STC_f.EDIR == 1){ pulse = FM3_BT1_PWC->DTBF;	rc_in[1] = (uint16_t)(pulse * USEC_PER_TICK);}
	if(FM3_BT2_PWC->STC_f.EDIR == 1){ pulse = FM3_BT2_PWC->DTBF;	rc_in[2] = (uint16_t)(pulse * USEC_PER_TICK);}
	if(FM3_BT3_PWC->STC_f.EDIR == 1){ pulse = FM3_BT3_PWC->DTBF;	rc_in[3] = (uint16_t)(pulse * USEC_PER_TICK);}
	if(FM3_BT4_PWC->STC_f.EDIR == 1){ pulse = FM3_BT4_PWC->DTBF;	rc_in[4] = (uint16_t)(pulse * USEC_PER_TICK);}
	if(FM3_BT5_PWC->STC_f.EDIR == 1){ pulse = FM3_BT5_PWC->DTBF;	rc_in[5] = (uint16_t)(pulse * USEC_PER_TICK);}
	if(FM3_BT6_PWC->STC_f.EDIR == 1){ pulse = FM3_BT6_PWC->DTBF;	rc_in[6] = (uint16_t)(pulse * USEC_PER_TICK);}
	if(FM3_BT7_PWC->STC_f.EDIR == 1){ pulse = FM3_BT7_PWC->DTBF;	rc_in[7] = (uint16_t)(pulse * USEC_PER_TICK);}
}


void BT8_15_IRQHandler(void){
	
	if(FM3_BT8_PWM->STC_f.UDIR == 1){
		FM3_BT8_PWM->STC_f.UDIR = 0;
		FM3_BT8_PWM->PDUT = rc_out[0];
		FM3_BT8_PWM->TMCR_f.STRG = 1;
	}
	if(FM3_BT9_PWM->STC_f.UDIR == 1){
		FM3_BT9_PWM->STC_f.UDIR = 0;
		FM3_BT9_PWM->PDUT = rc_out[1];
		FM3_BT9_PWM->TMCR_f.STRG = 1;
	}
	if(FM3_BT10_PWM->STC_f.UDIR == 1){
		FM3_BT10_PWM->STC_f.UDIR = 0;
		FM3_BT10_PWM->PDUT = rc_out[2];
		FM3_BT10_PWM->TMCR_f.STRG = 1;
	}
	if(FM3_BT11_PWM->STC_f.UDIR == 1){
		FM3_BT11_PWM->STC_f.UDIR = 0;
		FM3_BT11_PWM->PDUT = rc_out[3];
		FM3_BT11_PWM->TMCR_f.STRG = 1;
	}
	if(FM3_BT12_PWM->STC_f.UDIR == 1){
		FM3_BT12_PWM->STC_f.UDIR = 0;
		FM3_BT12_PWM->PDUT = rc_out[4];
		FM3_BT12_PWM->TMCR_f.STRG = 1;
	}
	if(FM3_BT13_PWM->STC_f.UDIR == 1){
		FM3_BT13_PWM->STC_f.UDIR = 0;
		FM3_BT13_PWM->PDUT = rc_out[5];
		FM3_BT13_PWM->TMCR_f.STRG = 1;
	}
	if(FM3_BT14_PWM->STC_f.UDIR == 1){
		FM3_BT14_PWM->STC_f.UDIR = 0;
		FM3_BT14_PWM->PDUT = rc_out[6];
		FM3_BT14_PWM->TMCR_f.STRG = 1;
	}
	if(FM3_BT15_PWM->STC_f.UDIR == 1){
		FM3_BT15_PWM->STC_f.UDIR = 0;
		FM3_BT15_PWM->PDUT = rc_out[7];
		FM3_BT15_PWM->TMCR_f.STRG = 1;
	}
	
}
