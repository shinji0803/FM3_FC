
#include <math.h>

#include "hw_config.h"

#include "i2c.h"
#include "timer.h"
#include "rc_io.h"
#include "fram.h"
#include "console.h"

#include "myMath.h"
#include "AHRS.h"

#include "mavlink.h"
#include "mavlink_support.h"

#define VectorPrintf(v) printf("%+6.4f, %+6.4f, %+6.4f", v.x, v.y, v.z)

static void InitLED(void);
inline void loop_1hz(void);
inline void loop_20hz(void);
inline void loop_50hz(void);
inline void loop_100hz(void);
inline void loop_200hz(void);
extern volatile timeFlg time;

extern mavlink_message_t msg;

volatile uint32_t now = 0, rev_update = 0, delta_time = 0;
uint8_t input_detect = 0;
uint8_t p_flg, input, menu_flg;

int32_t main(void){

	uint32_t start, end;

	//‰Šú‰»ŠJŽn
	conio_init(57600UL);
	
	Init_timer();
	InitLED();
	rcin_enable(0);
	
	Init_i2c();
	Init_fram();
	Init_DT();
	
	printf("Initialize OK.\r\n");
	wait(100);
	//‰Šú‰»I—¹
	
	AHRS_Init();
	
	Mavlink_port_init(2, 57600UL);
		
	Vector3f gyro;
	Vector3f acc, mag;
	Vector3f attitude;
	
	while(1){
		Mavlink_rx_check();
		
		if(time.flg_1hz == 1){
			time.flg_1hz = 0;
			Mavlink_heartbeat_send();
		}
		
		if(time.flg_20hz == 1){
			time.flg_20hz = 0;
			//rc_multiread(radio);
			//Mavlink_rcin_raw_send(radio);
			
			AHRS_get_raw_gyro(&gyro);
			AHRS_get_raw_acc(&acc);
			AHRS_get_raw_mag(&mag);
			Mavlink_imu_raw_send( &acc, &gyro, &mag);
			
			//Mavlink_debug_send( 0, (float)(delta_time) / 1000000.f);
			//Mavlink_debug_send( 1, (float)(end - start));
		}
		
		if(time.flg_50hz == 1){
			time.flg_50hz = 0;
			
			AHRS_get_euler(&attitude);
			AHRS_get_omega(&gyro);
			Mavlink_att_send(&attitude, &gyro);
		}
		
		if(time.flg_100hz == 1){
			time.flg_100hz = 0;
			start = get_micros();
			AHRS_read_imu();
			
			now = get_micros();
			delta_time = now - rev_update;
			AHRS_dcm_update((float)delta_time / 1000000.f);
			rev_update = now;
			
			AHRS_dcm_normalize();
			AHRS_drift_correction();
			end = get_micros();
		}
		
		if(input_detect == 1){
			if(menu_flg == 0){
				if(input == '\n'){
					input_detect = 0;
					top_menu();
				}
				else if(input != '\r')	top_menu_branch(input);
			}
			
			else if(menu_flg == 1){
				input_detect = 0;
				if(input == '\n') gain_menu();
				if(input != '\r' && input != '\n') gain_menu_branch(input);	
			}
		}
	} 
}


static void InitLED(void)
{
	FM3_GPIO->PFRF_f.P3		= 0; 
	FM3_GPIO->PZRF_f.P3		= 1; 
	FM3_GPIO->DDRF_f.P3		= 1;
	FM3_GPIO->PDORF_f.P3	= 0;
}

inline void loop_1hz(){
	if(time.calibrate != 1) FM3_GPIO->PDORF_f.P3 = ~FM3_GPIO->PDORF_f.P3;
}

inline void loop_20hz(){
	if(time.calibrate != 0) FM3_GPIO->PDORF_f.P3 = ~FM3_GPIO->PDORF_f.P3;
	p_flg = 1;
}

inline void loop_50hz(){

}

inline void loop_100hz(){
	//detect user input
	if(conio_available() > 0){
		input_detect = 1;
		input = getch();
	}
}

inline void loop_200hz(){

}

