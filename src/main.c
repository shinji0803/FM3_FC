
#include <math.h>

#include "hw_config.h"

#include "i2c.h"
#include "timer.h"
#include "rc_io.h"
#include "fram.h"
#include "console.h"
#include "scheduler.h"

#include "myMath.h"
#include "AHRS.h"

#include "mavlink.h"
#include "mavlink_support.h"

#define MAIN_LOOP_FREQ_MICROS 10000
#define VectorPrintf(v) printf("%+6.4f, %+6.4f, %+6.4f", v.x, v.y, v.z)

static void main_loop(void);
static void send_mavlink(void);

static void InitLED(void);
inline void loop_1hz(void);
inline void loop_20hz(void);
inline void loop_50hz(void);
inline void loop_100hz(void);

static const Task scheduler_tasks[] = {
	{ console_run, 		5, 		1500},
	{ send_mavlink,		5,		120}
};

int32_t main(void){
	uint32_t main_loop_start, time_available = 0;
	
	//Initialize start
	conio_init(57600UL);
	Init_timer();
	InitLED();
	
	/* RC In & Out port initialize */
	rcin_enable(0); 
	rcout_enable(0);
	rcout_enable(1);
	rcout_enable(2);
	rcout_enable(3);
	
	Init_i2c();
	Init_fram();
	Init_DT();
	Init_DT2();
	
	wait(100);
	printf("Initialize OK.\r\n");
	//Initialize complete
	
	Mavlink_port_init(2, 57600UL);
	
	AHRS_Init();
	console_init();
	scheduler_init((Task *)scheduler_tasks, sizeof(scheduler_tasks) / sizeof(scheduler_tasks[0]));
	
	while(1){
		main_loop_start = get_micros();
		
		main_loop();
		
		scheduler_tick();
		
		time_available = (main_loop_start + MAIN_LOOP_FREQ_MICROS) - get_micros();
		scheduler_run(time_available);
	} 
}

/* Fast Loop */
/* This funtion run as fast as possible. */
/* Run frequency is change by other tasks. But this function is run at least 100hz */
void main_loop(void)
{
	uint32_t dt = 0, now = 0;
	static uint32_t main_loop_timer = 0;
	
	/* Update AHRS */
	AHRS_read_imu();
	
	dt = Stop_DT2();
	now = get_micros();
	
	if(dt == 0) dt = 10000;
	AHRS_dcm_update(dt / 1000000.f);
	
	//set_debug_msg("AHRS_Rate = %d, %d", dt, (now - main_loop_timer));
	main_loop_timer = now;
	Start_DT2();
	
	AHRS_dcm_normalize();
	AHRS_drift_correction();
	AHRS_calc_euler();
	
}

#define MAVLINK_SEND_MSG_NUM 3
static void send_mavlink(void)
{
	static uint8_t mav_msg[MAVLINK_MAX_PACKET_LEN];
	static int32_t msg_length, sent_byte = 0, remain_byte = 0;
	
	static uint32_t last_send[MAVLINK_SEND_MSG_NUM] = { 0, 0, 0};
	static const uint32_t send_interval[MAVLINK_SEND_MSG_NUM] = { 1000, 100, 50};
	
	uint32_t now;
	int32_t dt;
	uint32_t mindt;
	Vector3f temp1, temp2, temp3;
	uint8_t i, send_index = 0;
	
	if(Mavlink_enabled() != 0){
		Mavlink_rx_check();
		
		if(remain_byte <= 0){
			now = get_millis();
			mindt = now;
			for(i = 0; i < MAVLINK_SEND_MSG_NUM; i ++){
				dt = (int32_t)(now - (last_send[i] + send_interval[i]));
				if(labs(dt) < mindt){
					mindt = labs(dt);
					send_index = i;
				}
			}
			
			/* Create send message and get message length */
			switch(send_index){
				case 0:
				Mavlink_get_heartbeat_msg(mav_msg, &msg_length);
				break;
				
				case 1:
				AHRS_get_euler(&temp1);
				AHRS_get_omega(&temp2);
				Mavlink_get_att_msg(mav_msg, &msg_length, &temp1, &temp2);
				break;
				
				case 2:
				AHRS_get_raw_acc(&temp1);
				AHRS_get_raw_gyro(&temp2);
				AHRS_get_raw_mag(&temp3);
				Mavlink_get_imu_raw_msg(mav_msg, &msg_length, &temp1, &temp2, &temp3);
				break;
			}
			last_send[send_index] = now;
			
			remain_byte = msg_length;
			sent_byte = 0;
		}
		
		remain_byte = msg_length - sent_byte;
		Mavlink_tx_nonblocking((mav_msg + sent_byte), &remain_byte);
		sent_byte = remain_byte;
	}
	else{
		now = get_millis();
		for(i = 0; i < MAVLINK_SEND_MSG_NUM; i ++) last_send[i] = now;
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
}

inline void loop_50hz(){

}

inline void loop_100hz(){

}
