
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

static void InitLED(void);
inline void loop_1hz(void);
inline void loop_20hz(void);
inline void loop_50hz(void);
inline void loop_100hz(void);

static const Task scheduler_tasks[] = {
	{ console_run, 5, 1500}
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

/* Most Fast Loop */
/* This funtion run as fast as possible. */
/* Run frequency is change by other tasks. But this function is run at 100hz */
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
	
	set_debug_msg("AHRS_Rate = %d, %d", dt, (now - main_loop_timer));
	main_loop_timer = now;
	Start_DT2();
	
	AHRS_dcm_normalize();
	AHRS_drift_correction();
	
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
