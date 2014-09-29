
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

uint8_t p_flg;

int32_t main(void){
	
	//Initialize start
	conio_init(57600UL);
	Init_timer();
	InitLED();
	
	rcin_enable(0); //RC In & Out port initialize
	rcout_enable(0);
	rcout_enable(1);
	rcout_enable(2);
	rcout_enable(3);
	
	Init_i2c();
	Init_fram();
	Init_DT();
	wait(100);
	
	printf("Initialize OK.\r\n");
	//Initialize complete
	
	console_init();
	
	while(1){
		
		console_run();
		
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

}

inline void loop_200hz(){

}

