
#include "console.h"

#include <stdarg.h>

#include "uart_support.h"
#include "mavlink_support.h"
#include "scheduler.h"
#include "parameters.h"
#include "rc_io.h"
#include "AHRS.h"
#include "myMath.h"

static uint8_t _menu_flg = 0, _input = 0, _input_detect = 0, _start_input = 0;
static void *inputValiable = NULL;

static Vector3f _att, _omega;
static uint16_t _rcInput[8];

static axis_t _axis = ROLL;

/* Debug message */
static uint8_t _debug_msg[64];

/* Local gain */
float _kp, _ki, _kp_yaw, _ki_yaw;
float _att_p = 0, _att_i = 0, _att_d = 0;

/* Menu print function */
static void top_menu(void);
static void ahrs_gain_menu(void);
static void att_ctrl_gain_menu(void);
static void att_ctrl_axis_gain_menu(void);

/* Menu brunch control function */
static void top_menu_branch(uint8_t command);
static void att_gain_menu_branch(uint8_t command);
static void att_axis_gain_menu_branch(uint8_t command);
static void ahrs_gain_menu_branch(uint8_t command);

/* Capture user input function */
static uint8_t get_float_input(uint8_t inputChar);


/* Command type */
static const Menu top_menu_content[] = {
	{'a', "read AHRS", 				NULL},
	{'r', "read Radio", 			NULL},
	{'c', "set Attitude Gain",		NULL},
	{'g', "set AHRS Gain",			NULL},
	{'l', "Print CPU Load",			NULL},
	{'d', "Print Debug Message",	NULL},
	{'t', "Task Scheduler Status",	NULL},
	{'m', "Switch Mavlink Output",	NULL}
};
static const uint8_t top_menu_content_size = sizeof(top_menu_content) / sizeof(top_menu_content[0]);

static const Menu att_gain_menu_content[] = {
	{'1', "set Roll Gain",			NULL},
	{'2', "set Pitch Gain",			NULL},
	{'3', "set Yaw Gain",			NULL},
	{'4', "set Roll Rate Gain",		NULL},
	{'5', "set Pitch Rate Gain",	NULL},
	{'6', "set Yaw Rate Gain",		NULL},
	{'e', "exit",					NULL}
};

static const Menu att_axis_gain_menu_content[] = {
	{'d', "display All Gain",		NULL},
	{'1', "set P_Gain",				NULL},
	{'2', "set I_Gain",				NULL},
	{'3', "set D_Gain",				NULL},
	{'s', "save all gain setting",	NULL},
	{'r', "restore parameters",		NULL},
	{'e', "exit",					NULL}
};
static const uint8_t att_axis_gain_menu_content_size = sizeof(att_axis_gain_menu_content) / sizeof(att_axis_gain_menu_content[0]);

static const Menu ahrs_gain_menu_content[] = {
	{'d', "display All Gain",			NULL},
	{'1', "set AHRS RollPitch P_Gain",	NULL},
	{'2', "set AHRS RollPitch I_Gain",	NULL},
	{'3', "set AHRS Yaw P_Gain",		NULL},
	{'4', "set AHRS Yaw I_Gain",		NULL},
	{'s', "save all gain setting",		NULL},
	{'r', "restore parameters",			NULL},
	{'e', "exit",						NULL}
};
static const uint8_t ahrs_gain_menu_content_size = sizeof(ahrs_gain_menu_content) / sizeof(ahrs_gain_menu_content[0]);


void console_init(void)
{
	top_menu();
	/* Initial AHRS Gain load */
	AHRS_get_gain(&_kp, &_ki, &_kp_yaw, &_ki_yaw);
}

/* Console control function */
void console_run(void)
{
	uint8_t old_menu_flg;
	
	if(inputValiable != NULL) _start_input = 1;
	
	if(conio_available() > 0){
		_input = getch();
		_input_detect = 1;
	}
	
	if(_input_detect == 1 && _start_input == 0){
		old_menu_flg = _menu_flg;
		
		switch(_menu_flg){
			case 0: // Top menu
			if(_input == '\n'){
				_input_detect = 0;
				top_menu();
			}
			else if(_input != '\r')	top_menu_branch(_input);
			break;
			
			case 1: // Attitude Gain Set Menu
			_input_detect = 0;
			if(_input == '\n'){
				_input_detect = 0;
				att_ctrl_gain_menu();
			}
			if(_input != '\r' && _input != '\n') att_gain_menu_branch(_input);	
			break;
			
			case 2: // AHRS Gain Set Menu
			_input_detect = 0;
			if(_input == '\n'){
				_input_detect = 0;
				ahrs_gain_menu();
			}
			if(_input != '\r' && _input != '\n') ahrs_gain_menu_branch(_input);	
			break;
			
			case 3: // Attitude Gain (Only one axis) Set Menu
			_input_detect = 0;
			if(_input == '\n'){
				att_ctrl_axis_gain_menu();
				_input_detect = 0;
			}
			if(_input != '\r' && _input != '\n') att_axis_gain_menu_branch(_input);	
			break;
		}
		
		/* Print menu command when menu changed */
		if(_menu_flg != old_menu_flg){
			switch(_menu_flg){
				case 0:
				top_menu();
				break;
				case 1:
				att_ctrl_gain_menu();
				break;
				case 2:
				ahrs_gain_menu();
				break;
				case 3:
				att_ctrl_axis_gain_menu();
				break;
			}
		}
	}
	
	/* Get value from user input */
	if(_start_input == 1 && _input != 0){
		_start_input = get_float_input(_input);
		_input = 0;
	}
}

static void top_menu(void)
{
	uint8_t i;
	printf("----  CLI + AHRS ver4.0  ----\r\n");
	for( i = 0; i < top_menu_content_size; i ++)
		printf("%c: %s\r\n", top_menu_content[i].command, top_menu_content[i].brief);
	printf("-----------------------------\r\n");
}	

static void ahrs_gain_menu(void)
{
	uint8_t i;
	printf("------AHRS Gain Setting------\r\n");
	for( i = 0; i < ahrs_gain_menu_content_size; i ++)
		printf("%c: %s\r\n", ahrs_gain_menu_content[i].command, ahrs_gain_menu_content[i].brief);
	printf("-----------------------------\r\n");
}

static void att_ctrl_gain_menu(void)
{
	uint8_t i;
	printf("--- Attitude Gain Setting ---\r\n");
	for( i = 0; i < (3 + 3 + 1); i ++)
		printf("%c: %s\r\n", att_gain_menu_content[i].command, att_gain_menu_content[i].brief);
	printf("-----------------------------\r\n");
}

static void att_ctrl_axis_gain_menu(void)
{
	uint8_t axisname[16];
	int i;
	
	switch(_axis){
		case ROLL:
		sprintf((char *)axisname, "Roll");
		break;
		case PITCH:
		sprintf((char *)axisname, "Pitch");
		break;
		case YAW:
		sprintf((char *)axisname, "Yaw");
		break;
		case RATE_ROLL:
		sprintf((char *)axisname, "Roll Rate");
		break;
		case RATE_PITCH:
		sprintf((char *)axisname, "Pitch Rate");
		break;
		case RATE_YAW:
		sprintf((char *)axisname, "Yaw Rate");
		break;
	}
		
	printf("--- Attitude %s Gain Setting ---\r\n", axisname);
	for( i = 0; i < att_axis_gain_menu_content_size; i ++)
		printf("%c: %s\r\n", att_axis_gain_menu_content[i].command, att_axis_gain_menu_content[i].brief);
	printf("-----------------------------\r\n");
}

static void top_menu_branch(uint8_t command)
{
	
	if(time.print == 1){
		switch(command){
			case 'r':
			rc_multiread(_rcInput);
			printf("IN: %4d, %4d, %4d, %4d, %4d\r\n", _rcInput[0], _rcInput[1], _rcInput[2], _rcInput[3], _rcInput[4]);
			break;
			
			case 'a':
			AHRS_get_euler(&_att);
			AHRS_get_omega(&_omega);
			printf("ATT: %+5.2f, %+5.2f, %+5.2f  Omega: %+6.2f, %+6.2f, %+6.2f\r\n",
						ToDeg(_att.x), ToDeg(_att.y), ToDeg(_att.z), 
						ToDeg(_omega.x), ToDeg(_omega.y), ToDeg(_omega.z));
			break;
			
			case 'c':
			_input_detect = 0;
			_menu_flg = 1;
			break;
			
			case 'g':
			_input_detect = 0;
			_menu_flg = 2;
			break;
			
			case 'd':
			if(_debug_msg == NULL) printf("> No debug data set\n");
			else{
				printf("> Debug: ");
				printf("%s\r\n", _debug_msg);
			}
			break;
			
			case 'l':
			printf("> SchedulerLoad: %4.2f\r\n", scheduler_load_average(10000) * 100.f);
			break;
			
			case 'm':
			Mavlink_enabled_switch();
			_input_detect = 0;
			break;
			
			case 't':
			scheduler_print_timetaken();
			break;
			
			default:
			printf("> invalid command\r\n");
			_input_detect = 0;
			break;
		}
		time.print = 0;
	}
}

static void att_gain_menu_branch(uint8_t command)
{
	switch(command){
		case '1':
		_menu_flg = 3;
		_axis = ROLL;
		break;
		
		case '2':
		_menu_flg = 3;
		_axis = PITCH;
		break;
		
		case '3':
		_menu_flg = 3;
		_axis = YAW;
		break;
		
		case 'e':
		_menu_flg = 0;
		break;
		
		default:
		printf("> invalid command\r\n");
		break;
	}
}

static void att_axis_gain_menu_branch(uint8_t command)
{
	/* Add get gain function */
	
	switch(command){
		case 'd':
		printf("> P: %f I: %f D: %f\r\n", _att_p, _att_i, _att_d);
		break;
		
		case '1':
		printf("> P Gain is %f, input new value.\r\n", _att_p);
		inputValiable = &_att_p;
		break;
		
		case '2':
		printf("> I Gain is %f, input new value.\r\n", _att_i);
		inputValiable = &_att_i;
		break;
		
		case '3':
		printf("> D Gain is %f, input new value.\r\n", _att_d);
		inputValiable = &_att_d;
		break;
		
		case 's':
		/* not implemtented */
		printf("> Not implemtented!\r\n");
		break;
		
		case 'r':
		/* not implemtented */
		printf("> Not implemtented!\r\n");
		break;
		
		case 'e':
		_menu_flg = 1;
		break;
		
		default:
		printf("> invalid command\r\n");
		break;
	}
	_input = 0;
}

static void ahrs_gain_menu_branch(uint8_t command)
{
	
	switch(command){
		case 'd':
		printf("> RollPitch: %f, %f, Yaw: %f, %f\r\n", _kp, _ki, _kp_yaw, _ki_yaw);
		break;
		
		case '1':
		printf("> R&P P Gain is %f, input new value.\r\n", _kp);
		inputValiable = &_kp;
		break;
		
		case '2':
		printf("> R&P I Gain is %f, input new value.\r\n", _ki);
		inputValiable = &_ki;
		break;
		
		case '3':
		printf("> Yaw P Gain is %f, input new value.\r\n", _kp_yaw);
		inputValiable = &_kp_yaw;
		break;
		
		case '4':
		printf("> Yaw I Gain is %f, input new value.\r\n", _ki_yaw);
		inputValiable = &_ki_yaw;
		break;
		
		case 's':
		storage_set_param(AHRS_ROLLPITCH_P, _kp);
		storage_set_param(AHRS_ROLLPITCH_I, _ki);
		storage_set_param(AHRS_YAW_P, _kp_yaw);
		storage_set_param(AHRS_YAW_I, _ki_yaw);
		printf("> Saved!\r\n");
		break;
		
		case 'r':
		_kp = storage_get_param(AHRS_ROLLPITCH_P);
		_ki = storage_get_param(AHRS_ROLLPITCH_I);
		_kp_yaw = storage_get_param(AHRS_YAW_P);
		_ki_yaw = storage_get_param(AHRS_YAW_I);
		printf("> Restore Parameters.\r\n");
		break;
		
		case 'e':
		_menu_flg = 0;
		break;
		
		default:
		printf("> Invalid command\r\n");
		break;
	}
	AHRS_set_gain(_kp, _ki, _kp_yaw, _ki_yaw);
	_input = 0;
}

/* Get user input as float value */
/* Return input start or end status */
static uint8_t get_float_input(uint8_t inputChar)
{
	static uint8_t inputStr[32];
	static uint8_t index = 0;
	
	if(inputValiable == NULL){
		printf("> inputValiable is NULL\r\n");
		return 0;
	}
	
	switch(inputChar){
		case '\b':
			if(index > 0){
				inputStr[index - 1] = 0;
				index --;
				putch('\b');
				putch(' ');
				putch('\b');
			}
			return 1; // Input continuous
			
		//case '\r':
		case '\n':
			putch('\r');
			putch('\n');
			*(float *)inputValiable = (float)atof((const char *)inputStr);
			printf("> Input is %f\n", *(float *)inputValiable);
			
			/* Valiables Initialize */
			inputValiable = NULL;
			memset(inputStr, 0, sizeof(inputStr));
			index = 0;
			_input_detect = 0;
			return 0; // Input end
			
		default:
			putch(inputChar);
			inputStr[index] = inputChar;
			index ++;
			return 1; // Input continuous
	}
}

void set_debug_msg(const char* format, ...)
{
	va_list args;
	
	va_start( args, format);
	vsprintf((char *)_debug_msg, format, args);
	va_end(args);
	
}


