
#include "console.h"

static Vector3f att, omega;
static uint16_t rcInput[8];
static uint8_t menu_flg = 0, input = 0, input_detect = 0;
static axis_t axis = ROLL;

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


/* Command type */
static const Menu top_menu_content[] = {
	{'a', "read AHRS", NULL},
	{'r', "read Radio", NULL},
	{'c', "set Attitude Gain", NULL},
	{'g', "set AHRS Gain", NULL} 
};
static const uint8_t top_menu_content_size = sizeof(top_menu_content) / sizeof(top_menu_content[0]);

static const Menu att_gain_menu_content[] = {
	{'1', "set Roll Gain", NULL},
	{'2', "set Pitch Gain", NULL},
	{'3', "set Yaw Gain", NULL},
	{'4', "set Roll Rate Gain", NULL},
	{'5', "set Pitch Rate Gain", NULL},
	{'6', "set Yaw Rate Gain", NULL},
	{'e', "exit", NULL}
};

static const Menu att_axis_gain_menu_content[] = {
	{'d', "display All Gain", NULL},
	{'1', "set P_Gain", NULL},
	{'2', "set I_Gain", NULL},
	{'3', "set D_Gain", NULL},
	{'s', "save all gain setting", NULL},
	{'r', "restore parameters", NULL},
	{'e', "exit", NULL}
};
static const uint8_t att_axis_gain_menu_content_size = sizeof(att_axis_gain_menu_content) / sizeof(att_axis_gain_menu_content[0]);

static const Menu ahrs_gain_menu_content[] = {
	{'d', "display All Gain", NULL},
	{'1', "set AHRS RollPitch P_Gain", NULL},
	{'2', "set AHRS RollPitch I_Gain", NULL},
	{'3', "set AHRS Yaw P_Gain", NULL},
	{'4', "set AHRS Yaw I_Gain", NULL},
	{'s', "save all gain setting", NULL},
	{'r', "restore parameters", NULL},
	{'e', "exit", NULL}
};
static const uint8_t ahrs_gain_menu_content_size = sizeof(ahrs_gain_menu_content) / sizeof(ahrs_gain_menu_content[0]);


void console_init(void)
{
	top_menu();
}

/* Console control function */
void console_run(void)
{
	uint8_t old_menu_flg;
	
	if(conio_available() > 0){
		input = getch();
		input_detect = 1;
	}
	
	if(input_detect == 1){
		old_menu_flg = menu_flg;
		
		switch(menu_flg){
			case 0: // Top menu
			if(input == '\n'){
				input_detect = 0;
				top_menu();
			}
			else if(input != '\r')	top_menu_branch(input);
			break;
			
			case 1: // Attitude Gain Set Menu
			input_detect = 0;
			if(input == '\n') att_ctrl_gain_menu();
			if(input != '\r' && input != '\n') att_gain_menu_branch(input);	
			break;
			
			case 2: // AHRS Gain Set Menu
			input_detect = 0;
			if(input == '\n') ahrs_gain_menu();
			if(input != '\r' && input != '\n') ahrs_gain_menu_branch(input);	
			break;
			
			case 3: // Attitude Gain (Only one axis) Set Menu
			input_detect = 0;
			if(input == '\n') att_ctrl_axis_gain_menu();
			if(input != '\r' && input != '\n') att_axis_gain_menu_branch(input);	
			break;
		}
		
		if(menu_flg != old_menu_flg){
			switch(menu_flg){
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
}

static void top_menu(void)
{
	uint8_t i;
	printf("-----------------------------\r\n");
	printf("----  CLI + AHRS ver3.0  ----\r\n");
	printf("-----------------------------\r\n");
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
	
	switch(axis){
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
	uint8_t i = 0;
	
	if(p_flg == 1){
	/*
		while(i < (top_menu_content_size + 1)){
			if(top_menu[i].command == command) break;
			i ++;
		}
	*/	
	
		switch(command){
			case 'r':
			rc_multiread(rcInput);
			printf("IN: %d, %d, %d, %d, %d\r\n", rcInput[0], rcInput[1], rcInput[2], rcInput[3], rcInput[4]);
			break;
			
			case 'a':
			AHRS_get_euler(&att);
			AHRS_get_omega(&omega);
			printf("ATT: %+f, %+f, %+f  Omega: %+f, %+f, %+f\r\n", att.x, att.y, att.z, omega.x, omega.y, omega.z);
			break;
			
			case 'c':
			input_detect = 0;
			menu_flg = 1;
			break;
			
			case 'g':
			input_detect = 0;
			menu_flg = 2;
			break;
			
			default:
			printf("invalid command\r\n");
			input_detect = 0;
			break;
		}
		p_flg = 0;
	}
}

static void att_gain_menu_branch(uint8_t command)
{
	switch(command){
		case '1':
		menu_flg = 3;
		axis = ROLL;
		break;
		
		case '2':
		menu_flg = 3;
		axis = PITCH;
		break;
		
		case '3':
		menu_flg = 3;
		axis = YAW;
		break;
		
		case 'e':
		menu_flg = 0;
		break;
		
		default:
		printf("invalid command\r\n");
		break;
	}
}

static void att_axis_gain_menu_branch(uint8_t command)
{
	Gain g = { 0, 0, 0, 24, 28, 32};
	switch(command){
		case 'd':
		printf("P: %f I: %f D: %f\r\n", g.p_gain, g.i_gain, g.d_gain);
		break;
		
		case '1':
		printf("P Gain is %f, input new value.\r\n", g.p_gain);
		g.p_gain = get_float_input();
		break;
		
		case '2':
		printf("I Gain is %f, input new value.\r\n", g.i_gain);
		g.i_gain = get_float_input();
		break;
		
		case '3':
		printf("D Gain is %f, input new value.\r\n", g.d_gain);
		g.d_gain = get_float_input();
		break;
		
		case 's':
		write_float(g.p_add, g.p_gain);
		write_float(g.i_add, g.i_gain);
		write_float(g.d_add, g.d_gain);
		printf("Saved!\r\n");
		break;
		
		case 'r':
		g.p_gain = read_float(g.p_add);
		g.i_gain = read_float(g.i_add);
		g.d_gain = read_float(g.d_add);
		printf("Restore Parameters.\r\n");
		break;
		
		case 'e':
		menu_flg = 1;
		break;
		
		default:
		printf("invalid command\r\n");
		break;
	}
}

static void ahrs_gain_menu_branch(uint8_t command)
{
	Gain rp, y;
	AHRS_get_gain(&rp, &y);
	
	switch(command){
		case 'd':
		printf("R&P: %f, %f, Y: %f, %f\r\n", rp.p_gain, rp.i_gain, y.p_gain, y.i_gain);
		break;
		
		case '1':
		printf("R&P P Gain is %f, input new value.\r\n", rp.p_gain);
		rp.p_gain = get_float_input();
		break;
		
		case '2':
		printf("R&P I Gain is %f, input new value.\r\n", rp.i_gain);
		rp.i_gain = get_float_input();
		break;
		
		case '3':
		printf("Yaw P Gain is %f, input new value.\r\n", y.p_gain);
		y.p_gain = get_float_input();
		break;
		
		case '4':
		printf("Yaw I Gain is %f, input new value.\r\n", y.i_gain);
		y.i_gain = get_float_input();
		break;
		
		case 's':
		write_float(RP_P_ADD, rp.p_gain);
		write_float(RP_I_ADD, rp.i_gain);
		write_float(Y_P_ADD, y.p_gain);
		write_float(Y_I_ADD, y.i_gain);
		printf("Saved!\r\n");
		break;
		
		case 'r':
		rp.p_gain = read_float(RP_P_ADD);
		rp.i_gain = read_float(RP_I_ADD);
		y.p_gain = read_float(Y_P_ADD);
		y.i_gain = read_float(Y_I_ADD);
		printf("Restore Parameters.\r\n");
		break;
		
		case 'e':
		menu_flg = 0;
		break;
		
		default:
		printf("invalid command\r\n");
		break;
	}
	AHRS_set_gain(&rp, &y);
}

