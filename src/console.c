
#include "console.h"

static Vector3f att, omega;
static uint16_t rcInput[8];

void top_menu(void)
{
	printf("-----------------------------\r\n");
	printf("----  CLI + AHRS ver2.0  ----\r\n");
	printf("-----------------------------\r\n");
	printf("a: read AHRS\r\n");
	printf("r: read radio\r\n");
	//printf("f: read flow\r\n");
	printf("g: set gain\r\n");
	printf("-----------------------------\r\n");
}	

void gain_menu(void)
{
	printf("------Gain Setting Menu------\r\n");
	printf("d: display all gain\r\n");
	printf("1: set AHRS RollPitch P_Gain\r\n");
	printf("2: set AHRS RollPitch I_Gain\r\n");
	printf("3: set AHRS Yaw P_Gain\r\n");
	printf("4: set AHRS Yaw I_Gain\r\n");
	printf("s: save all gain setting\r\n");
	printf("r: restore parameters\r\n");
	printf("e: return top menu\r\n");
	printf("-----------------------------\r\n");
}

void top_menu_branch(uint8_t com_type)
{
	if(p_flg == 1){
		switch(com_type){
			case RADIO:
			rc_multiread(rcInput);
			printf("IN: %d, %d, %d, %d, %d\r\n", rcInput[0], rcInput[1], rcInput[2], rcInput[3], rcInput[4]);
			break;
			
			case AHRS:
			AHRS_get_euler(&att);
			AHRS_get_omega(&omega);
			printf("ATT: %+f, %+f, %+f  Omega: %+f, %+f, %+f\r\n", att.x, att.y, att.z, omega.x, omega.y, omega.z);
			break;
			
			case SET_GAIN:
			input_detect = 0;
			menu_flg = 1;
			gain_menu();
			break;
			
			default:
			printf("invalid command\r\n");
			input_detect = 0;
			break;
		}
		p_flg = 0;
	}
}

void gain_menu_branch(uint8_t com_type)
{
	gain rp, y;
	AHRS_get_gain(&rp, &y);
	
	switch(com_type){
		case DISPLAY:
		printf("R&P: %f, %f, Y: %f, %f\r\n", rp.p_gain, rp.i_gain, y.p_gain, y.i_gain);
		break;
		
		case SET_RP_P:
		printf("R&P P Gain is %f, input new value.\r\n", rp.p_gain);
		rp.p_gain = get_float_input();
		break;
		
		case SET_RP_I:
		printf("R&P I Gain is %f, input new value.\r\n", rp.i_gain);
		rp.i_gain = get_float_input();
		break;
		
		case SET_Y_P:
		printf("Yaw P Gain is %f, input new value.\r\n", y.p_gain);
		y.p_gain = get_float_input();
		break;
		
		case SET_Y_I:
		printf("Yaw I Gain is %f, input new value.\r\n", y.i_gain);
		y.i_gain = get_float_input();
		break;
		
		case SAVE:
		write_float(RP_P_ADD, rp.p_gain);
		write_float(RP_I_ADD, rp.i_gain);
		write_float(Y_P_ADD, y.p_gain);
		write_float(Y_I_ADD, y.i_gain);
		printf("Saved!\r\n");
		break;
		
		case RESTORE:
		rp.p_gain = read_float(RP_P_ADD);
		rp.i_gain = read_float(RP_I_ADD);
		y.p_gain = read_float(Y_P_ADD);
		y.i_gain = read_float(Y_I_ADD);
		printf("Restore Parameters.\r\n");
		break;
		
		case EXIT:
		menu_flg = 0;
		input_detect = 0;
		top_menu();
		break;
		
		default:
		printf("invalid command\r\n");
		input_detect = 0;
		break;
	}
	AHRS_set_gain(&rp, &y);
}

