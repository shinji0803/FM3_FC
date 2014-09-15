
#include "mavlink_support.h"

static UartDev_IOB *mavlink;
mavlink_message_t msg;

static uint8_t mav_send_msg[MAVLINK_MAX_PACKET_LEN];

static const int system_type = MAV_TYPE_GENERIC;
static const int autopilot_type = MAV_AUTOPILOT_GENERIC;

int32_t Mavlink_port_init(uint8_t ch, uint32_t baudrate)
{
	int32_t result;
	
	// Port setting
	FM3_GPIO->PFR7 = FM3_GPIO->PFR7 | 0x0C;
	FM3_GPIO->EPFR07 = FM3_GPIO->EPFR07 | 0x40000;
	
	// Driver setting
	mavlink = &UartDev[ch];
	
	mavlink->Cfg.BufSize = 256;
	mavlink->Cfg.BaudRate = baudrate;
	mavlink->Cfg.BitOrder = UartDev_BITORDER_LSB;
	
	result = mavlink->Init();
	return result;
}

int32_t Mavlink_tx(void *data, int32_t *size)
{
	return mavlink->BufTx(data, size, UartDev_FLAG_BLOCKING);
}

void Mavlink_heartbeat_send(void)
{
	int32_t size;
	mavlink_msg_heartbeat_pack(100, 200, &msg, system_type, autopilot_type, 0, 0, 0);
			
	size = mavlink_msg_to_send_buffer(mav_send_msg, &msg);
	Mavlink_tx(mav_send_msg, &size);
}

void Mavlink_rcin_raw_send(uint16_t *input)
{
	int32_t size;
	mavlink_msg_rc_channels_raw_pack(100, 200, &msg, get_millis(), 0 , input[0], input[1], input[2],
												input[3], input[4], input[5], input[6], input[7],  255);
	
	size = mavlink_msg_to_send_buffer(mav_send_msg, &msg);
	Mavlink_tx(mav_send_msg, &size);
}

void Mavlink_imu_raw_send(Vector3f *a, Vector3f *g, Vector3f *m)
{
	int32_t size;
	mavlink_msg_raw_imu_pack(100, 200, &msg, get_micros(), 
								(int16_t)a->x	, (int16_t)a->y, (int16_t)a->z,
								(int16_t)g->x, (int16_t)g->y, (int16_t)g->z, 
								(int16_t)m->x, (int16_t)m->y, (int16_t)m->z);
	
	size = mavlink_msg_to_send_buffer(mav_send_msg, &msg);
	Mavlink_tx(mav_send_msg, &size);
}

void Mavlink_att_send(Vector3f *att, Vector3f *attSpeed)
{
	int32_t size;
	mavlink_msg_attitude_pack(100, 200, &msg, get_millis(), 
								att->x, att->y, att->z,
								attSpeed->x, attSpeed->y, attSpeed->z);

	size = mavlink_msg_to_send_buffer(mav_send_msg, &msg);
	Mavlink_tx(mav_send_msg, &size);
}

void Mavlink_debug_vect_send(uint8_t *name, Vector3f *v)
{
	int32_t size;
	mavlink_msg_debug_vect_pack(100, 200, &msg, (const char *)name, get_micros(), v->x, v->y, v->z);
	size = mavlink_msg_to_send_buffer(mav_send_msg, &msg);
	Mavlink_tx(mav_send_msg, &size);
}

void Mavlink_debug_send(uint8_t index, float value)
{
	int32_t size;
	mavlink_msg_debug_pack(100, 200, &msg, get_millis(), index, value);
	size = mavlink_msg_to_send_buffer(mav_send_msg, &msg);
	Mavlink_tx(mav_send_msg, &size);
}

void Mavlink_rx_check(void)
{
	int32_t size;
	uint8_t receive_buf[8];
	
	if(mavlink->RxAvailable() > 0){
		size = 1;
		mavlink->BufRx(receive_buf, &size, UartDev_FLAG_NONBLOCKING);
		
		/*
		if(size == 1){
			if(receive_buf[0] == MAVLINK_STX) printf("\r\n");
			printf("%2x ", receive_buf[0]);
		}
		*/
	}
}


void Mavlink_printf(const char* format, ...)
{
	va_list args;
	uint8_t convert[256];
	int32_t size = 0;
		
	va_start( args, format);
	vsprintf((char *)convert, format, args);
	va_end(args);
	
	size = strlen((char *)convert);
	mavlink->BufTx(convert, &size, UartDev_FLAG_BLOCKING);
}
