
#ifndef _MAVLINK_SUP_DEF_H_
#define _MAVLINK_SUP_DEF_H_

/* Support functions for Mavlink command */

#include "hw_config.h"
#include "parameters.h"

int32_t Mavlink_port_init(uint8_t ch, uint32_t baudrate);

/* Mavlink Message Send Function */
int32_t Mavlink_tx(void *data, int32_t *size); // Control stop until all data send
int32_t Mavlink_tx_nonblocking(void *data, int32_t *size);

/* Mavlink Message send by UART NON BLOCKING */
void Mavlink_heartbeat_send(void);
void Mavlink_rcin_raw_send(uint16_t *input);
void Mavlink_imu_raw_send(Vector3f *g, Vector3f *a, Vector3f *m);
void Mavlink_att_send(Vector3f *att, Vector3f *attSpeed);
void Mavlink_debug_send(uint8_t index, float value);
void Mavlink_debug_vect_send(uint8_t *name, Vector3f *v);

/* Get Mavlink Message Function */
void Mavlink_get_heartbeat_msg(uint8_t *mav_msg, int32_t *size);
void Mavlink_get_imu_raw_msg(uint8_t *mav_msg, int32_t *size, Vector3f *a, Vector3f *g, Vector3f *m);
void Mavlink_get_att_msg(uint8_t *mav_msg, int32_t *size, Vector3f *att, Vector3f *attSpeed);

/* Dummy Read Function */ 
void Mavlink_rx_check(void);

void Mavlink_printf(const char* format, ...);

#endif