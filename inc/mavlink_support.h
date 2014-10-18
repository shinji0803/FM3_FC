
#ifndef _MAVLINK_SUP_DEF_H_
#define _MAVLINK_SUP_DEF_H_

/* Support functions for Mavlink command */

#include "hw_config.h"
#include "parameters.h"

int32_t Mavlink_port_init(uint8_t ch, uint32_t baudrate);

int32_t Mavlink_tx(void *data, int32_t *size);

void Mavlink_heartbeat_send(void);
void Mavlink_rcin_raw_send(uint16_t *input);
void Mavlink_imu_raw_send(Vector3f *g, Vector3f *a, Vector3f *m);
void Mavlink_att_send(Vector3f *att, Vector3f *attSpeed);
void Mavlink_debug_send(uint8_t index, float value);
void Mavlink_debug_vect_send(uint8_t *name, Vector3f *v);

void Mavlink_rx_check(void);

void Mavlink_printf(const char* format, ...);

#endif