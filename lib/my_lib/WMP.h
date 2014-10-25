
#ifndef WMP_H
#define WMP_H

/* Wii Motion Plus */

#include "parameters.h"

#define WMP_CONF_ADD 0x53
#define WMP_ADD 0x52
#define WMP_BAUD 400000

void WMP_init(void);

void WMP_update(void);

void WMP_get_raw_gyro(Vector3f *g);

void WMP_get_gyro_mode(uint8_t *mode);

#endif
