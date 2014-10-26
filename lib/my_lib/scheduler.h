
#ifndef SCHEDULER_H
#define SCHEDULER_H

/* Task Scheduler */

#include "hw_config.h"

typedef struct __task {
	void (*function)(void);
	uint16_t interval_ticks;
	uint16_t max_time_micros;
} Task;

/* Initialize task scheduler */
void scheduler_init(Task *tasks, uint8_t num_tasks);

/* Pass one tick function */
void scheduler_tick(void);

/* Run the task */
void scheduler_run(uint16_t time_available);

/* Get load average. Range is 0.0 ~ 1.0 */
float scheduler_load_average(uint32_t tick_time_usec);

/* Print Time Taken of All Tasks */
void scheduler_print_timetaken(void);
#endif