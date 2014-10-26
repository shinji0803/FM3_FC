
#include "scheduler.h"

#include "timer.h"


/* Local Valiables */
static Task *_tasks;
static uint8_t _num_tasks;
static uint16_t *_last_run;
static uint16_t _tick_counter;

static uint32_t _task_time_allowed;
static uint32_t _task_time_started;

/* Total remain time when scheduler run */
static uint32_t _spare_micros;
/* Scheduler run count */
static uint8_t _spare_ticks;

/* Task Scheduler Status */
static uint32_t *_task_time_taken;
static uint16_t *_task_run_count;
static uint32_t _before_scheduler_run;

void scheduler_init(Task *tasks, uint8_t num_tasks)
{
	_tasks = tasks;
	_num_tasks = num_tasks;
	_last_run = malloc(sizeof(uint16_t) * _num_tasks);
	memset(_last_run, 0, sizeof(uint16_t) * _num_tasks);
	
	_task_time_taken = malloc(sizeof(uint32_t) * (_num_tasks + 1));
	memset(_task_time_taken, 0, sizeof(uint32_t) * (_num_tasks + 1));
	_task_run_count = malloc(sizeof(uint16_t) * (_num_tasks + 1));
	memset(_task_run_count, 0, sizeof(uint16_t) * (_num_tasks + 1));
	
	_tick_counter = 0;
}

void scheduler_tick(void)
{
	_tick_counter ++;
}

void scheduler_run(uint16_t time_available)
{
	uint32_t run_started_usec = get_micros();
	uint32_t now = run_started_usec;
	
	/* Main Task Time Taken Calculate */
	_task_time_taken[0] = now - _before_scheduler_run;
	_before_scheduler_run = now;
	
	uint8_t i;
	for(i = 0; i < _num_tasks; i ++){
		uint16_t dt = _tick_counter - _last_run[i];
		uint16_t interval_ticks = _tasks[i].interval_ticks;
		if(dt >= interval_ticks){
			_task_time_allowed = _tasks[i].max_time_micros;
			
			if(dt >= interval_ticks * 2){
				printf("Scheduler slip task[%u] (%u/%u/%u)\n", 
												(unsigned)i, 
												(unsigned)dt,
												(unsigned)interval_ticks,
												(unsigned)_task_time_allowed);
			}
			
			if(_task_time_allowed <= time_available){
				_task_time_started = now;
				void (*func)(void) = _tasks[i].function;
				func();
				
				_last_run[i] = _tick_counter;
				now = get_micros();
				
				uint32_t time_taken = now - _task_time_started;
				
				_task_time_taken[i + 1] += time_taken;
				_task_run_count[i + 1] ++;
				if(_task_run_count[i + 1] >= 32){
					_task_time_taken[i + 1] = _task_time_taken[i + 1] >> 2;
					_task_run_count[i + 1] = _task_run_count[i + 1] >> 2;
				}
				
				if(time_taken > _task_time_allowed){
					/* Current Task is Overran. Print Debug Message. */
					printf("Scheduler overrun task[%u] (%u/%u)\n", 
                                            (unsigned)i, 
                                            (unsigned)time_taken,
                                            (unsigned)_task_time_allowed);
					//printf("TaskStart: %lu TaskEnd: %lu\n", _task_time_started, now);
				}
				
				if(time_taken >= time_available){
					goto update_spare_ticks;
				}
				time_available -= time_taken;
			}
		}
	}
	_spare_micros += time_available;

update_spare_ticks:
	_spare_ticks ++;
	_task_run_count[0] = _spare_ticks;
	if(_spare_ticks == 32){
		_spare_ticks /= 2;
		_spare_micros /= 2;
		
		_task_run_count[0] = _task_run_count[0] >> 2;
	}
}

/* Return scheduler load */
float scheduler_load_average(uint32_t tick_time_usec)
{
	if(_spare_ticks == 0) return 0.0f;
	
	/* average spare time = _spare_micros / _spare_ticks */
	uint32_t used_time = tick_time_usec - (_spare_micros / _spare_ticks);
	return used_time / (float)tick_time_usec;
}

/* Print Time Taken of All Taks */
void scheduler_print_timetaken(void)
{
	uint8_t i;
	static uint32_t last_print = 0, last_call = 0;
	uint32_t now = get_millis();
	
	if(now - last_call >= 200) last_print = 0;
	
	if(now - last_print > 1000){
		
		if(last_print != 0){
			printf("\r");
			printf("\033[%dA" , (_num_tasks + 1));
		}
		last_print = now;
		
		printf("> Main: %5d\r\n", (uint16_t)_task_time_taken[0]);
		
		for(i = 1; i < (_num_tasks + 1); i ++){
			printf("> Task[%d]: %4d / %4d\r\n",
					i - 1, (uint16_t)(_task_time_taken[i] / _task_run_count[i]), _tasks[i - 1].max_time_micros);
		}
	}
	last_call = now;
}

