#ifndef TASK_H_
#define TASK_H_

/**
 * @file
 * @brief task structure header
 */
 
#include <stdint.h>
/**
 * @struct task_t
 * @brief task data structure
 * @var task_t::tid
 * unique task id in scheduler task list
 * @var task_t::tname
 * task name unique for every task type
 * @var task_t::periodic
 * periodic task flag
 * @var task_t::periods
 * Number of perdiods
 * @var task_t::exec_time
 * task execution time in timer ticks
 * @var task_t::period_sec
 * task period in seconds
 * @var task_t::priority
 * task priority
 * @var task_t::task_execute
 * task start function pointer
 * @var task_t::data
 * task private data
 */
typedef struct task_t
{
	uint8_t tid;
	uint8_t tname;
	uint8_t periodic;
	uint8_t periods;
	uint32_t exec_time;
	uint32_t period_sec;
	uint8_t priority;
	void (*task_execute)(void* data);
	void* data;
} task_s, *task_p;

#endif
