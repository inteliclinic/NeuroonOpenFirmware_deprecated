#ifndef SCHEDULER_H_
#define SCHEDULER_H_

/**
 * @file
 * @brief tasks scheduler header
 */
#include "ic_lib_config.h"
#include "ic_task.h"

typedef struct tm * date_p;

void scheduler_set_systime(uint64_t timestamp);
uint64_t scheduler_get_systime();

void scheduler_init(uint32_t prescaler);
void scheduler_start();
void scheduler_clear_tasks();
void scheduler_clear_current();
void scheduler_end_current_task();
void scheduler_execute_task();
void scheduler_register_task(task_p task);
void scheduler_remove_task(task_p task);
uint8_t scheduler_is_current_task_ready();
void scheduler_change_task_time(task_p task, uint32_t time);

#endif
