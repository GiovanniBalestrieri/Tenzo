#ifndef task_h
#define task_h

#include "sched.h"

// Task related
int num_tasks;
struct task taskset[MAX_NUM_TASKS];
void initTaskset();
void createTasks(int);                
int create_task(int id, unsigned long period,
		unsigned long phase,
                unsigned long prio_dead,
                int type,
                const char *name);               
                
#endif
