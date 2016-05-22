#ifndef task_h
#define task_h

#include "sched.h"

struct task taskset[MAX_NUM_TASKS];

int num_tasks;

void init_taskset();

#endif
