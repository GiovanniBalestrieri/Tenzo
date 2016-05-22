#include "task.h"

void init_taskset()
{
	int i;
	num_tasks = 0;
	for (i = 0; i < MAX_NUM_TASKS; ++i)
		taskset[i].valid = 0;

	/* Task 0 is special: it is the idle (or kernel) task,
	 * which runs whenever no other job is runnable.
	 * Actually, taskset[0] is only used as an address
	 * placeholder to be assigned to the current variable */

	current = &taskset[0];
}


