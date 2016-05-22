#include "task.h"
#include "Arduino.h"

void initTaskset()
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
        Serial.println("[ Ok ] TaskSet initialized");
}


int create_task(int id, unsigned long period,
		unsigned long phase,
                unsigned long prio_dead,
                int type,
                const char *name)
{
	int i;
	struct task *t;
	for (i = 1; i < MAX_NUM_TASKS; ++i)	/* skip task 0 (idle task) */
		if (!taskset[i].valid)
			break;
	if (i == MAX_NUM_TASKS)
		return -1;
	t = taskset + i;
	//t->job = job;
	//t->arg = (arg == NULL ? t : arg);
        t->id = id;
	t->name = name;
	t->period = period;
	t->releasetime = ticks + phase;
	if (type == EDF) {
		/* this is an EDF task
		 * priority is set to the absolute deadline of the first job
		 * a small absolute deadline yields a large priority */
		if (prio_dead == 0)
			return -1;
		t->priority = prio_dead + t->releasetime;
		t->deadline = prio_dead;
	} else {
		/* this is a fixed-priority task
		 * to be run in background if no other EDF job is pending */
		t->priority = prio_dead;
		t->deadline = 0;
	}
	t->released = 0;
//	init_task_context(t, i);

	cli();
	++num_tasks;
	t->valid = 1;
	sei();
	return i;
}



