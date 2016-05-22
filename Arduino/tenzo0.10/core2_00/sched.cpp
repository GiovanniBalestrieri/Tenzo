#include "sched.h"
#include "CommunicationUtils.h"

struct task *current;

volatile unsigned long globalreleases = 0;	/* used as a status changed flag */

volatile unsigned long trigger_schedule = 0;	/* force rescheduling */

void check_periodic_tasks(void)
{
	unsigned long now = ticks;
	struct task *f;
	int i;

	for (i = 0, f = taskset + 1; i < num_tasks; ++f) {	/* skip task 0 (idle task) */
		if (f - taskset >= MAX_NUM_TASKS)
			panic(0);	/* Should never happen */
		if (!f->valid)
			continue;
		if (now >= f->releasetime) { // se è già stato rilasciato
			f->releasetime += f->period;
			++f->released;
			trigger_schedule = 1;	/* force scheduler invocation */
			++globalreleases;
		}
		++i;
	}
}

struct task *select_best_task(void)
{
	unsigned long maxprio;
	int i, edf;
	struct task *best, *f;

	maxprio = 100;
	edf = 0;
	best = &taskset[0];
	for (i = 0, f = taskset + 1; i < num_tasks; ++f) { // Salta task 0 (idle)
		if (f - taskset >= MAX_NUM_TASKS)
			panic(0);	/* Should never happen */
		if (!f->valid)
			continue;
		++i;
		if (f->released == 0)
			continue;
		if (edf) {
			/* fixed-priority tasks have lower priority than EDF ones */
			if (f->deadline == 0)
				continue;
			/* priority in EDF tasks is basically a time instant */
			if (f->priority <= maxprio) {
				maxprio = f->priority;
				best = f;
			}
			continue;
		} 
		if (f->deadline != 0) {
			edf = 1;
			maxprio = f->priority;
			best = f;
			continue;
		}
		if (f->priority < maxprio) {
			maxprio = f->priority;
			best = f;
		}
	}
	return best;
}

struct task *schedule(void)
{
	static int do_not_enter = 0;
	struct task *best;
	unsigned long oldreleases;
	//irq_disable();
        sei();
	if (do_not_enter != 0) {
		//irq_enable();
		cli();
		return NULL;
	}
	do_not_enter = 1;
	do {
		oldreleases = globalreleases;
		//irq_enable();
                sei();
		best = select_best_task();
                cli();		
                //irq_disable();
	} while (oldreleases != globalreleases);
	trigger_schedule = 0;
	best = (best != current ? best : NULL);
	do_not_enter = 0;
	cli();
        //irq_enable();
	return best;
}

