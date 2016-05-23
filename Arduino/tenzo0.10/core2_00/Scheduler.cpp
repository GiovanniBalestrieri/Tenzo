#include "Scheduler.h"
//#include "Task.h"

struct task *current;

volatile unsigned long globalreleases = 0;	/* used as a status changed flag */

volatile unsigned long trigger_schedule = 0;	/* force rescheduling */


Scheduler::Scheduler(int sizeT)
{
  taskset = new task[sizeT];
  MAX_NUM_TASKS = sizeT;
}



  
void Scheduler::initTaskset()
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
  Serial.println("[ Ok ] Taskset initialized");
}


  
int Scheduler::create_task(int id, 
            unsigned long period,
            unsigned long phase,
                    unsigned long prio_dead,
                    int type,
                    const char *name)
  
{
  /*
  int i;
  struct task *t;
  for (i = 1; i < MAX_NUM_TASKS; ++i) // skip task 0 (idle task) 
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
    /*
    if (prio_dead == 0)
      return -1;
    t->priority = prio_dead + t->releasetime;
    t->deadline = prio_dead;
  } else {
    /* this is a fixed-priority task
     * to be run in background if no other EDF job is pending */
     /*
    t->priority = prio_dead;
    t->deadline = 0;
  }
  t->released = 0;
//  init_task_context(t, i);

  cli();
  ++num_tasks;
  t->valid = 1;
  sei();
  return i;
*/
} 



void checkPeriodicTasks(void)
{
  /*
	unsigned long now = ticks;
	struct task *f;
	int i;

	for (i = 0, f = taskset + 1; i < task1.num_tasks; ++f) 
	{	
	  // skip task 0 (idle task) 
		if (f - taskset >= MAX_NUM_TASKS)
			panic(0);	// Should never happen 
		if (!f->valid)
			continue;
		if (now >= f->releasetime) { // se è già stato rilasciato
			f->releasetime += f->period;
			++f->released;
			trigger_schedule = 1;	// force scheduler invocation 
			++globalreleases;
		}
		++i;
	}
 */
}

int selectBestTask()
{
  /*
	unsigned long maxprio;
	int i, edf;
	struct task *best, *f;

	maxprio = 100;
	edf = 0;
	best = &taskset[0];
	for (i = 0, f = taskset + 1; i < num_tasks; ++f) { // Salta task 0 (idle)
		if (f - taskset >= MAX_NUM_TASKS)
			panic(0);	// Should never happen 
		if (!f->valid)
			continue;
		++i;
		if (f->released == 0)
			continue;
		if (edf) {
			// fixed-priority tasks have lower priority than EDF ones 
			if (f->deadline == 0)
				continue;
			// priority in EDF tasks is basically a time instant 
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
	return best->id;
  */
}

int schedule()
{
  /*
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
	return best->id;
  */
}
