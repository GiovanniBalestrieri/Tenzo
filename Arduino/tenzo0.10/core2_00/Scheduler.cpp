#include "Scheduler.h"
//#include "Task.h"

struct task *current;

volatile unsigned long globalreleases = 0;	/* used as a status changed flag */

volatile unsigned long trigger_schedule = 0;	/* force rescheduling */


Scheduler::Scheduler(int sizeT)
{
  taskset = new task[sizeT];
  this->MAX_NUM_TASKS = sizeT;
}
  
void Scheduler::initTaskset()
{
  int i;
  num_tasks = 0;
  for (i = 0; i < this->MAX_NUM_TASKS; ++i)
    taskset[i].valid = 0;

  /* Task 0 is special: it is the idle (or kernel) task,
   * which runs whenever no other job is runnable.
   * Actually, taskset[0] is only used as an address
   * placeholder to be assigned to the current variable */

  current = &taskset[0];
  Serial.println("[ Ok ] Taskset initialized");
}


void Scheduler::createTasks()
{
  if (this->create_task(0, HZ, 5, HZ, EDF, "number0") == -1) {
    //puts("ERROR: cannot create task led_cycle\n");
    this->panic(1);
  }
  
  if (this->create_task(1, HZ, 5, 50, EDF, "number1") == -1) {
    //puts("ERROR: cannot create task led_cycle\n");
    this->panic(1);
  }
}

  
int Scheduler::create_task(int id, 
            unsigned long period,
            unsigned long phase,
                    unsigned long prio_dead,
                    int type,
                    const char *label)  
{  
  int i;
  struct task *t;
  for (i = 1; i < this->MAX_NUM_TASKS; ++i) // skip task 0 (idle task) 
    if (!taskset[i].valid)
      break;
  if (i == this->MAX_NUM_TASKS)
    return -1;
  t = taskset + i;
  //t->job = job;
  //t->arg = (arg == NULL ? t : arg);
        t->id = id;
  t->label = label;
  t->period = period;
  t->active = 1;
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
//  init_task_context(t, i);

  cli();
  ++num_tasks;
  t->valid = 1;
  sei();
  return i;
} 

/*
 * Sets active field to 0
 * Returns -1 if task with id "id" is not present
 */
int Scheduler::delete_task(int id)  
{  
  int i;
  for (i = 1; i < this->MAX_NUM_TASKS; ++i) // skip task 0 (idle task) 
    if (taskset[i].id == id)
      taskset[i].active = 0;
  if (i == this->MAX_NUM_TASKS)
    return -1;
  return 1;
} 



void Scheduler::checkPeriodicTasks(void)
{
	unsigned long now = ticks;
	struct task *f;
	int i;

	for (i = 0, f = this->taskset + 1; i < this->num_tasks; ++f) 
	{	
	  // skip task 0 (idle task) 
		if (f - this->taskset >= this->MAX_NUM_TASKS)
			panic(0);	// Should never happen 
    if (!f->valid)
      continue; 
    if (!f->active)
      continue;
		if (now >= f->releasetime) 
		{ 
		  // se è già stato rilasciato update next release
			f->releasetime += f->period;
      // update number of job released
			++f->released;
      // force scheduler invocation 
			trigger_schedule = 1;	
      // increment total number of job released
			++globalreleases;
		}
		++i;
	}
}

int Scheduler::getTaskDeadline(int id)
{
  return taskset[id+1].deadline;
  
}


String Scheduler::getTaskLabel(int id)
{
  return ((String) taskset[id+1].label);
  
}

void Scheduler::panic(int l1)
{
    Serial.println("PANICOOOO!");
}

int Scheduler::selectBestTask()
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

int Scheduler::schedule()
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