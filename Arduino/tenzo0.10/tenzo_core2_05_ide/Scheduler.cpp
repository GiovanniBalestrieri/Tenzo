#include "Scheduler.h"

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
    this->taskset[i].valid = 0;

  /* Task 0 is special: it is the idle (or kernel) task,
   * which runs whenever no other job is runnable.
   * Actually, taskset[0] is only used as an address
   * placeholder to be assigned to the current variable */

  this->current = &this->taskset[0];
  Serial.println("[ Ok ] Taskset initialized");
}


void Scheduler::createTasks()
{
  if (this->create_task(1, 20, 0, 15, EDF, "GetEulerW") == -1) {
    //puts("ERROR: cannot create task led_cycle\n");
    this->panic(1);
  }
  
  if (this->create_task(2, 300, 0, 250, EDF, "SerialRoutine") == -1) {
    //puts("ERROR: cannot create task led_cycle\n");
    this->panic(1);
  }
  
  if (this->create_task(3, 4000, 0, 4000, EDF, "UX") == -1) {
    //puts("ERROR: cannot create task led_cycle\n");
    this->panic(1);
  }

  if (SONAR)
  {
    if (this->create_task(4, 150, 1000, 150, EDF, "Sonar set") == -1) {
      //puts("ERROR: cannot create task led_cycle\n");
      this->panic(1);
    }
    
    if (this->create_task(5, 350, 1000, 350, EDF, "Sonar read") == -1) {
      //puts("ERROR: cannot create task led_cycle\n");
      this->panic(1);
    }
  }
  if (RTC_ON)
  {
    if (this->create_task(6, 1000, 1000, 1000, EDF, "RTC") == -1) {
        //puts("ERROR: cannot create task led_cycle\n");
        this->panic(1);
      }
  }
  
  if (this->create_task(7, 5000, 0, 5000, EDF, "LOG") == -1) {
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
    if (!this->taskset[i].valid)
      break;
  if (i == this->MAX_NUM_TASKS)
    return -1;
  t = this->taskset + i;
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
  this->num_tasks++;
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
    if (this->taskset[i].id == id)
      this->taskset[i].active = 0;
  if (i == this->MAX_NUM_TASKS)
    return -1;
  return 1;
} 



void Scheduler::checkPeriodicTasks(void)
{
	unsigned long now = ticks;
	struct task *f;
	int i;

  // skip task 0 (idle task) 
	for (i = 0, f = this->taskset + 1; i < this->num_tasks; ++f) 
	{	
		//if (f - this->taskset >= this->MAX_NUM_TASKS)
		//	this->panic(9);	// Should never happen 
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
			this->trigger_schedule = 1;	
      // increment total number of job released
			++this->globalreleases;
		}
		++i;
	}
}

int Scheduler::getTaskDeadline(int id)
{
  if (this->taskset[id].active == 0 || this->taskset[id].valid == 0)
    return -1;
  else
    return this->taskset[id].deadline;
}

int Scheduler::getTaskPeriod(int id)
{
  if (this->taskset[id].active == 0 || this->taskset[id].valid == 0)
    return -1;
  else
    return this->taskset[id].period;
}

int Scheduler::getTaskPriority(int id)
{
  if (this->taskset[id].active == 0 || this->taskset[id].valid == 0)
    return -1;
  else
    return this->taskset[id].priority;
}


unsigned long Scheduler::getJobReleased(int id)
{
  if (this->taskset[id].active == 0 || this->taskset[id].valid == 0)
    return -1;
  else
    return this->taskset[id].released;
}

String Scheduler::getTaskLabel(int id)
{
  if (this->taskset[id].active == 0 || this->taskset[id].valid == 0)
    return "-1";
  else
    return ( this->taskset[id].label);  
}

int Scheduler::isTaskAlive(int id)
{
  if (this->taskset[id].active == 0 || this->taskset[id].valid == 0)
    return 0;
  else if  (this->taskset[id].active == 1 || this->taskset[id].valid == 1)
    return 1;
}

int Scheduler::isTaskActive(int id)
{
  if (this->taskset[id].active == 0)
    return 0;
  else if  (this->taskset[id].active == 1)
    return 1;
}

int Scheduler::isTaskValid(int id)
{
  if (this->taskset[id].valid == 0)
    return 0;
  else if  (this->taskset[id].valid == 1)
    return 1;
}

void Scheduler::panic(int l1)
{
    Serial.print("PANICOOOO!  \t");
    Serial.println(l1);
}

int Scheduler::jobCompletedById(int id)
{
  if (this->taskset[id].active == 0 || this->taskset[id].valid == 0)
    return -999;
  else if  (this->taskset[id].active == 1 || this->taskset[id].valid == 1)
  {  
   if (this->taskset[id].released > 0)
       --this->taskset[id].released;
   // force scheduler invocation 
   this->trigger_schedule = 1; 
   return this->taskset[id].released;
  }
}

struct task* Scheduler::selectBestTask()
{
	unsigned long maxprio;
	int i, edf;
	struct task *best, *f;

	maxprio = 100;
	edf = 0;
	best = &this->taskset[0];
	// Salta task 0 (idle)
	for (i = 0, f = this->taskset + 1; i < num_tasks; ++f) 
	{ 
		//if (f - taskset >= MAX_NUM_TASKS)
			//panic(0);	// Should never happen 
		if (!f->valid)
			continue;
		++i;
		if (f->released == 0)
			continue;
		if (edf) 
		{
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
		if (f->deadline != 0) 
		{
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

int Scheduler::schedule()
{  
	static int do_not_enter = 0;
	struct task *best;
	unsigned long oldreleases;
	noInterrupts();
	if (do_not_enter != 0) {
		interrupts();
		return -1;
	}
	do_not_enter = 1;
	do {
		oldreleases = this->globalreleases;
    interrupts();
		best = this->selectBestTask();
    noInterrupts();
	} while (oldreleases != globalreleases);
	this->trigger_schedule = 0;
	best = (best != this->current ? best : NULL);
  // Check validity of next statement
  this->current = best;
	do_not_enter = 0;
  interrupts();
	return best->id;
}
