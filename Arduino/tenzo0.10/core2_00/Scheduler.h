#ifndef Scheduler_h
#define Scheduler_h

#include "Arduino.h"

/* The types of task known to the scheduler */
#define FPR 0
#define EDF 1


class Scheduler
{
  public:
    Scheduler(int);
    

    void checkPeriodicTasks();
    int selectBestTask();
    int schedule();

    struct task *taskset;
      
    //void createTasks();  
          
     int create_task(int id, 
                    unsigned long period,
                    unsigned long phase,
                    unsigned long prio_dead,
                    int type,
                    const char *name);       
                   
    void initTaskset();
    int num_tasks;
    int MAX_NUM_TASKS;
};

struct task {
  int valid;    /* this descriptor is associated with a valid task */
    int id;
  unsigned long releasetime;  /* next release time */
  unsigned long released; /* number of released, pending jobs */
  unsigned long period; /* period of the task in ticks */
  unsigned long priority; /* priority of the task (FPR) or job (EDF) */
  unsigned long deadline; /* relative deadline of the job (EDF), zero for FPR */
  const char *name; /* task name */
};

extern volatile unsigned long ticks;

//extern int num_tasks;
//extern struct task taskset[MAX_NUM_TASKS];
//extern void initTaskset();

//extern volatile unsigned long trigger_schedule;
extern struct task *current;	/* the task of the job in execution */

#endif
