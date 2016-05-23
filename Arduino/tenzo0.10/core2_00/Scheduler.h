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
    int getTaskDeadline(int);
    String getTaskLabel(int);

    struct task *taskset;
      
    void createTasks();  
          
    int create_task(int id, 
                    unsigned long period,
                    unsigned long phase,
                    unsigned long prio_dead,
                    int type,
                    const char *name); 

    int delete_task(int id);                          
                   
    void initTaskset();
    int num_tasks;
    int MAX_NUM_TASKS;
    void panic(int);
};

struct task {
  int valid;    /* this descriptor is associated with a valid task */
  int id;       /* unique identifier */
  unsigned long releasetime;  /* next release time */
  unsigned long released; /* number of released, pending jobs */
  unsigned long period; /* period of the task in ticks */
  unsigned long priority; /* priority of the task (FPR) or job (EDF) */
  unsigned long deadline; /* relative deadline of the job (EDF), zero for FPR */
  const char *label; /* task name */
  int active;  
};

extern volatile unsigned long ticks;

//extern int num_tasks;
//extern struct task taskset[MAX_NUM_TASKS];
//extern void initTaskset();
extern int HZ;
//extern volatile unsigned long trigger_schedule;
extern struct task *current;	/* the task of the job in execution */

#endif
