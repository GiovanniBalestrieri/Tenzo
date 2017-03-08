#ifndef Scheduler_h
#define Scheduler_h

#include "Arduino.h"

/* The types of task known to the scheduler */
#define FPR 0
#define EDF 1

extern int SONAR;
extern int RTC_ON;

class Scheduler
{
  public:
    Scheduler(int);
    void checkPeriodicTasks();
    struct task* selectBestTask();
    int schedule();
    int getTaskDeadline(int);
    String getTaskLabel(int);
    int getTaskPeriod(int);
    int getTaskPriority(int);
    // Method to create tasks for the application
    void createTasks();  
    int create_task(int id, 
                    unsigned long period,
                    unsigned long phase,
                    unsigned long prio_dead,
                    int type,
                    const char *label); 
    int delete_task(int id);
    int isTaskAlive(int id);
    int isTaskActive(int id);
    int isTaskValid(int id);
    void panic(int);     
    void initTaskset();
    unsigned long getJobReleased(int id);
    int jobCompletedById(int id);

    struct task *current;
    struct task *taskset;
    int num_tasks;
    int MAX_NUM_TASKS;

    /* used as a status changed flag */
    volatile unsigned long globalreleases = 0; 
    /* force rescheduling */
    volatile unsigned long trigger_schedule = 0;  
};

struct task {
  volatile int valid;    /* this descriptor is associated with a valid task */
  volatile int id;       /* unique identifier */
  volatile unsigned long releasetime;  /* next release time */
  volatile unsigned long released; /* number of released, pending jobs */
  volatile unsigned long period; /* period of the task in ticks */
  volatile unsigned long priority; /* priority of the task (FPR) or job (EDF) */
  volatile unsigned long deadline; /* relative deadline of the job (EDF), zero for FPR */
  const char *label; /* task name */
  volatile int active;  
};

extern volatile unsigned long ticks;

extern int HZ;

#endif
