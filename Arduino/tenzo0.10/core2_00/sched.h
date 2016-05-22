#ifndef sched_h
#define sched_h

#define MAX_NUM_TASKS	32

typedef void (*job_t) (void *);

struct task {
	int valid;		/* this descriptor is associated with a valid task */
	job_t job;		/* job function to be executed at any activation */
	void *arg;		/* arguments to be passed to the job function */
	unsigned long releasetime;	/* next release time */
	unsigned long released;	/* number of released, pending jobs */
	unsigned long period;	/* period of the task in ticks */
	unsigned long priority;	/* priority of the task (FPR) or job (EDF) */
	unsigned long deadline; /* relative deadline of the job (EDF), zero for FPR */
	const char *name;	/* task name */
	unsigned long sp;	/* saved value for r13/sp */
	unsigned long regs[8];	/*storage area for registers r4-r11 */
};

/* The types of task known to the scheduler */
#define FPR 0
#define EDF 1

void check_periodic_tasks(void);

extern volatile unsigned long ticks;
extern int num_tasks;
extern struct task taskset[MAX_NUM_TASKS];

void init_taskset(void);

extern volatile unsigned long trigger_schedule;
extern struct task *current;	/* the task of the job in execution */

#endif
