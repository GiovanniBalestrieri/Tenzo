#ifndef Task_h
#define Task_h


class Task
{
  public:
    Task();

	// Task related
	struct task taskset[MAX_NUM_TASKS];     
    
	void initTaskset();
	void createTasks(int);  
	int create_task(int id, 
					unsigned long period,
					unsigned long phase,
               		unsigned long prio_dead,
            		int type,
            		const char *name);          

	int num_tasks;

  private:

}

                
#endif
