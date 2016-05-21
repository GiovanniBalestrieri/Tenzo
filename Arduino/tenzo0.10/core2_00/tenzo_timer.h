#ifndef tenzo_timer_h
#define tenzo_timer_h


#define HZ          1000	/* Tick frequency (Hz) */
#define CONFIG_TICK_ADJUST 0

#define TICK_V0	(Timer1_Freq/HZ)
#define TICK_V1	(Timer1_Freq*1000*(1000/HZ))


#define time_after(a,b)		((long)((b)-(a))<0)
#define time_before(a,b)	time_after(b,a)
#define time_after_eq(a,b)	((long)((a)-(b))>=0)
#define time_before_eq(a,b)	time_after_eq(b,a)

// Set period (frequency) scheduler's frequency
// 80 Hz = 0.0125
float period_sched = 0.0093; //us
int freq_sched = 108; //Hz

unsigned int ticks;

#endif
