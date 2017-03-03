/*
  tenzo_timer_h - Timer variables for Tenzo code.
  Created by Giovanni Balestrieri -
  UserK, August 26, 2015
*/

#ifndef tenzo_timer_h
#define tenzo_timer_h

/* Main Display Loop */
#define MAIN_LOOP_DISP_PERIOD 1000000

int HZ = 1000;	/* Tick frequency (Hz) */
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

volatile unsigned long ticks;



// Counters
volatile unsigned long accTimer = 0;
volatile unsigned long lastAccTimer = 0;
volatile unsigned long timeToRead = 0;
volatile unsigned long lastTimeToRead = 0;

volatile unsigned long eulerTimer = 0;
volatile unsigned long maxeulerTimer = 0;
volatile unsigned long eulerTimeTot = 0;

volatile unsigned long gyroTimer = 0;
volatile unsigned long maxgyroTimer = 0;
volatile unsigned long gyroTimeTot = 0;

volatile unsigned long servoTimer = 0;
volatile unsigned long maxservoTimer = 0;
volatile unsigned long servoTimeTot = 0;


volatile unsigned long controlTimer = 0;
volatile unsigned long maxcontrolTimer = 0;
volatile unsigned long controlTimeTot = 0;

volatile unsigned long isrTimer = 0;
volatile unsigned long maxisrTimer = 0;
volatile unsigned long isrTimeTot = 0;

volatile unsigned long serialTimer = 0;
volatile unsigned long maxserialTimer = 0;
volatile unsigned long serialTimeTot = 0;

// Task 5 Sonar's timers
volatile unsigned long sonarTimer = 0;
volatile unsigned long maxsonarTimer = 0;
volatile unsigned long sonarTimeTot = 0;

// Task 6 Sonar's timers
volatile unsigned long rtcTimer = 0;
volatile unsigned long maxrtcTimer = 0;
volatile unsigned long rtcTimeTot = 0;

/*
 * Variables to hande wcet logging
 */
boolean newCrossThreshold = false;
String labelTaskCross = "";
float valueTaskCross = 0;

/** **/
float timerLoop = 0, timerReading = 0, timerSec = 0;
float timerRoutine = 0, count = 0;
float redingTime = 0, samplingTime = 0, calcTime =0, printTime = 0;
/** **/

volatile int cont = 0;
volatile int countISR = 0;
volatile int countServoAction = 0;
volatile int countCtrlCalc = 0;
volatile int contGyroSamples=0;
volatile int contEulerSamples=0;
volatile int contSerialRoutine=0; // boh
volatile int contSonarRoutine=0;
volatile int contRtcRoutine=0;
volatile int contCalc=0;

volatile int contCtrl = 0;

#endif
