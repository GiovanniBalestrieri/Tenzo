/*
  tenzo_timer_h - Timer variables for Tenzo code.
  Created by Giovanni Balestrieri -
  UserK, August 26, 2015
*/

#ifndef tenzo_timer_h
#define tenzo_timer_h


volatile unsigned long ticks;

volatile int ctrlPeriod = 1; // 10 ms

volatile unsigned long isrTimer = 0;
volatile unsigned long maxisrTimer = 0;
volatile unsigned long isrTimeTot = 0;




volatile unsigned long generatorTimer = 0;
volatile unsigned long maxgeneratorTimer = 0;
volatile unsigned long generatorTimeTot = 0;

volatile unsigned long serialTimer = 0;
volatile unsigned long maxserialTimer = 0;
volatile unsigned long serialTimeTot = 0;

volatile int contCtrl=0;
volatile int cont = 0;
volatile int countISR = 0;
volatile int countServoAction = 0;
volatile int contSerialRoutine=0;
volatile int contGenerator=0;
volatile int contGeneratorRoutine= 0;
volatile int contCalc=0;


// Servo Counter
volatile unsigned long servoTimer = 0;
volatile unsigned long maxservoTimer = 0;
volatile unsigned long servoTimeTot = 0;
volatile int contServo = 0;


#endif
