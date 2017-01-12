#include "header.h"
#include "Scheduler.h"
#include "timerh.h"
#include <Servo.h> 

Servo myservo;

// Init scheduler with MAX_TASKS
Scheduler scheduler = Scheduler(MAX_TASKS);

void setup() {
  Serial.begin(9600);

  myservo.attach(servoPin); 
  myservo.writeMicroseconds(MAX_SIGNAL);
  delay(2000);
  myservo.writeMicroseconds(MIN_SIGNAL);
  
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count, FALLING);
  pinMode(13, OUTPUT);

  setupTimerInterrupt();  
  setupScheduler();
  
  timeTracker = micros();
  Serial.println("Welcome");
}

void loop() {
  
  timerSec = micros() - secRoutine;
 //scheduler.checkPeriodicTasks();
  bestId = scheduler.schedule();
  
  switch(bestId)
  {
    case(1):
      // Task 1
      computeRevolutions();
      scheduler.jobCompletedById(bestId);
      break;

    case(2):
      // Task 2
      serialRoutine();
      scheduler.jobCompletedById(bestId);
      break;

    case(3):
      // Task 3
      //computeSignal();
      scheduler.jobCompletedById(bestId);
      break;
        
  } 
}

void setupTimerInterrupt()
{
  // Timer settings
  noInterrupts();
  TCCR2A = 0;
  TCCR2B = 0;

  // Set compare match register to the desired timer count
  OCR2A=14; //16*10^6/(1000Hz*1024)-1 = 77 -> 1068 Hz 
  //OCR3A=77; //16*10^6/(200Hz*1024)-1 = 77 -> 200 Hz 
  //OCR3A=193; //16*10^6/(80Hz*1024)-1 = 193 -> 80 Hz 
  //OCR3A=103; //16*10^6/(150Hz*1024)-1 = 103 -> 150 Hz 
  //OCR3A=143; //16*10^6/(109Hz*1024)-1 = 143 -> 109 Hz s
  //OCR3A=780; //16*10^6/(20Hz*1024)-1 = 780 -> 20 Hz 
  //OCR3A=2000; //16*10^6/(8Hz*1024)-1 = 780 -> 8 Hz 
  //OCR3A=50; //16*10^6/(308Hz*1024)-1 = 50 -> 308 Hz 

  // Set CTC Mode [ WGM22 WGM21 WGM20 ] = [ 0 1 0 ]
  TCCR2A |= (1 << WGM21);
  // Set CS10 and CS12 bits for 1024 prescaler: 
  // [ CS22 CS21 CS20 ] = [ 1 1 1 ]
  
  TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22);
  // When the OCIE2B bit is written to one and the I-bit in 
  // the Status Register is set (one), the Timer/Counter2 
  // Compare Match B interrupt is enabled:
  TIMSK2 |= (1 << OCIE2B);

  // enable global interrupts:
  interrupts();
 
  Serial.println("[ OK ] Init Timers"); 
}

void setupScheduler()
{
  scheduler.initTaskset(); 
  scheduler.createTasks(); 
}

void computeSignal() {

  // TakeOff
  if (initialize) {
    initializing = true;
    
    signalInitializeSequence = currentUs;
    
    if (signalInitializeSequence <= REF_SIGNAL) {
      signalInitializeSequence += signalCounter;
    }
    
    signalCounter++;
    if (signalInitializeSequence >= REF_SIGNAL) {
      signalInitializeSequence = REF_SIGNAL;
      currentUs = REF_SIGNAL;
      initialized = true;
      initialize = false;
      initializing = false;

      
      signalCounter = 0;
      Serial.println("Initialized");
    }
  }  

  // Landing
  if (land) {
    landing = true;
    
    signalLandingSequence = currentUs;
    
    if (signalLandingSequence >= MIN_SIGNAL) {
      signalInitializeSequence -= signalCounter;
      currentUs = signalLandingSequence;
    }
    
    signalCounter++;

    if (signalLandingSequence <= MIN_SIGNAL) {
      currentUs = MIN_SIGNAL;
      landed = true;
      land = false;
      landing = false;
      Serial.println("Landed");
      
      signalCounter = 0;
    }
    
  }

  
  if (test && initialized) {
    testing = true;
    Serial.println("Phase 1");
    
    for (float i = 0; i<100; i=i+0.1) {
      float val = ((MAX_SIGNAL - MIN_SIGNAL)/2) * sin(i) + MIN_SIGNAL;
      myservo.writeMicroseconds((int) val);
      delay(10);
    }
    
    Serial.println("Phase 2");      
    for (int i = 0; i<2000; i++) {
      int val = MIN_SIGNAL;
      if (i<500) {
        val = MIN_SIGNAL;
      }
      else if (i<1000) {
        val = REF_SIGNAL;
      }
      else if (i<1500) {
        val = MAX_SIGNAL*0.7;
      }
      else {
        val = MIN_SIGNAL;
      }
      myservo.writeMicroseconds((int) val);
      delay(10);
    }
    currentUs = MIN_SIGNAL;
    test = false;
    Serial.println("Tested");
  }
  
}

void servoRoutine() {
  myservo.writeMicroseconds(currentUs);
  
  if (initialize) {
    initializing = true;
    for (int i = MIN_SIGNAL; i<=REF_SIGNAL; i++) {
      myservo.writeMicroseconds(i);
      delay(10);
    }
    currentUs = REF_SIGNAL;
    initialized = true;
    initialize = false;
    initializing = false;
    Serial.println("Initialized");
  }
  
  if (land) {
    landing = true;
    for (int i = currentUs; i>MIN_SIGNAL; i--) {
      myservo.writeMicroseconds(i);
      delay(10);
    }
    currentUs = MIN_SIGNAL;
    landed = true;
    land = false;
    landing = false;
    Serial.println("Landed");
  }

  
  if (test && initialized) {
    
    Serial.println("Phase 1");
    for (float i = 0; i<100; i=i+0.1) {
      float val = ((MAX_SIGNAL - MIN_SIGNAL)/2) * sin(i) + MIN_SIGNAL;
      myservo.writeMicroseconds((int) val);
      delay(10);
    }
    
    Serial.println("Phase 2");
      
    for (int i = 0; i<2000; i++) {
      int val = MIN_SIGNAL;
      if (i<500) {
        val = MIN_SIGNAL;
      }
      else if (i<1000) {
        val = REF_SIGNAL;
      }
      else if (i<1500) {
        val = MAX_SIGNAL*0.7;
      }
      else {
        val = MIN_SIGNAL;
      }
      myservo.writeMicroseconds((int) val);
      delay(10);
    }
    currentUs = MIN_SIGNAL;
    test = false;
    Serial.println("Tested");
  }
}


ISR(TIMER2_COMPB_vect) // #ISR
{ 
  isrTimer = micros();
  // increments scheduler ticks
  ticks++;
  contCtrl++;
  // update Tasks info
  scheduler.checkPeriodicTasks();

  //digitalWrite(pinInit, HIGH);
  


    if (contCtrl == ctrlPeriod)
    {
      servoTimer = micros();     
      // [max] 250 us [avg] 240 us   
      
      myservo.writeMicroseconds(currentUs);
      
      //tenzoProp.setSpeeds(tenzoProp.getThrottle(), OutputCascPitchW, OutputCascRollW, OutputCascYawW, OutputCascAlt);
      // update counter control  
  
      countServoAction++;  
      servoTimer = micros() - servoTimer;
      
      if (maxservoTimer <= servoTimer)
        maxservoTimer = servoTimer;
      servoTimeTot = servoTimeTot + servoTimer;

      contCtrl = 0;
    }
    
        
  cont++;    
  countISR++;  
  
  // Compute isr time
  isrTimer = micros() - isrTimer;
  if (maxisrTimer <= isrTimer)
    maxisrTimer = isrTimer;
  isrTimeTot = isrTimeTot + isrTimer;  
  
  //digitalWrite(pinInit, LOW);
}

void resetCounters()
{    
    //cont=0;         
    contCalc=0; 
    countCtrlCalc=0;
    countServoAction=0;  
    contSerialRoutine=0;   
    
    servoTimeTot = 0;
    countISR = 0;
}

