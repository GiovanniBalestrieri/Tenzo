/*
  Propulsion.h - Library to calibrate ESC's, arm, test, spin 
  and stop your Brushless motors.
  Created by Giovanni Balestrieri - UserK, August 25, 2015.
  www.userk.co.uk
*/
#ifndef Propulsion_h
#define Propulsion_h

#include "Arduino.h"
#include <Servo.h>

// Constants
const int MAX_SIGNAL = 2000;
const int MIN_SIGNAL = 700;
const int IDLE_THRESHOLD = 900;

class Propulsion
{
  public:
    Propulsion(int, int, int, int);
    void init();
    void calibrateOnce();
     void calibrateAgain();
    void resetMotors();
    void test();
    void stopAll();
    int initialize();
    void setSpeeds(int, float, float, float, float);
    void setSpeedWUs(int);
    void setSpeedradS(int);
    void detachAll();
    void idle(); 
    
    int getwUs1();
    int getwUs2();
    int getwUs3();
    int getwUs4();
    
    void touchwUs1();
    void touchwUs2();
    void touchwUs3();
    void touchwUs4();
    
    
    int getThrottle();
    void setThrottle(volatile int);  
  
    Servo servo1;
    Servo servo2;
    Servo servo3;
    Servo servo4;
  
    volatile int wUs1;
    volatile int wUs2;
    volatile int wUs3;
    volatile int wUs4;
  
    volatile int throttle = MIN_SIGNAL;
   
  private:
    int _pinM1;
    int _pinM2;
    int _pinM3;
    int _pinM4;
};
#endif
