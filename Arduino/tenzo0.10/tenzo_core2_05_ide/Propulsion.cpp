/*
  Propulsion.cpp - Library for Tenzo Propulsion code.
  Created by Giovanni Balestrieri - UserK, August 25, 2015
  Released into the public domain.
*/

#include "Arduino.h"
#include "Propulsion.h"
#include <Servo.h>

#define DEBUG_PROP = 1;

Propulsion::Propulsion(int M1, int M2, int M3, int M4)
{
  extern Servo servo1;
	extern Servo servo2;
	extern Servo servo3;
	extern Servo servo4;
          
  wUs1=0, wUs2=0, wUs3=0, wUs4=0;        
  _pinM1 = M1;
  _pinM2 = M2;
  _pinM3 = M3;
  _pinM4 = M4;
}

int Propulsion::getThrottle()
{
  return  this->throttle;
}

void Propulsion::setThrottle(volatile int t)
{
  if (t<=MIN_SIGNAL)
    this->throttle = MIN_SIGNAL;
  else if (t>MAX_SIGNAL)
    this->throttle = MAX_SIGNAL;
  else
    this->throttle = t;
}

void Propulsion::init()
{
    this->throttle = MIN_SIGNAL;
    //Serial.println("Propulsion initialized.");
}

void Propulsion::calibrateOnce()
{        
  	Propulsion::detachAll();
    delay(1000);
  	//Serial.println("This program will Arm and Calibrate the ESC. Max...");
    
  	servo1.attach(_pinM1);
  	servo2.attach(_pinM2);
  	servo3.attach(_pinM3);
  	servo4.attach(_pinM4);
  	delay(500);
  	servo1.writeMicroseconds(MAX_SIGNAL);
  	servo2.writeMicroseconds(MAX_SIGNAL);
  	servo3.writeMicroseconds(MAX_SIGNAL);
  	servo4.writeMicroseconds(MAX_SIGNAL);

  	//Serial.println("Minimum...");
  
  	delay(MAX_SIGNAL);
  	servo1.writeMicroseconds(MIN_SIGNAL);
   	servo2.writeMicroseconds(MIN_SIGNAL);
  	servo3.writeMicroseconds(MIN_SIGNAL);
   	servo4.writeMicroseconds(MIN_SIGNAL);
  
  	//Serial.println("Done!");
  	throttle = 790;
}

void Propulsion::calibrateAgain()
{
  	Propulsion::calibrateOnce();
  	Propulsion::init();
}

void Propulsion::setSpeeds(int throttle, float rollpid, float pitchpid, float yawpid, float altpid)
{
  // compute motor inputs
  wUs1 = throttle + altpid - pitchpid + yawpid;
  wUs2 = throttle + altpid + rollpid - yawpid;
  wUs3 = throttle + altpid + pitchpid + yawpid; 
  wUs4 = throttle + altpid - rollpid - yawpid;

  if (wUs1>MAX_SIGNAL)
    wUs1 = MAX_SIGNAL;
  if (wUs2>MAX_SIGNAL)
    wUs2 = MAX_SIGNAL;
  if (wUs3>MAX_SIGNAL)
    wUs3 = MAX_SIGNAL;
  if (wUs4>MAX_SIGNAL)
    wUs4 = MAX_SIGNAL;
    
    
  if (wUs1<MIN_SIGNAL)
    wUs1 = MIN_SIGNAL;
  if (wUs2<MIN_SIGNAL)
    wUs2 = MIN_SIGNAL;
  if (wUs3<MIN_SIGNAL)
    wUs3 = MIN_SIGNAL;
  if (wUs4<MIN_SIGNAL)
    wUs4 = MIN_SIGNAL;
   
  // send input to motors
  servo1.writeMicroseconds(wUs1);
  servo2.writeMicroseconds(wUs2);
  servo3.writeMicroseconds(wUs3);
  servo4.writeMicroseconds(wUs4);    	
}

void Propulsion::test()
{
    Serial.println("Testing motor!");
    Serial.println("They should start spinning.");
    for (int i = MIN_SIGNAL ;i<1500;i++)
    {
      if (i==MIN_SIGNAL || i == 1000)
        Serial.println("tick");
      delay(2);
      servo1.writeMicroseconds(i);
      servo2.writeMicroseconds(i);
      servo3.writeMicroseconds(i);
      servo4.writeMicroseconds(i);
    }
    delay(1000);
    Serial.println("Now, decreasing");
    for (int i = 1500;i<=1000;i--)
    {
      delay(2);
      servo1.writeMicroseconds(i);
      servo2.writeMicroseconds(i);
      servo3.writeMicroseconds(i);
      servo4.writeMicroseconds(i);
    }
    Serial.println("Like that!");
    delay(MAX_SIGNAL);
    Serial.println("And stop.");
    for (int i = 1000;i<=MIN_SIGNAL;i--)
    {
      delay(2);
      servo1.writeMicroseconds(i);
      servo2.writeMicroseconds(i);
      servo3.writeMicroseconds(i);
      servo4.writeMicroseconds(i);
    }
    
    servo1.writeMicroseconds(MIN_SIGNAL);
    servo2.writeMicroseconds(MIN_SIGNAL);
    servo3.writeMicroseconds(MIN_SIGNAL);
    servo4.writeMicroseconds(MIN_SIGNAL);
}

void Propulsion::stopAll()
{
  Serial.println("Arresto forzato.");
  servo1.writeMicroseconds(MIN_SIGNAL);
  servo2.writeMicroseconds(MIN_SIGNAL);
  servo3.writeMicroseconds(MIN_SIGNAL);
  servo4.writeMicroseconds(MIN_SIGNAL);
}

void Propulsion::idle()
{
  servo1.writeMicroseconds(MIN_SIGNAL);
  servo2.writeMicroseconds(MIN_SIGNAL);
  servo3.writeMicroseconds(MIN_SIGNAL);
  servo4.writeMicroseconds(MIN_SIGNAL);
  Serial.println("IDLE");
}

void Propulsion::resetMotors()
{
  Serial.println("This program will calibrate the ESC.");
  Serial.println("Now writing maximum output.");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  
  servo1.attach(_pinM1);
  servo2.attach(_pinM2);
  servo3.attach(_pinM3);
  servo4.attach(_pinM4);
  
  servo1.writeMicroseconds(MIN_SIGNAL);
  servo2.writeMicroseconds(MIN_SIGNAL);
  servo3.writeMicroseconds(MIN_SIGNAL);
  servo4.writeMicroseconds(MIN_SIGNAL);
  
  Serial.println("Armed!");
}

void Propulsion::setSpeedWUs(int thr)
{  
  servo1.writeMicroseconds(thr);
  servo2.writeMicroseconds(thr);
  servo3.writeMicroseconds(thr);
  servo4.writeMicroseconds(thr);
}

void Propulsion::detachAll()
{  
  servo1.writeMicroseconds(0);
  servo2.writeMicroseconds(0);
  servo3.writeMicroseconds(0);
  servo4.writeMicroseconds(0);
  
  servo1.detach();
  servo2.detach();
  servo3.detach();
  servo4.detach();
  
  //Serial.println("Disarmed");
}

int Propulsion::getwUs1() {
    return wUs1;
}



int Propulsion::getwUs2() {
    return wUs2;
}

int Propulsion::getwUs3() {
    return wUs3;
}

int Propulsion::getwUs4() {
    return wUs4;
}

void Propulsion::touchwUs1() {
    wUs1 = wUs1;
}

void Propulsion::touchwUs2() {
    wUs2 = wUs2;
}

void Propulsion::touchwUs3() {
    wUs3 = wUs3;
}

void Propulsion::touchwUs4() {
    wUs4 = wUs4;
}
