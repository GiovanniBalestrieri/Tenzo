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

  	_pinM1 = M1;
  	_pinM2 = M2;
  	_pinM3 = M3;
  	_pinM4 = M4;
}

void Propulsion::init()
{
        Serial.println("Propulsion initialized.");
}

void Propulsion::calibrateOnce()
{
  	Serial.println("This program will Arm and Calibrate the ESC.");
  	Serial.println("Now writing maximum output.");
  	Serial.println("Turn on power source, then wait 2 seconds and press any key.");
    
  	servo1.attach(_pinM1);
  	servo2.attach(_pinM2);
  	servo3.attach(_pinM3);
  	servo4.attach(_pinM4);
  	delay(500);
  	servo1.writeMicroseconds(MAX_SIGNAL);
  	servo2.writeMicroseconds(MAX_SIGNAL);
  	servo3.writeMicroseconds(MAX_SIGNAL);
  	servo4.writeMicroseconds(MAX_SIGNAL);

  	Serial.println("Sending minimum output.");
  
  	delay(MAX_SIGNAL);
  	servo1.writeMicroseconds(MIN_SIGNAL);
 	servo2.writeMicroseconds(MIN_SIGNAL);
	servo3.writeMicroseconds(MIN_SIGNAL);
 	servo4.writeMicroseconds(MIN_SIGNAL);
  
  	Serial.println("Done!");
  	throttle = 790;
}
/*
int Propulsion::phase0(int throttle, float rollpid, float pitchpid, float yawpid, float altpid)
{
  // compute motor inputs
  servo1 = throttle + altpid + rollpid - yawpid;
  servo2 = throttle + altpid + pitchpid + yawpid;
  servo3 = throttle + altpid - rollpid - yawpid;
  servo4 = throttle + altpid - pitchpid + yawpid; 

  if (servo1>MAX_SIGNAL)
    servo1 = MAX_SIGNAL;
  if (servo2>MAX_SIGNAL)
    servo2 = MAX_SIGNAL;
  if (servo3>MAX_SIGNAL)
    servo3 = MAX_SIGNAL;
  if (servo4>MAX_SIGNAL)
    servo4 = MAX_SIGNAL;

  // send input to motors
  servo1.writeMicroseconds(servo1);
  servo2.writeMicroseconds(servo2);
  servo3.writeMicroseconds(servo3);
  servo4.writeMicroseconds(servo4);

  // Increase control counter 
  countCtrlAction++;			
}
*/
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
