/*
  propulsion.h - Library to calibrate ESC's, arm, test, spin 
  and stop your Brushless motors.
  Created by Giovanni Balestrieri - UserK, August 25, 2015.
*/
#ifndef Propulsion_h
#define Propulsion_h

#include "Arduino.h"
#include <Servo.h>


// Constants
const int MAX_SIGNAL = 2000;
const int MIN_SIGNAL = 700;
const int IDLE_THRESHOLD = 790;

class Propulsion
{
  public:
        Propulsion(int pinM1, int pinM2, int pinM3, int pinM4);
        void calibrateOnce();
 	// void calibrateAgain();
        void spin(int throttle);
	void resetMotors();
        void test();
        void stopAll();
	int initialize();
	void motorSpeedPid(int throttle, float rollpid, float pitchpid, float yawpid, float altpid);
	int phase0(); 	 

        Servo servo1;
	Servo servo2;
	Servo servo3;
	Servo servo4;

	int throttle = 700;
  private:
        int _pinM1;
        int _pinM2;
        int _pinM3;
        int _pinM4;
};
#endif
