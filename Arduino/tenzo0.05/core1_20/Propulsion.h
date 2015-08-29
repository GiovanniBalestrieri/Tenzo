/*
  propulsion.h - Library to calibrate ESC's, arm, test, spin 
  and stop your Brushless motors.
  Created by Giovanni Balestrieri - UserK, August 25, 2015.
*/
#ifndef Propulsion_h
#define Propulsion_h

#include "Arduino.h"
#include <Servo.h>
#include "Ux.h"


// Constants
const int MAX_SIGNAL = 2000;
const int MIN_SIGNAL = 700;
const int IDLE_THRESHOLD = 790;

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


        int getThrottle();
        void setThrottle(int);	 

        Servo servo1;
	Servo servo2;
	Servo servo3;
	Servo servo4;

        Ux sakuraChan;

        int wUs1;
        int wUs2;
        int wUs3;
        int wUs4;

	int throttle = 700;
  private:
        int _pinM1;
        int _pinM2;
        int _pinM3;
        int _pinM4;
};
#endif
