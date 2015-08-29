/*
  NonLinearPid.h - Library implementing a model free approach based on 
  pid control. 
  Created by Giovanni Balestrieri - UserK, August 28, 2015.
*/
#ifndef NonLinearPid_h
#define NonLinearPid_h

#include "Arduino.h"
#include <Servo.h>
#include "PID_v2.h"

class NonLinearPid
{
  public:
        NonLinearPid(float, float, float, float);
        PID RollPID;
        PID PitchPID;
        PID YawPID;
        PID AltitudePID;
        void init();
	void setAutoPid(boolean);
        void setEnablePid(boolean);
	void setEnablePitch(boolean);
        void setEnableRoll(boolean);
	void setEnableWPitch(boolean);
        void setEnableWRoll(boolean);
        void setEnableAltitude(boolean);
        //void setSpeedradS(int);
        //void detachAll();
	//void idle();

        //int getThrottle();
        //void setThrottle(int);	 
  private:
        //int getThrottle();
        //void setThrottle(int);	
};

 

#endif
