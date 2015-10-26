/*
  NonLinearPid.h - Library implementing a model free approach based on 
  pid control. 
  Created by Giovanni Balestrieri - UserK, August 28, 2015.
*/
#ifndef NonLinearPid_h
#define NonLinearPid_h

#include "Arduino.h"
#include "Propulsion.h"
#include "PID_v2.h"

class NonLinearPid
{
  public:
        NonLinearPid(float, float, float, float);
        
//        PID rollPid( double*,  double*,  double*, double,  double,  double, int);
//        PID pitchPid( double*,  double*,  double*, double,  double,  double, int);
//        PID yawPid( double*,  double*,  double*, double,  double,  double, int);
//        PID wrollPid( double*,  double*,  double*, double,  double,  double, int);
//        PID wpitchPid( double*,  double*,  double*, double,  double,  double, int);
//        PID wyawPid( double*,  double*,  double*, double,  double,  double, int);
//        PID altitudePid( double*,  double*,  double*, double,  double,  double, int);
        
        void changePidState(boolean);
        
        void init();
	void setAutoPid(boolean);
        void setEnablePid(boolean);
	void setEnablePitch(boolean);
        void setEnableRoll(boolean);
	void setEnableWPitch(boolean);
        void setEnableWRoll(boolean);
        void setEnableAltitude(boolean);
        
        //void setSpeedradS(int);
        
  private:
        //int getThrottle();
};
     
        


#endif
