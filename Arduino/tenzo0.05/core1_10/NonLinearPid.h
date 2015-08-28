/*
  NonLinearPid.h - Library implementing a model free approach based on 
  pid control. 
  Created by Giovanni Balestrieri - UserK, August 28, 2015.
*/
#ifndef NonLinearPid_h
#define NonLinearPid_h

#include "Arduino.h"
#include <Servo.h>

class NonLinearPid
{
  public:
        NonLinearPid(float, float, float, float);
        void init();
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

boolean autoEnablePid = true;
boolean enablePid = false;

// theta
boolean enableRollPid = true;
boolean enablePitchPid = false;
boolean enableYawPid = false;
// w
boolean enableWRollPid = true;
boolean enableWPitchPid = true;
boolean enableWYawPid = false;
boolean enableAltitudePid = false;

// Define IO and setpoint for control
double SetpointRoll = 0, InputRoll, errorRoll;
double SetpointPitch = 0, InputPitch, errorPitch;Non
double SetpointYaw = 180, InputYaw, errorYaw;
double SetpointAltitude = 1, InputAltitude, errorAltitude;


// Define IO and setpoint for control -----------  W
double SetpointWRoll = 0, InputWRoll, errorWRoll;
double SetpointWPitch = 0, InputWPitch, errorWPitch;
double SetpointWYaw = 180, InputWYaw, errorWYaw;


double OutputRoll = 0;
double OutputPitch = 0;
double OutputYaw = 0;
double OutputAltitude = 0;


double OutputWRoll = 0;
double OutputWPitch = 0;
double OutputWYaw = 0;

// Define the aggressive and conservative Tuning Parameters
// Angle Roll
float aggKpRoll=0.10, aggKiRoll=0.06, aggKdRoll=0.04;
float consKpRoll=0.26, consKiRoll=0.09, consKdRoll=0.03;
float farKpRoll=0.05, farKiRoll=0.09, farKdRoll=0.03;

// Angle Pitch
float aggKpPitch=0.07, aggKiPitch=0.06, aggKdPitch=0.04;
float consKpPitch=0.23, consKiPitch=0.2, consKdPitch=0.01;
float farKpPitch=0.02, farKiPitch=0.09,  farKdPitch=0.02;

// Angle Yaw
double aggKpYaw=0.3, aggKiYaw=0.0, aggKdYaw=0.1;
double consKpYaw=0.3, consKiYaw=0, consKdYaw=0.0;


// Altitude  ->> *** Add it, judst created
double aggKpAltitude=0.2, aggKiAltitude=0.0, aggKdAltitude=0.1;
double consKpAltitude=0.1, consKiAltitude=0, consKdAltitude=0.1;

///////////////////////  WWWWWWWWWWWWWWW   ///////////////////

// W Roll
float aggKpWRoll=0.10, aggKiWRoll=0.06, aggKdWRoll=0.04;
float consKpWRoll=0.1815, consKiWRoll=0.17, consKdWRoll=0.00;
float farKpWRoll=0.05, farKiWRoll=0.09, farKdWRoll=0.03;

// W Pitch
float aggKpWPitch=0.07, aggKiWPitch=0.06, aggKdWPitch=0.04;
float consKpWPitch=0.1, consKiWPitch=0.0, consKdWPitch=0.0;
float farKpWPitch=0.02, farKiWPitch=0.09,  farKdWPitch=0.02;

// W Yaw
double aggKpWYaw=0.3, aggKiWYaw=0.0, aggKdWYaw=0.1;
double consKpWYaw=0.3, consKiWYaw=0, consKdWYaw=0.0;



//Specify the links and initial tuning parameters
PID myRollPID(&InputRoll, &OutputRoll, &SetpointRoll, consKpRoll, consKiRoll, consKdRoll, DIRECT);
PID myPitchPID(&InputPitch, &OutputPitch, &SetpointPitch, consKpPitch, consKiPitch, consKdPitch, DIRECT);
PID myYawPID(&InputYaw, &OutputYaw, &SetpointYaw, consKpYaw, consKiYaw, consKdYaw, DIRECT);
PID myAltitudePID(&InputAltitude, &OutputAltitude, &SetpointAltitude, consKpAltitude, consKiAltitude, consKdAltitude, DIRECT);


//Specify the links and initial tuning parameters
PID wRollPID(&InputWRoll, &OutputWRoll, &SetpointWRoll, consKpWRoll, consKiWRoll, consKdWRoll, DIRECT);
PID wPitchPID(&InputWPitch, &OutputWPitch, &SetpointWPitch, consKpWPitch, consKiWPitch, consKdWPitch, DIRECT);
PID wYawPID(&InputWYaw, &OutputWYaw, &SetpointWYaw, consKpWYaw, consKiWYaw, consKdWYaw, DIRECT);

// Threshold
volatile int thresholdRoll = 7;
volatile int thresholdFarRoll = 20;
volatile int thresholdPitch = 7; 
volatile int thresholdFarPitch = 25;
volatile int thresholdYaw = 15;
volatile int thresholdAlt = 20;

// initialize pid outputs
volatile int rollPID = 0;
volatile int pitchPID = 0;
volatile int yawPID = 0;

#endif
