#ifndef SENSORS_H
  #define SENSOR_H


float k=0, kM1=0, kMReading = 0, kMRoutine=0, kMLoop=0, secRoutine=0;

// Sonar
int SONAR = 1;
double altitudeSonar = 0;

// RTC
int RTC_ON = 1;

// Volatile vars

volatile float thetaOLD = 0;
volatile float phi=0;
volatile float theta=0;
volatile float psi=0;
volatile int x=0;
volatile int y = 0;
volatile int z= 0;
 float wVal[3] = {0,0,0};
volatile int rawAx = 0;
volatile int rawAy = 0;
volatile int rawAz = 0;
float rawAcc[3];
volatile int dt=0;

volatile float wF[3] = {0,0,0};
volatile float aF[3] = {0,0,0};

// Moved to UX
//volatile boolean filterGyro = true;
//volatile boolean filterAcc = true;

volatile boolean initializedSetup = false;
volatile boolean initializedGyroCalib = false;



volatile float bearing1;

volatile float angleXAcc;
volatile float angleYAcc;
volatile float angleXAccF;
volatile float angleYAccF;

volatile float aax,aay,aaz;
volatile float axm1,aym1,azm1;
volatile float wxm1,wym1,wzm1;
volatile float alphaA= 0.993, alphaW = 0.8;
volatile float estXAngle = 0, estYAngle = 0, estZAngle = 0;
volatile float kG = 0.975, kA = 0.025, kGZ=0.60, kAZ = 0.40;



#endif
