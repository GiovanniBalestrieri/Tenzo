
#ifndef ControlPid_h
  #define ControlPid_h

// number of ticks to wait before actuation
volatile int ctrlPeriod = 12;

float angles[3];
float inertiaValues[6];

boolean autoEnablePid = true;
volatile boolean enablePid = false;

// theta
volatile boolean enableRollPid = true;
volatile boolean enablePitchPid = true;
volatile boolean enableYawPid = false;
// w
volatile boolean enableWRollPid = true;
volatile boolean enableWPitchPid = true;
volatile boolean enableWYawPid = false;
volatile boolean enableAltitudePid = false;


 
boolean inConsRoll = false; 
boolean inConsPitch = false;
boolean verbosePidValuesFeedback = true;
boolean verboseFilterAccMatlab = true;          


/*
 * Cascade Pid & settings
 */

volatile int limitPidMax = 750;
 
// Roll

// Aggressive settings theta >= thre     
// Rise time: 2.0 s
// Overshoot: 0 %
// Settling time 2.0 s
// Steady state error: 2°
volatile float aggKpCascRoll=1.68, aggKiCascRoll=0, aggKdCascRoll=0.75;
// Conservative settings theta < thre
volatile float consKpCascRoll=3.31, consKiCascRoll=0.54, consKdCascRoll=0.04; //1.5 / 3.2 0.6 0.4

// W part   
//float consKpCascRollW=1.28, consKiCascRollW=1.30, consKdCascRollW=0.10;  // Expensive 
volatile float consKpCascRollW=0.69, consKiCascRollW=0.0, consKdCascRollW=0.009;  // Expensive 

// Pitch        

// Aggressive settings theta >= thre     
volatile float aggKpCascPitch=1.1, aggKiCascPitch=0.00, aggKdCascPitch=0.00;
// Conservative settings theta < thre
volatile float consKpCascPitch=1.1, consKiCascPitch=0.00, consKdCascPitch=0.00; //1.5 / 3.2 0.6 0.4

// W part   
volatile float consKpCascPitchW=0.9, consKiCascPitchW=1.65, consKdCascPitchW=0.3;   

// Yaw

// Aggressive settings theta >= thre     
volatile float aggKpCascYaw=0, aggKiCascYaw=0.00, aggKdCascYaw=0.00;
// Conservative settings theta < thre
volatile float consKpCascYaw=0, consKiCascYaw=0.00, consKdCascYaw=0.00; //1.5 / 3.2 0.6 0.4

// W part   
volatile float consKpCascYawW=0.7, consKiCascYawW=0.01, consKdCascYawW=0.5; 

// Aggressive settings theta >= thre     
volatile float consKpCascAlt=2.5, consKiCascAlt=0.00, consKdCascAlt=0.00;
         
volatile double SetpointCascRoll = 0, InputCascRoll, errorCascRoll;
volatile double SetpointCascRollW = 0, InputCascRollW, errorCascRollW;        
volatile double SetpointCascPitch = 0, InputCascPitch, errorCascPitch;       
volatile double SetpointCascPitchW = 0, InputCascPitchW, errorCascPitchW;
volatile double SetpointCascYaw = 180, InputCascYaw, errorCascYaw;
volatile double SetpointCascYawW = 0, InputCascYawW, errorCascYawW;
volatile double SetpointCascAlt = 1, InputCascAlt, errorCascAlt, OutputCascAlt = 0;    


volatile double OutputCascRoll = 0;
volatile double OutputCascPitch = 0;
volatile double OutputCascYaw = 0;
volatile double OutputCascAltitude = 0;
volatile double OutputCascRollW = 0;
volatile double OutputCascPitchW = 0;
volatile double OutputCascYawW = 0;      


// Threshold
volatile int thresholdRoll = 10;
volatile int thresholdFarRoll = 40;
volatile int thresholdPitch = 10; 
volatile int thresholdFarPitch = 40;
volatile int thresholdYaw = 15;
volatile int thresholdAlt = 20;

// initialize pid outputs
volatile int rollPID = 0;
volatile int pitchPID = 0;
volatile int yawPID = 0;

#endif
