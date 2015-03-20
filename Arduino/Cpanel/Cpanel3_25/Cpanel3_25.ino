
/**********************************************************************
 *             Arduino & L3G4200D gyro & KALMAN & 3 threshold PID     *
 *                running I2C mode, MMA7260Q Accelerometer            *
 *                   Gps & XBEE & MATLAB communication                *
 *                        by Balestrieri Giovanni                     *
 *                          AKA UserK, 2013                           *
 **********************************************************************/

/**
 *  Features:
 *  
 *  - Bias substraction
 *  - Std filter for measurements
 *  - XBee connectivity
 *  - Matlab connection
 *  - Kalman filter
 *  - Complementary Filter
 *  - Motors control
 *  - FreeIMU Ready
 **/
//#define DEBUGMODE

#include <Servo.h>
#include <Wire.h>
#include <CMPS10.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include "Kalman.h"
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#if ARDUINO >= 100
#include <SoftwareSerial.h>
#endif
#include <PID_v1.h>

int versionCode = 320;

/**
 * Print sensor's value
 */
 
boolean printSetupInfo = true;
boolean printAckCommands = true;
boolean printVerboseSerial = true;
boolean printPIDVals = false;
boolean printRawKalman= false;
boolean printRawAcc = false;
boolean printAcc = false;
boolean printRawGyro= false;
boolean printGyro = false;
boolean printRawCompass= false;
boolean printCompass = false;
boolean printKalman = false; 
boolean printMotorsVals = false;
boolean printThrottle = false;

// Keep track of the state
boolean initializing = false;
boolean initialized = false;

/**
 * VTOL settings
 */
 // Take Off settings
int rampTill = 68;
int motorRampDelayFast = 150;
int motorRampDelayMedium = 350;
int motorRampDelaySlow = 550;
int motorRampDelayVerySlow = 850;

// Safe Mode: after timeToLand ms tenzo will land automatically
unsigned long timeToLand = 20000;
boolean autoLand = false;
boolean landing = false;
int landSpeed = 1;
// Landing protocol variables
unsigned long timerStart;
unsigned long checkpoint;

/**
 * Pid Controller 
 */
boolean autoEnablePid = true;
boolean enablePid = false; // Lascia false autoEnablePid lo gestisce.
boolean enableRollPid = false;
boolean enablePitchPid = true;
boolean enableYawPid = false;
boolean enableAltitudePid = false;

// Define IO and setpoint for control
double SetpointRoll = 0, InputRoll, errorRoll, OutputRoll;
double SetpointPitch = 0, InputPitch, errorPitch, OutputPitch;
double SetpointYaw = 180, InputYaw, errorYaw, OutputYaw;
double SetpointAltitude = 1, InputAltitude, errorAltitude, OutputAltitude;

// Define the aggressive and conservative Tuning Parameters
// Roll
float aggKpRoll=0.10, aggKiRoll=0.06, aggKdRoll=0.04;
float consKpRoll=0.22, consKiRoll=0.09, consKdRoll=0.02;
float farKpRoll=0.05, farKiRoll=0.9, farKdRoll=0.03;

// Pitch
float aggKpPitch=0.07, aggKiPitch=0.06, aggKdPitch=0.04;
float consKpPitch=0.24, consKiPitch=0.09, consKdPitch=0.02;
float farKpPitch=0.02, farKiPitch=0.09,  farKdPitch=0.02;

// Yaw
double aggKpYaw=0.3, aggKiYaw=0.0, aggKdYaw=0.1;
double consKpYaw=0.3, consKiYaw=0, consKdYaw=0.0;

// Altitude  ->> *** Add it, judst created
double aggKpAltitude=0.2, aggKiAltitude=0.0, aggKdAltitude=0.1;
double consKpAltitude=0.1, consKiAltitude=0, consKdAltitude=0.1;

//Specify the links and initial tuning parameters
PID myRollPID(&InputRoll, &OutputRoll, &SetpointRoll, consKpRoll, consKiRoll, consKdRoll, DIRECT);
PID myPitchPID(&InputPitch, &OutputPitch, &SetpointPitch, consKpPitch, consKiPitch, consKdPitch, DIRECT);
PID myYawPID(&InputYaw, &OutputYaw, &SetpointYaw, consKpYaw, consKiYaw, consKdYaw, DIRECT);
PID myAltitudePID(&InputAltitude, &OutputAltitude, &SetpointAltitude, consKpAltitude, consKiAltitude, consKdAltitude, DIRECT);

// Threshold
int thresholdRoll = 7;
int thresholdFarRoll = 20;
int thresholdPitch = 7; 
int thresholdFarPitch = 25;
int thresholdYaw = 15;
int thresholdAlt = 20;

// initialize pid outputs
int rollPID = 0;
int pitchPID = 0;
int yawPID = 0;

/** 
 * Kalman
 */ 

Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;
// Final values
int pitchK;
int rollK;
int yawK;
volatile unsigned long timerMu;
float angK[3];
//Offset Kalman
int kalmanXOffset = -0;
int kalmanYOffset = 0;
int kalmanZOffset = 0;
// Estimates
double xAngle;
double yAngle;
double zAngle;

/**
 * Accelerometer
 */
 
const int xaxis = 0;
const int yaxis = 1;
const int zaxis = 2;
float XError,YError,ZError;
float xd,yd,zd;
const float RESOLUTION=800; //0.8 v/g -> resolution 1.5g -> 800mV/g
const float VOLTAGE=3.3;  
const float ZOUT_1G = 850;   // mv Voltage Zout at 1G
const int NADJ  = 50;   
volatile double angleXAcc;
volatile double angleYAcc;
volatile double angleZAcc;

/**
 * Compass
 */

CMPS10 my_compass;
float angPosFilter[3];
int pitch1;
int roll1;
float bearing1;
int filterAng = 0;

/**
  *  Gyroscope 
  */

int wxCandidate,wyCandidate,wzCandidate;
int gyrothresholdHigh = 1000;
volatile int wx,wy,wz;
int wx_past,wy_past,wz_past;
float gyroXAngle,gyroYAngle,gyroZAngle;
int biasWx = 10;
int biasWy = 0;//-12;
int biasWz = 1;
//I2C address of the L3G4200D
#define gyroAddress 105 
// Gyro registers definition
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

/**
  *  Brushless 
  */

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

/**
 * Filter acc params
 */
 
volatile float accF3[3]={0,0,0};
volatile float aax, aay, aaz, axF, ayF, azF;

volatile float xFM3 = 0;
volatile float xFM2 = 0;
volatile float xFM1 = 0;
volatile float uXM3 = 0;
volatile float uXM2 = 0;
volatile float uXM1 = 0;

volatile float yFM3 = 0;
volatile float yFM2 = 0;
volatile float yFM1 = 0;
volatile float uYM3 = 0;
volatile float uYM2 = 0;
volatile float uYM1 = 0;

volatile float zFM3 = 0;
volatile float zFM2 = 0;
volatile float zFM1 = 0;
volatile float uZM3 = 0;
volatile float uZM2 = 0;
volatile float uZM1 = 0;

volatile int contatore = 2;
int pastCount = 0;

//0.05
float aButter2[4] = {0,1,-1.7786,0.8008};
float bButter2[4] = {0,0.0055,0.0111,0.0055};

//0.05
//float aButter3[5] = {0,1,-2.6862,2.4197,-0.7302};
//float bButter3[5] = {0,0.0004,0.0012,0.0012,0.0004};

////0.04
float aButter3[5] = {0,1, -2.7488,2.5282,-0.7776};
float bButter3[5] = {0,0.0002196,0.0006588,0.0006588,0.0002196};


//0.20
//float aButter3[5] = {0,1,  -1.7600, 1.1829, -0.2781};
//float bButter3[5] = {0, 0.0181, 0.0543,0.0543,0.0181};

// Acc Timers
unsigned long accTimer;
unsigned long lastAccTimer;
unsigned long timeToRead = 0;
unsigned long lastTimeToRead = 0;

int contSamples = 0;
int samplesNum = 500;

byte modeS;
unsigned int sensorValue = 0;
boolean enableFilter = true;
boolean enableKalman = true;

int lenBuff = 100;
float values[100];

int rate = 0;
int pastTime = 0;
int timerRate = 0;
int pastTimerTime = 0;

int throttle = 0;

unsigned long aCont;

// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

/**
 *  Serial Communication Protocol
 **/

// Serial
int BaudRateSerial = 9600;
// Gps
int BaudRateGps = 4800;
// Xbee 
int BaudRateXbee = 9600;
uint8_t pinRx = 12 , pinTx = 13; // the pin on Arduino
char GotChar;
byte getData;
SoftwareSerial xbee(pinRx, pinTx);
byte loBytew1, hiBytew1,loBytew2, hiBytew2;
int loWord,hiWord;

// Serial Protocol
int versionArduinoProtocol = 5;
boolean matlab = false;
int mode = 0;
long cmd1;
long cmd2;
long cmd3;
long cmd4;
int numCommandToSend = 0;
int numFilterValuesToSend = 0;

int inputBuffSize = 30;
int outputBuffSize = 30;
byte bufferBytes[30];

// Serial IDs
int arduinoAdd = 1;
int MatlabAdd = 2;

int motorsID = 1;
int accID = 2;
int gyroID = 3;
int magnID = 4;
int estID = 5;
int sonicID = 6;
int gpsID = 7;
int baroID = 8;
int rollConsID = 9;
int pitchConsID = 10;
int yawConsID = 11;
int altitudeConsID = 12;
int rollAggID = 13;
int pitchAggID = 14;
int yawAggID = 15;
int altitudeAggID = 16;
int takeOffID = 17;
int iHoverID = 18;
int landID = 19;
int enableMotorsID = 20;
int sendConsPidRollID = 21;
int sendConsPidPitchID = 22;
int sendConsPidYawID = 23;
int sendConsPidAltID = 24;
int sendAggPidRollID = 25;
int sendAggPidPitchID = 26;
int sendAggPidYawID = 27;
int sendAggPidAltID = 28;
int tenzoStateID = 30;
int connID = 31;
int accValuesID = 32;


int cmdLength = 17;
int shortCmdLength = 9;
int headerLength = 13;

typedef struct mcTag {  // 13 bytes
  unsigned char srcAddr;
  unsigned char dstAddr;
  unsigned long versionX;
  unsigned char numCmds;
  unsigned char hdrLength;
  unsigned char cmdLength;
  unsigned short totalLen;
  unsigned short crc; 
} 
MyControlHdr;

typedef struct ctrTag { // 17 bytes
  unsigned char cmd;
  long param1;
  long param2;
  long param3;
  long param4;   
} 
MyCommand;

unsigned char buffer[47];  // or worst case message size 70, 47: 2 mess

MyControlHdr * pCtrlHdr = (MyControlHdr *)(&buffer[0]);

/**
 * Motors
 */
 
 // Motor constant
int thresholdUp=255, thresholdDown=1;
// Motor speed;
 
// Control Sensors data transmission
boolean sendAccToMatlab = false;
boolean sendGyroToMatlab = false;
boolean sendEstToMatlab = false;
boolean sendMagnToMatlab = false;
boolean sendMotorsToMatlab = false;
boolean sendRollToMatlab = false;
boolean sendPitchToMatlab = false;
boolean sendYawToMatlab = false;
boolean sendAltToMatlab = false;
boolean sendPidState = false;
boolean sendLandAck = false;
boolean sendTakeOffAck = false;
boolean sendHoverState = false;
boolean sendTenzoState = false;
boolean sendConnAck = false;
boolean sendDisconnAck = false;
boolean storeAccData = false;
boolean accDataReady = false;

/**
 * Detect Freq
 */
 
int  contHz = 0;
int  sumHz=0;
boolean detectFreq=false;

float AccelAdjust(int axis)
{
 float acc = 0;
 for (int j=0;j<NADJ;j++)
 {
   float lectura=analogRead(axis);
   acc = acc + ((lectura*5000)/1023.0);
   delay(11); //número primo para evitar ciclos de lectura proporcionales
 }
 return acc/NADJ;
}

void setup()
{  
 Serial.begin(BaudRateSerial); 
 xbee.begin(BaudRateXbee);
 pinMode(xaxis,INPUT);
 pinMode(yaxis,INPUT);
 pinMode(zaxis,INPUT);

 XError =  AccelAdjust(xaxis);
 YError =  AccelAdjust(yaxis);
 ZError =  AccelAdjust(zaxis);
 ZError = ZError - ZOUT_1G; 
 
 servo1.attach(3);  
 servo2.attach(5);    
 servo3.attach(6);   
 servo4.attach(9);

 servo1.write(0);  
 servo2.write(0); 
 servo3.write(0); 
 servo4.write(0);
   
   // Initialise Kalman Values
  kalmanX.setAngle(0);
  kalmanY.setAngle(0);
  kalmanZ.setAngle(0);

  kalmanX.setQbias(2); // Process noise variance for gyro bias
  kalmanY.setQbias(6); // Process noise variance for gyro bias
  kalmanZ.setQbias(6); // Process noise variance for gyro bias

  kalmanX.setQangle(0.05); // Process noise variance for the accelerometer
  kalmanY.setQangle(0.05); // Process noise variance for the accelerometer
  kalmanZ.setQangle(0.05); // Process noise variance for the accelerometer

  // KalmanX.setRmeasure(0.003); // Measurement noise variance

  kalmanX.setRmeasure(0.5); // Measurement noise variance  
  kalmanY.setRmeasure(0.5); // Measurement noise variance
  kalmanZ.setRmeasure(0.5); // Measurement noise variance
  
 /**
  *  Initialize Accelerometer Timer3 800 Hz
  */
  /*
 cli();          // disable global interrupts
 TCCR3A = 0;     // set entire TCCR3A register to 0
 TCCR3B = 0;     // same for TCCR3B
 
 // set compare match register to desired timer count: 800 Hz
 OCR3A = 500; //800Hz 5000; // 3 Hz
 // turn on CTC mode:
 TCCR3B |= (1 << WGM32);
 // Set CS10 and CS12 bits for 1024 prescaler:
 TCCR3B |= (1 << CS30) | (1 << CS32);
 // enable timer compare interrupt:
 TIMSK3 |= (1 << OCIE3B);
 // enable global interrupts:
 sei();
 */
 // set up the ADC
 ADCSRA &= ~PS_128;  // remove bits set by Arduino library
 // you can choose a prescaler from above.
 // PS_16, PS_32, PS_64 or PS_128
 ADCSRA |= PS_32;    // set our own prescaler to 64 
 if (printSetupInfo)
 {
     Serial.print("Tenzo: ");
     Serial.println(versionCode); 
     Serial.println(" Setting up L3G4200D");
     Serial.println("Setup Completed");
 }
 setupL3G4200D(2000);
 delay(500);
}

boolean cond = true;

void loop()
{
  while(true)
  {
    rateLoop = micros() - lastLoopTimer;
    accRoutine();
    getGyroValues();
    getCompassValues();
    estimateAngle();
    control();
    serialRoutine();
    sendDataSensors(matlab);
    
    //dispCountersSpeed();
    if (detectFreq)
    {
      if (contHz<lenBuff)
      {
        values[contHz] = rateLoop;
        contHz++;
      }
      else if (cond)
      {
        for (int j = 0; j<lenBuff; j++)
        //sum = sum + values[j];
        Serial.println(values[j]);
        //Serial.println(values[90]);
        cond = false;
        detectFreq = false;
      }
    }
    lastLoopTimer = micros(); 
  }
}

void control()
{
  if (enablePid)
  {
    // Roll PID
    if (enableRollPid)
    {
      InputRoll = rollK;
      errorRoll = abs(SetpointRoll - rollK); //distance away from setpoint
      //if(errorRoll<thresholdRoll)
      //{  //we're close to setpoint, use conservative tuning parameters
        myRollPID.SetTunings(consKpRoll, consKiRoll, consKdRoll);
      //}
      /*else if(errorRoll>=thresholdRoll && errorRoll<thresholdFarRoll)
      {
        //we're far from setpoint, use aggressive tuning parameters
        myRollPID.SetTunings(aggKpRoll, aggKiRoll, aggKdRoll);
      }
      else if(errorRoll>thresholdFarRoll)
      {
        //we're far from setpoint, use aggressive tuning parameters
        myRollPID.SetTunings(farKpRoll, farKiRoll, farKdRoll);
      }
      */
      myRollPID.Compute(); // Computes outputRoll

      if (printPIDVals)
      {
        Serial.print("ErrorRoll:");
        Serial.print(errorRoll);
        Serial.print("Roll PID: ");
        Serial.print(OutputRoll);
        Serial.println();
      }
    }
    else
    {
      OutputRoll = 0;
    }

    // Pitch PID
    if (enablePitchPid)
    {
      InputPitch = pitchK;
      errorPitch = abs(SetpointPitch - pitchK); //distance away from setpoint

      //if (errorPitch<thresholdPitch)
      //{  //we're close to setpoint, use conservative tuning parameters
        myPitchPID.SetTunings(consKpPitch, consKiPitch, consKdPitch);
      //}
      /*else if (errorPitch>=thresholdPitch && errorPitch<thresholdFarPitch)
      {
        //we're far from setpoint, use aggressive tuning parameters
        myPitchPID.SetTunings(aggKpPitch, aggKiPitch, aggKdPitch);
      }
      else if (errorPitch>=thresholdFarPitch)
      {
        //we're really far from setpoint, use other tuning parameters
        myPitchPID.SetTunings(farKpPitch, farKiPitch, farKdPitch);
      }
*/
      myPitchPID.Compute(); // Computes outputPitch

      if (printPIDVals)
      { 
        Serial.print("E:  ");
        Serial.print(errorPitch);
        Serial.print(" A:");
        Serial.print(OutputPitch);
        Serial.println();
      }
    }  
    else
    {
      OutputPitch = 0;
    }

    // Yaw PID
    if (enableYawPid)
    {
      if (filterAng == 1)
      {
        InputYaw = angPosFilter[2];
        errorYaw = abs(SetpointYaw - angPosFilter[2]); //distance away from setpoint
      }
      else if (filterAng == 0)
      {
        InputYaw = bearing1;
        errorYaw = abs(SetpointYaw - bearing1); 
      }

      if(errorYaw<thresholdYaw)
      {
        //we're close to setpoint, use conservative tuning parameters
        myYawPID.SetTunings(consKpYaw, consKiYaw, consKdYaw);
      }
      else
      {
        //we're far from setpoint, use aggressive tuning parameters
        myYawPID.SetTunings(aggKpYaw, aggKiYaw, aggKdYaw);
      }    
      myYawPID.Compute(); // Resturns outputYaw 

      if (printPIDVals)
      {
        Serial.print(" ErrorYaw: ");
        Serial.print(errorYaw);
        Serial.print(" Yawpid: ");
        Serial.print(OutputYaw);
        Serial.println();
      }     
    }
    else
    {
      OutputYaw=0;
    }
    if (printPIDVals)
    {
      Serial.println();      
    }
  }
  else
  {
    OutputRoll = 0;
    OutputPitch = 0;    
    OutputYaw = 0;
  }
}

void estimateAngle()
{
  xAngle = kalmanX.getAngle(angleXAcc,wx,(double)(micros() - timerMu)) + kalmanXOffset;
  yAngle = kalmanY.getAngle(angleYAcc,wy,(double)(micros() - timerMu)) + kalmanYOffset;
  yAngle = -yAngle;
  /*  
  Serial.println();
  Serial.print("K:( ");
  Serial.print((int)yAngle);
  Serial.print("(");  
  Serial.print("AngAcc:( ");
  Serial.print((int)angleYAcc);
  Serial.print("(");

  if (filterAng == 1)
  {
    zAngle = kalmanZ.getAngle(angPosFilter[2],wz,(double)(micros() - timerMu)) + kalmanZOffset;
  }
  else*/ if (filterAng == 0)
  {
    zAngle = kalmanZ.getAngle(bearing1,wz,(double)(micros() - timerMu)) + kalmanZOffset;
  }

  angK[0] = xAngle;
  angK[1] = yAngle;
  angK[2] = zAngle;
  
  pitchK = angK[0];
  rollK = angK[1];
  yawK = angK[2];

  timerMu = micros();
}

void getCompassValues()
{
  bearing1 =  my_compass.bearing();
  //pitch1 = my_compass.pitch();
//  roll1 = my_compass.roll();

  /*  Create float array of angular positions
   *  Roll  --  X
   *  Pitch --  Y
   *  Yaw   --  Z
   */
  /*
  
  if (filterAng == 1)
  {    
    angPosFilter[0] = roll1;
    angPosFilter[1] = pitch1;
    angPosFilter[2] = bearing1;
    angPosFilterExpMA(angPosFilter); 
    displayValues(angPosFilter[2],angPosFilter[1],angPosFilter[0]);
  }
  else */ 
  /*
  if (filterAng == 0)
  {
    displayValues(bearing1,pitch1,roll1);
  }
  */
}

void dispCountersSpeed()
{   
 if (millis() % 1000 == 0)
 {
   Serial.print(" Timer: "); 
   Serial.print(contatore); 
   Serial.print(" Hz | Loop:  ");
   Serial.print((float) (1000000/rate));
   Serial.println(" Hz");   
   Serial.println();
   contatore = 0;
 }
}

void initializeFast()
{
   for (int j=0; j<rampTill;j++)
   {
      motorSpeed(j);
      //Serial.println(j);
      delay(motorRampDelayFast); 
   }
   throttle=rampTill;
}

void accButter3(float val[])
{
  accF3[0] = -aButter3[2]*xFM1 - aButter3[3]*xFM2 - aButter3[4]*xFM3 + bButter3[1]*val[0] + bButter3[2]*uXM1 + bButter3[3]*uXM2 + bButter3[4]*uXM3;
  
  xFM3 = xFM2;
  xFM2 = xFM1;
  xFM1 = accF3[0];
  uXM3 = uXM2;
  uXM2 = uXM1;
  uXM1 = val[0];
  
  accF3[1] = -aButter3[2]*yFM1 -aButter3[3]*yFM2 -aButter3[4]*yFM3 + bButter3[1]*val[1]+bButter3[2]*uYM1+bButter3[3]*uYM2 +bButter3[4]*uYM3;
  
  yFM3 = yFM2;
  yFM2 = yFM1;
  yFM1 = accF3[1];
  uYM3 = uYM2;
  uYM2 = uYM1;
  uYM1 = val[1];
  
  accF3[2] = -aButter3[2]*zFM1 -aButter3[3]*zFM2 -aButter3[4]*zFM3 + bButter3[1]*val[2]+bButter3[2]*uZM1+bButter3[3]*uZM2 +bButter3[4]*uZM3;
  
  zFM3 = zFM2;
  zFM2 = zFM1;
  zFM1 = accF3[2];
  uZM3 = uZM2;
  uZM2 = uZM1;
  uZM1 = val[2];
  
  val[0]=accF3[0];
  val[1]=accF3[1];
  val[2]=accF3[2];
}


void motorSpeed(int x)
{    
  servo1.write(x);      
  servo2.write(x); 
  servo3.write(x); 
  servo4.write(x); 
}

void landFast()
{
  for (int j=throttle; j>40 ;j--)
  {
    motorSpeed(j);
    //Serial.println(j);
    // Kind or brutal land
    delay(motorRampDelayFast);
  }  
  throttle=0;
}

void getGyroValues()
{
  byte xMSB = readRegister(gyroAddress, 0x29);
  byte xLSB = readRegister(gyroAddress, 0x28);
  
  wxCandidate = ((xMSB << 8) | xLSB);  
  
  //if (abs(wxCandidate-wx_past) <= gyrothresholdHigh)
  //{
  wx=wxCandidate;
  //}
  //wx_past  =  wx;

  byte yMSB = readRegister(gyroAddress, 0x2B);
  byte yLSB = readRegister(gyroAddress, 0x2A);
  wyCandidate = ((yMSB << 8) | yLSB);
  //if (abs(wyCandidate-wy_past) <= gyrothresholdHigh)
  //{
  wy=wyCandidate;
    //wy_past  =  wy;
  //}
  //wy_past  =  wy;

/*
  byte zMSB = readRegister(gyroAddress, 0x2D);
  byte zLSB = readRegister(gyroAddress, 0x2C);
  wzCandidate = ((zMSB << 8) | zLSB);
  
  //if (abs(wzCandidate-wz_past) <= gyrothresholdHigh)
  //{
  wz=wzCandidate;
  //}
*/
  //wz_past  =  wz;
  
  wx = wx - biasWx;
  wy = wy - biasWy;
  wz = wz - biasWz; 
}

void accRoutine()
{  
  // ISR
   aax = analogRead(0);
   aay = analogRead(1);
   aaz = analogRead(2);
   
   float accs[3];        
   accs[0] = (((aax*5000.0)/1023.0)-XError)/RESOLUTION;  
   accs[1] = (((aay*5000.0)/1023.0)-YError)/RESOLUTION;
   accs[2] = (((aaz*5000.0)/1023.0)-ZError)/RESOLUTION;
   
   accButter3(accs);
   axF = accs[0];
   ayF = accs[1];
   azF = accs[2];
   
   if (enableKalman)
   {
      contatore++;          
   } 
  angleXAcc = (atan2(-accs[0],accs[2])) * RAD_TO_DEG;
  angleYAcc = (atan2(accs[1],accs[2])) * RAD_TO_DEG;
}

int setupL3G4200D(int scale)
{
  //From  Jim Lindblom of Sparkfun's code

    // Enable x, y, z and turn off power down:
  writeRegister(gyroAddress, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(gyroAddress, CTRL_REG2, 0b00001000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(gyroAddress, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(gyroAddress, CTRL_REG4, 0b00100000);
  }
  else if(scale == 500){
    writeRegister(gyroAddress, CTRL_REG4, 0b00010000);
  }
  else{
    writeRegister(gyroAddress, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(gyroAddress, CTRL_REG5, 0b00010000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address)
{
  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}

void serialRoutine()
{
   if (Serial.available())
   {
     modeS = Serial.read();
     
     if (modeS == 83)
     {
       lastTimeToRead = millis();
       for (int i=0; i <= lenBuff ; i++)
       {
         timeToRead = millis() - lastTimeToRead;
         accTimer = micros() - lastAccTimer;
         int x = analogRead(xaxis);
         int y = analogRead(yaxis);
         int z = analogRead(zaxis);        
         
         float accs[3];        
         accs[0] = (((x*5000.0)/1023.0)-XError)/RESOLUTION;
         accs[1] = (((y*5000.0)/1023.0)-YError)/RESOLUTION;
         accs[2] = (((z*5000.0)/1023.0)-ZError)/RESOLUTION;
         
         if (enableFilter)
         {
           //float axF = accButter2(aax);
           accButter3(accs);
           float axF = accs[0];
           float ayF = accs[1];
           float azF = accs[2];
           lastAccTimer = micros(); 
           if (enableKalman)
           {
              //getGyroValues();              
              // Store Val
              //values[i] = axF;
              //Serial.println(wx);
           }
           else
           {
             //values[i] = axF;
           }
         }
         else
         {         
           lastAccTimer = micros(); 
           //values[i] = aax;
         }
         
         //delay(3);
         //motorSpeed(throttle);
       }  
       
       for (int i=0;i<=lenBuff;i++)
       {
         Serial.print("X");
         Serial.print(",");
         //Serial.println(values[i]);
       }
       
       rate = accTimer;
       Serial.print("U");
       Serial.print(",");
       Serial.println(rate);   
       //landFast();  
       contSamples = 0;  
     } 
     else if (modeS == 84)
     {
       Serial.print("R");
       Serial.print(",");
       Serial.print(aax);
       Serial.print(",");
       Serial.print(aay);
       Serial.print(",");
       Serial.print(aaz);
       Serial.print(",");
       Serial.println(lastAccTimer);
     }
     else if (modeS == 85) // 'U'
     {
       Serial.print(axF);
       Serial.print("  ");
       Serial.print(wx);
       Serial.print("  ");
       Serial.print(pastCount);
       Serial.println();
       Serial.print("   Ax : ");
       Serial.print(axF);
       Serial.print("   Wx : ");
       Serial.print(wx);
       Serial.print("   Pitch: ");
       Serial.print(pitchK);
       Serial.print("   Roll: ");
       Serial.println(rollK);
       Serial.print("   Yaw : ");
       Serial.println(yawK);
     }
    else if(modeS == 16)
    {  	
      Serial.write(17);
    }
    else if (modeS == 18)
    {
      Serial.write(19);      
      initialize();
    }
    else if (modeS == 'w')
    {
      if (throttle<thresholdUp)
      {
        throttle = throttle + 5;
      }
    }
    else if (modeS == 'z')
    {
      Serial.println("Serial outputs disabled");
      disableSerialOut();
    }
    else if (modeS == 'f')
    {
      detectFreq = true;
      contHz=0;
      cond = true;
      Serial.println("Detecting loop Frequency...");
    }
    else if (modeS == 's')
    {
      if (throttle>thresholdDown)
      {
        throttle = throttle - 5;
      }
    }

    if (modeS == 'e')
    {
      if (throttle<thresholdUp)
      {
        throttle = throttle + 1;
      }
    }
    else if (modeS == 'd')
    {
      if (throttle>thresholdDown)
      {
        throttle = throttle - 1;
      }
    }

    if (modeS== '1')
    {
      Serial.println("M1");
      testMotor(1);
    }
    else if (modeS== '2')
    {
      testMotor(2);
    }
    else if (modeS== '3')
    {
      testMotor(3);
    }
    else if (modeS== '4')
    {
      testMotor(4);
    }
    else if(modeS == 'L')
    {
      land();
    }
    else if(modeS == 'B')
    {  	
      //getBmpValues();
    }
    else if(modeS == 'G')
    {  	
      enablePid = false;
    }
    else if(modeS == 'T')
    {  	
      enablePid = true;
    }  
    else if(modeS == 'r')
    {  	
      resetMotorsPidOff();
    } 
    else if (modeS == 'P')
    {
      dispActualAllPidVals();
    }
  }
   
  if(xbee.available())
  { 
    //if (printVerboseSerial)
    //{
      Serial.print("- Received Tot: ");
      Serial.println(xbee.available());
    //}
    if (xbee.available() >= inputBuffSize)
    {  
      for (int j=0;j<=inputBuffSize && printVerboseSerial;j++)
      {
        bufferBytes[j]=xbee.read();
        Serial.print("Received: ");
        Serial.print(bufferBytes[j]);
        Serial.println();
      }
      if (bufferBytes[0] == 2 && bufferBytes[1]==1)
      {
        /**
         * Message has been correctly sent by MATLAB and delivered to the Arduino 
         * Decoding Header
         **/

        // Assembling VERSION long skip first two bytes

        hiBytew2 = bufferBytes[4];
        //Serial.println(hiBytew2,BIN);
        loBytew2 = bufferBytes[5];
        //Serial.println(loBytew2,BIN);
        hiWord = word(hiBytew2, loBytew2);
        //versionProtocol = makeLong( hiWord, loWord);
        int versionProtocol = hiWord;
        //  number cmd
        int numCmd = bufferBytes[6];
        int headL = bufferBytes[7];
        int cmdL = bufferBytes[8];
        //  total Length
        hiBytew2 = bufferBytes[9];
        //Serial.println(hiBytew2,BIN);
        loBytew2 = bufferBytes[10];
        //Serial.println(loBytew2,BIN);
        short totL = word(hiBytew2, loBytew2);
        // CRC
        hiBytew2 = bufferBytes[11];
        //Serial.println(hiBytew2,BIN);
        loBytew2 = bufferBytes[12];
        //Serial.println(loBytew2,BIN);
        short crc = word(hiBytew2, loBytew2);
        /**
         * Decoding Command
         **/
        int type = bufferBytes[13];
        if (printVerboseSerial)
        {
          Serial.println();
          Serial.print("Version");
          Serial.print(versionProtocol);
          if (versionProtocol != versionArduinoProtocol)
            Serial.println("Warning Sync Repos, different serial protocols");
          Serial.println();
          Serial.print("Tot Len");
          Serial.println(totL);
          Serial.println();
          Serial.print("Crc");
          Serial.println(crc);
          Serial.print("TYPE");
          Serial.println(type);
        }
        if (type == connID)
        {
          // Connection channel
          hiBytew2 = bufferBytes[16];
          loBytew2 = bufferBytes[17];
          int conAck = word(hiBytew2, loBytew2); 

          if (conAck == 1)
          {
            if (!matlab)
            {
              sendConnAck = true;
              sendTenzoState = true;
            }
            else 
            {
              sendConnAck = true;
              sendTenzoState = true;
              // Connection requested but already connected
              if (printVerboseSerial)
              {
                Serial.println();
                Serial.println(" Warning!! Something wrong during CONnection sync.");
                Serial.print("Connection requested but already connected");
                Serial.println();
              }
            }
          }
          else if (conAck == 3)
          {
            // Communication problems
            if (!matlab)
            {
              sendConnAck = true;
              sendTenzoState = true;
              if (printVerboseSerial)
              {
                Serial.println();
                Serial.println(" Warning!! Communication problems. Sending again.");
                Serial.println();
              }
            }
            else 
            {
              sendConnAck = true;
              sendTenzoState = true;
              // Impossible: Connection requested but already connected
              if (printVerboseSerial)
              {
                Serial.println();
                Serial.println(" Warning!! Something wrong during CONnection sync.");
                Serial.print("Connection requested but already connected");
                Serial.println();
              }
            }
          }
          else if (conAck == 0)
          {
            if (matlab)
            {
              sendDisconnAck = true;
            }
            else 
            {
              sendDisconnAck = true;
              // Impossible: Disconnection requested but already disconnected
              if (printVerboseSerial)
              {
                Serial.println();
                Serial.println(" Warning!! Something wrong during DISconnection sync.");
                Serial.print("Disconnection requested but already disconnected");
                Serial.println();
              }
            }
          }
          else
          {
            //  Shouldn't get here - Wrong mess received
            Serial.println();
            Serial.println(" Warning!! Third Condition.");
            Serial.println();
          }
        }
        else if (type == tenzoStateID)
        {
          hiBytew2 = bufferBytes[16];
          loBytew2 = bufferBytes[17];
          int remoteTakeOffTemp = word(hiBytew2, loBytew2);

          hiBytew2 = bufferBytes[20];
          loBytew2 = bufferBytes[21];
          int remoteHoverTemp = word(hiBytew2, loBytew2);

          hiBytew2 = bufferBytes[24];
          loBytew2 = bufferBytes[25];
          int remoteLandTemp = word(hiBytew2, loBytew2);

          if (printAckCommands) 
          {
            // Impossible: Disconnection requested but already disconnected
            Serial.println();
            Serial.println(" | Check state requested |");
            Serial.println();
          }

          checkLocalRemoteStates(remoteTakeOffTemp,remoteHoverTemp,remoteLandTemp);

        }
        else if (type == motorsID)
        {
          // Motors : same cmd   

          stopSendingToMatlab();

          hiBytew2 = bufferBytes[16];
          loBytew2 = bufferBytes[17];
          int motorSpeedChange = word(hiBytew2, loBytew2);
          motorSpeedChange = motorSpeedChange - 100;

          hiBytew2 = bufferBytes[20];
          loBytew2 = bufferBytes[21];
          int m2 = word(hiBytew2, loBytew2);
          m2 = m2 - 100;

          hiBytew2 = bufferBytes[24];
          loBytew2 = bufferBytes[25];
          int m3 = word(hiBytew2, loBytew2);
          m3 = m3 -100;

          hiBytew2 = bufferBytes[28];
          loBytew2 = bufferBytes[29];
          int m4 = word(hiBytew2, loBytew2);
          m4 = m4 -100;

          if (motorSpeedChange == 10)
          {
            //Serial.println("M1");
            testMotor(1);
          }
          else if (m2 == 10)
          {
            //Serial.println("M2");
            testMotor(2);
          }
          else if (m3 == 10)
          {
            //Serial.println("M3");
            testMotor(3);  
          }
          else if (m4 == 10)
          { 
            //Serial.println("M4");
            testMotor(4);
          }
          else if (motorSpeedChange > 1)
          {
            if (throttle<thresholdUp)
            {
              throttle = throttle + motorSpeedChange;
              if (printVerboseSerial)
              {
                Serial.println();
                Serial.print("new Speed: ");
                Serial.print(motorSpeedChange);
                Serial.println(); 
              }
              sendMotorsToMatlab = true;
            }
          }
          else if (motorSpeedChange < -1)
          {
            if (throttle>thresholdDown)
            {
              throttle = throttle + motorSpeedChange;
              if (printVerboseSerial)
              {
                Serial.println();
                Serial.print("new Speed: ");
                Serial.print(motorSpeedChange);
                Serial.println(); 
              }
              sendMotorsToMatlab = true;
            } 
          }
        }
        else if (type == accID)
        {
          // Send Acc Values
          stopSendingToMatlab();
          sendAccToMatlab = true;
        }
        else if (type == gyroID)
        {
          // Send Gyro Values
          stopSendingToMatlab();
          sendGyroToMatlab = true;
        }
        else if (type == magnID || type == estID)
        {
          // Send Est + Magn Values
          stopSendingToMatlab();
          sendMagnToMatlab = true;
          sendEstToMatlab = true;
        }
        else if (type == takeOffID)
        {
          // Channel 17: Take Off
          stopSendingToMatlab();

          hiBytew2 = bufferBytes[16];
          loBytew2 = bufferBytes[17];
          int altitudeRef = word(hiBytew2, loBytew2);
          if (printVerboseSerial)
          {
            Serial.println();
            Serial.print("Channel: 17   altitudeRef: ");
            Serial.print(altitudeRef);
            Serial.println();
          }
          // TODO set altitude reference when initiate
          if (!initialized)
          {
            initialize();
          }
          else
          {
            Serial.println(" Already out in space");
          }
        }
        else if (type == landID)
        {
          // Channel 19: Land
          stopSendingToMatlab();

          hiBytew2 = bufferBytes[16];
          loBytew2 = bufferBytes[17];
          landSpeed = word(hiBytew2, loBytew2);
          if (printVerboseSerial)
          {
            Serial.println();
            Serial.print("Channel: 19   landSpeed: ");
            Serial.print(landSpeed);
            Serial.println();
          }
          // TODO: Aggressive or kind stop
          if (initialized)
          {
            land();
          }
          else
          {
            Serial.println();
            Serial.print("Received Land command but Tenzo is not hovering!! WARNING!!");
            Serial.println();
          }
        }
        else if (type == iHoverID)
        {
          // Channel 18: Hovering
          if (initialized)
          {
            // Hover with PID
            stopSendingToMatlab();

            hiBytew2 = bufferBytes[16];
            loBytew2 = bufferBytes[17];
            int enab = word(hiBytew2, loBytew2);             
            if (printVerboseSerial)
            {
              Serial.println();
              Serial.print("Channel: 18   enab Pid: ");
              Serial.print(enab);
              Serial.println();
            }
            if (enab==0 && enablePid)
            {
              enablePid = false;
              sendHoverState = true;
              numCommandToSend++;
            }
            else if (enab==0 && !enablePid)
            {
              Serial.println();
              Serial.print("Received Pid 'switch off' request but has already been disabled! WARNING!!");
              Serial.println();
            }
            else if (enab == 1 && enablePid)
            {
              Serial.println();
              Serial.print("Received Pid activation request but already enable! WARNING!!");
              Serial.println();
            }
            else if (enab == 1 && !enablePid)
            {
              enablePid = true;
              sendHoverState = true;
              numCommandToSend++;               
            }
          }
          else
          {
            Serial.println();
            Serial.print("Received Hovering command but Tenzo is not flying!! WARNING!!");
            Serial.println();             
          }
        }
        else if (type == enableMotorsID) // Deprecated
        {
          // Request Motors Data  type: 20  
          stopSendingToMatlab();

          hiBytew2 = bufferBytes[16];
          loBytew2 = bufferBytes[17];
          int enab = word(hiBytew2, loBytew2);
          if (enab==0)
            sendMotorsToMatlab = false;
          else if (enab == 1)
            sendMotorsToMatlab = true;
        }
        else if (type == sendConsPidRollID || type == sendAggPidRollID)
        {
          // Request Motors Data   type: 21      || 25  
          stopSendingToMatlab();
          sendPidState = true;
          sendRollToMatlab = true;          
          if (printVerboseSerial)
          {
            Serial.println();
            Serial.print("Channel: 21||25   roll Pid? ");
            Serial.println();
          }
        }
        else if (type == sendConsPidPitchID || type == sendAggPidPitchID)
        {
          // Request Motors Data   type: 22      || 26 
          stopSendingToMatlab();
          sendPidState = true;
          sendPitchToMatlab = true;         
          if (printVerboseSerial)
          {
            Serial.println();
            Serial.print("Channel: 22||26   pitch Pid? ");
            Serial.println();
          }
        }
        else if (type == sendConsPidYawID || type == sendAggPidYawID)
        {
          // Request Motors Data   type: 23      || 27
          stopSendingToMatlab();
          sendPidState = true;
          sendYawToMatlab = true;         
          if (printVerboseSerial)
          {
            Serial.println();
            Serial.print("Channel: 23||27   yaw Pid? ");
            Serial.println();
          }
        }
        else if (type == sendConsPidAltID || type == sendAggPidAltID)
        {
          // Request Motors Data   type: 24      || 28
          stopSendingToMatlab();
          sendPidState = true;
          sendAltToMatlab = true;         
          if (printVerboseSerial)
          {
            Serial.println();
            Serial.print("Channel: 24||28   alt Pid? ");
            Serial.println();
          }
        }
        else 
        {
          stopSendingToMatlab();

          hiBytew2 = bufferBytes[16];
          loBytew2 = bufferBytes[17];
          cmd1 = word(hiBytew2, loBytew2);

          hiBytew2 = bufferBytes[20];
          loBytew2 = bufferBytes[21];
          cmd2 = word(hiBytew2, loBytew2);

          hiBytew2 = bufferBytes[24];
          loBytew2 = bufferBytes[25];
          cmd3 = word(hiBytew2, loBytew2);

          hiBytew2 = bufferBytes[28];
          loBytew2 = bufferBytes[29];
          cmd4 = word(hiBytew2, loBytew2);

          if (type == 9)
          {
            // Pid Cons ROLL 
            consKpRoll = (double) cmd1/1000;
            consKdRoll = (double) cmd2/1000;
            consKiRoll = (double) cmd3/1000;
            SetpointRoll = cmd4;
            sendPidState = true;
            changePidValues();
            sendRollToMatlab = true;        
            if (printVerboseSerial)
            {
              Serial.println();
              Serial.println("AA Cons Roll Pid ");
              Serial.println(consKpRoll);
              Serial.println(consKdRoll);
              Serial.println(consKiRoll);
              Serial.println(SetpointRoll);
              Serial.println();
            }
          }
          else if (type == 13)
          {
            // Pid AGG ROLL 
            aggKpRoll = (double) cmd1/1000;
            aggKdRoll = (double) cmd2/1000;
            aggKiRoll = (double) cmd3/1000;
            SetpointRoll = cmd4;
            sendPidState = true;
            changePidValues();
            sendRollToMatlab = true;      
            if (printVerboseSerial)
            {
              Serial.println();
              Serial.println("Agg Roll Pid ");
              Serial.println(aggKpRoll);
              Serial.println(aggKdRoll);
              Serial.println(aggKiRoll);
              Serial.println(SetpointRoll);
              Serial.println();
            }
          }
          else if (type == 10)
          {
            // Pid Cons PITCH 
            consKpPitch = (double) cmd1/1000;
            consKdPitch = (double) cmd2/1000;
            consKiPitch = (double) cmd3/1000;
            SetpointPitch = cmd4;
            sendPidState = true;
            changePidValues();
            sendPitchToMatlab = true;
            if (printVerboseSerial)
            {
              Serial.println();
              Serial.println("cons pitch Pid ");
              Serial.println(consKpPitch);
              Serial.println(consKdPitch);
              Serial.println(consKiPitch);
              Serial.println(SetpointRoll);
              Serial.println();
            }
          }
          else if (type == 14)
          {
            // Pid AGG PITCH 
            aggKpPitch = (double) cmd1/1000;
            aggKdPitch = (double) cmd2/1000;
            aggKiPitch = (double) cmd3/1000;
            SetpointPitch = cmd4;
            sendPidState = true;
            changePidValues();
            sendPitchToMatlab = true;
            if (printVerboseSerial)
            {
              Serial.println();
              Serial.println("Agg Roll Pid ");
              Serial.println(aggKpPitch);
              Serial.println(aggKdPitch);
              Serial.println(aggKiPitch);
              Serial.println(SetpointRoll);
              Serial.println();
            }
          }
          else if (type == 11)
          {
            // Pid Cons YAW 
            consKpYaw = (double) cmd1/1000;
            consKdYaw = (double) cmd2/1000;
            consKiYaw = (double) cmd3/1000;
            SetpointYaw = cmd4;
            sendPidState = true;
            changePidValues();
            sendYawToMatlab = true;
          }
          else if (type == 15)
          {
            // Pid AGG YAW 
            aggKpYaw = (double) cmd1/1000;
            aggKdYaw = (double) cmd2/1000;
            aggKiYaw = (double) cmd3/1000;
            SetpointYaw = cmd4;
            sendPidState = true;
            changePidValues();
            sendYawToMatlab = true;
          }
          else if (type == 12)
          {
            // Pid Cons ALTITUDE 
            consKpAltitude = (double) cmd1/1000;
            consKdAltitude = (double) cmd2/1000;
            consKiAltitude = (double) cmd3/1000;
            SetpointAltitude = cmd4;
            sendPidState = true;
            changePidValues();
            sendAltToMatlab = true;
          }
          else if (type == 16)
          {
            // Pid AGG ALTITUDE 
            aggKpAltitude = (double) cmd1/1000;
            aggKdAltitude = (double) cmd2/1000;
            aggKiAltitude = (double) cmd3/1000;
            SetpointAltitude = cmd4;
            sendPidState = true;
            changePidValues();
            sendAltToMatlab = true;
          }           
        }
      } // Message to Arduino from Matlab
    }
    else if (xbee.available() > 0)
    { 
      getData = xbee.read();  

      Serial.println();
      Serial.print("Received: ");
      Serial.print(getData);
      Serial.println();

      // Establishing Matlab Communication  
      if(getData == 16)
      {  	 
        byte message[] = {0x11};
        xbee.write(message, sizeof(message));

        Serial.println();
        Serial.print(" Connection requested. Sending to Matlab:  ");
        Serial.println();
        Serial.print(message[0]);
        sendTenzoState = true;
      } 
      else if (getData == 18)
      {
        Serial.println(" Matlab communication established. ");
        matlab = true;
      }
      else if(getData == 19)
      {  	 
        Serial.println(" [Warning] Matlab communication problem. ");
      }  
      else if (getData == 20)
      {  	 
        byte message[] = {
          0x15        };
        xbee.write(message, sizeof(message));

        Serial.println();
        Serial.print(" Sending to Matlab:  ");
        Serial.println(message[0]);  
        Serial.println(" Disconnected. ");
        Serial.println(); 

        matlab = false;
      }
    }
  }
  
  if (initialized||matlab)
  {
    //motorSpeed(throttle);
    motorSpeedPID(throttle,OutputPitch ,OutputRoll, OutputYaw);
    if (printThrottle)
    {
      Serial.println(throttle);
    }
  }
}

void changePidValues()
{
  if (enablePid)
  {
    // Roll PID
    if (enableRollPid)
    {
      if(errorRoll<thresholdRoll)
      {  //we're close to setpoint, use conservative tuning parameters
        myRollPID.SetTunings(consKpRoll, consKiRoll, consKdRoll);
      }
      else
      {
        //we're far from setpoint, use aggressive tuning parameters
        myRollPID.SetTunings(aggKpRoll, aggKiRoll, aggKdRoll);
      }
    }

    // Pitch PID
    if (enablePitchPid)
    {
      if(errorPitch<thresholdPitch)
      {  //we're close to setpoint, use conservative tuning parameters
        myPitchPID.SetTunings(consKpPitch, consKiPitch, consKdPitch);
      }
      else
      {
        //we're far from setpoint, use aggressive tuning parameters
        myPitchPID.SetTunings(aggKpPitch, aggKiPitch, aggKdPitch);
      }
    }

    // Yaw PID
    if (enableYawPid)
    {
      if(errorYaw<thresholdYaw)
      {
        //we're close to setpoint, use conservative tuning parameters
        myYawPID.SetTunings(consKpYaw, consKiYaw, consKdYaw);
      }
      else
      {
        //we're far from setpoint, use aggressive tuning parameters
        myYawPID.SetTunings(aggKpYaw, aggKiYaw, aggKdYaw);
      } 
    }
    if (printPIDVals)
    {
      dispActualAllPidVals();     
    }
  }
}

void checkLocalRemoteStates(int to, int h, int l)
{
  boolean toCond = false, hCond = false, lCond = false;
  // Checks whether states have been correctly synced
  if (initialized && to == 1)
  {
    toCond = true;
  } 
  else if (!initialized && to == 0)
  {
    toCond = true;
  } 

  if (enablePid && h==1)
  {
    hCond = true;
  }
  else if (!enablePid && h==0)
  {   
    hCond = true;
  }

  if (initialized && l==0)
  {
    lCond = true;
  }
  else if (!initialized && l==1)
  {   
    lCond = true;
  } 

  if (lCond && hCond && toCond)
  {
    matlab = true;
    if (printAckCommands)
    {
      Serial.println();
      Serial.print("[Sync] Tutto a porno");
      Serial.println();
    }
  }
  else
  {
    sendTenzoState = true;
    if (printAckCommands)
    {
      Serial.println();
      Serial.print("[Sync] state problem between Arduino (local) and Matlab (remote). Sending states... ");
      Serial.println();
    }
  }
}
 
void stopSendingToMatlab()
{  
  sendAccToMatlab = false;
  sendGyroToMatlab = false;
  sendMagnToMatlab = false;
  sendEstToMatlab = false;
  sendRollToMatlab = false;
  sendMotorsToMatlab = false;
  sendPitchToMatlab = false;
  sendYawToMatlab = false;
  sendAltToMatlab = false;
} 
 
void resetMotorsPidOff()
{
  throttle = 0;
  motorSpeed(0);
  // Sets timerStart to 0
  timerStart = 0;
  checkpoint = 0;
  // Disable Pid when motors are off
  if (enablePid)
  {
    // Change only if PID is enabled 
    // Tenzo has landed, no need of controls
    changePidState(false);
  }
} 

void resetMotors()
{
  throttle = 0;
  motorSpeed(0);
  // Sets timerStart to 0
  timerStart = 0;
  checkpoint = 0;
  // Disable Pid when motors are off
}

void protocol1()
{  
  if (autoLand)
  {
    // If motors are on updates timer
    if (initialized)
      timerStart = millis() - checkpoint;
    // if Tenzo has already been initialized for a timeToLand period then land
    if (initialized && timerStart>=timeToLand)
    {
      land();
    }
  }
}
void land()
{
  if (initialized)
  {
    landing = true;
    Serial.println();
    Serial.print("Landing protocol started...");
    Serial.print(throttle);
    Serial.print(" ");
    for (int j=throttle; j>40 ;j--)
    {
      motorSpeed(j);
      Serial.println(j);
      // Kind or brutal land
      if (landSpeed == 1)
        delay(motorRampDelayFast);
      else if (landSpeed == 2)
        delay(motorRampDelayMedium);
      else if (landSpeed == 3)
        delay(motorRampDelaySlow);
      else
        delay(motorRampDelayVerySlow);
    }
    resetMotorsPidOff();
    initialized = false;
    // Notify Matlab after landing
    if (matlab)
    {
      sendLandAck = true;
      numCommandToSend++;
    }
    //Serial.print("   finished");
    landing = false;
  }
  else
  {
    Serial.println();
    Serial.print("Land command received but Tenzo is not Flying   !! WARNING !!");
    Serial.println();
  }
}

void initialize()
{
  if (!initialized)
  {
    initializing = true;
    if (!autoEnablePid)
    {
      resetMotorsPidOff();
    }
    else
    {
      resetMotors();
      enablePid = true;
    }
    delay(500);
    for (int j=0; j<rampTill;j++)
    {
      motorSpeed(j);
      Serial.println(j);
      delay(motorRampDelayFast); 
    }
    checkpoint = millis();
    throttle=rampTill;
    initialized = true;
    // Notify Matlab after Take Off  
    if (matlab)
    {
      sendTakeOffAck =true;
      numCommandToSend++;
    }
    if (enablePid)
    {  
      changePidState(true);
    }
    else
    {
      changePidState(false);
    }
    initializing = false;
  }
  else
  {
    Serial.println();
    Serial.print("First Land ortherwise Tenzo will crash");
    Serial.println();
  }
}

void changePidState(boolean cond)
{
  if (matlab)
  {
    // If pid state is modified, notify Matlab
    /*
    if (enablePid == cond && matlab)
     {
     sendHoverState = false;
     }
     */
    /*
     * AEP + TO
     **/
    if (enablePid && cond && autoEnablePid) 
    {
      Serial.println();
      Serial.print("   eP + cond + AEP ");
      Serial.println();

      sendHoverState = true;
      numCommandToSend++;
    }
    else if (!enablePid && !cond && autoEnablePid)
    {    
      Serial.println();
      Serial.print("   !eP + !cond + AEP ");
      Serial.println();

      sendHoverState = true;
      numCommandToSend++;
    }
    /*
     * !AEP + TO
     **/
    else if (!enablePid && !cond && !autoEnablePid)
    {
      Serial.println();
      Serial.print("   !eP + !cond + !AEP ");
      Serial.println();

      sendHoverState = false;
    }
    /** 
     *  AEP + Land
     **/
    else if (enablePid && !cond && landing)
    {
      Serial.println();
      Serial.print("   eP + !cond + AEP + Land ");
      Serial.println();

      sendHoverState = true;
      numCommandToSend++;
    }
    else if (enablePid && !cond)
    {
      sendHoverState = true;
      numCommandToSend++;
    }
    else if (enablePid != cond )
    {
      sendHoverState = true;
      numCommandToSend++;
    }
  }
  if (cond)
  {
    // Enable Pid Actions
    myRollPID.SetMode(AUTOMATIC);
    //tell the PID to range between 0 and the full throttle
    //SetpointRoll = 0;
    myRollPID.SetOutputLimits(-500, 500);

    // Pitch
    myPitchPID.SetMode(AUTOMATIC);
    //SetpointPitch = 0;
    //tell the PID to range between 0 and the full throttle
    myPitchPID.SetOutputLimits(-500, 500);

    // Yaw
    myYawPID.SetMode(AUTOMATIC);
    //SetpointYaw=0;
    //tell the PID to range between 0 and the full throttle
    myYawPID.SetOutputLimits(-500, 500);
    enablePid = true;
  }
  else
  { 
    myRollPID.SetMode(MANUAL);
    myPitchPID.SetMode(MANUAL);
    myYawPID.SetMode(MANUAL);
    enablePid = false;
  }
}

void motorSpeedPID(int thr, int rollpid, int pitchpid, int yawpid)
{
  int motorA, motorB, motorC, motorD;

  // compute motor inputs
  motorA = thr + rollpid - yawpid;
  motorB = thr + pitchpid + yawpid;
  motorC = thr - rollpid - yawpid;
  motorD = thr - pitchpid + yawpid; 
  
  if (printMotorsVals)
  {
    Serial.println();
    Serial.print(" Motors:[  ");
    Serial.print(motorA);
    Serial.print("   ;   ");
    Serial.print(motorB);
    Serial.print("   ;   ");
    Serial.print(motorC);
    Serial.print("   ;   ");
    Serial.print(motorD);
    Serial.println("   ]");  
    Serial.print("        PID:[ROLL  ");
    Serial.print(rollpid);
    Serial.print("   ;PITCH   ");
    Serial.print(pitchpid);
    Serial.print("   ;YAW   ");
    Serial.print(yawpid);
    Serial.print("   ; THROTTLE    ");
    Serial.print(thr);
    Serial.print("   ]");
    Serial.println();
  }
  // send input to motors
  servo1.write(motorA);
  servo2.write(motorB);
  servo3.write(motorC);
  servo4.write(motorD);
}

void sendDataSensors(boolean device)
{
  if (!device)
  {
    // Not yet connected. 
    if (sendConnAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      numCommandToSend++;

      // Channel 31: Connection
      pMyCmd->cmd = connID;
      pMyCmd->param1 = 2; 
      sendConnAck = false;
      sendCommandToMatlab(); 
      if (printAckCommands)
      {
        Serial.println();
        Serial.println("Sending Connection Ack to Matlab: First Time");
        Serial.print(pMyCmd->param1);
        Serial.println(); 
      } 
     matlab = true;     
    }
   else if (sendDisconnAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      numCommandToSend++;

      // Channel 31: Disconnection
      pMyCmd->cmd = connID;
      pMyCmd->param1 = 10; 
      sendDisconnAck = false;
      matlab = false;
      if (printAckCommands)
      {
        Serial.println();
        Serial.println("Sending Disonnection Ack to Matlab: ");
        Serial.print(pMyCmd->param1);
        Serial.println(); 
      }      
      sendCommandToMatlab();      
    } 
  }
  else if (device)
  {    
    long cmdTmp1;
    long cmdTmp2;
    long cmdTmp3;
    long cmdTmp4;

    if (sendConnAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      numCommandToSend++;

      // Channel 31: Connection
      pMyCmd->cmd = connID;
      pMyCmd->param1 = 2; 
      sendConnAck = false;
      sendCommandToMatlab(); 
      if (printAckCommands)
      {
        Serial.println();
        Serial.println("Sending Connection Ack to Matlab: DOPO ");
        Serial.print(pMyCmd->param1);
        Serial.println(); 
      }      
    } 
    else if (sendDisconnAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      numCommandToSend++;

      // Channel 31: Disconnection
      pMyCmd->cmd = connID;
      pMyCmd->param1 = 10; 
      sendDisconnAck = false;
      matlab = false;
      if (printAckCommands)
      {
        Serial.println();
        Serial.println("Sending Disonnection Ack to Matlab: ");
        Serial.print(pMyCmd->param1);
        Serial.println(); 
      }      
      sendCommandToMatlab();      
    }
    else if (sendTenzoState)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);

      numCommandToSend++;
      pMyCmd->cmd = tenzoStateID;

      if (initialized)   
      {   
        pMyCmd->param1 = 1;
        pMyCmd->param3 = 0;     
      } 
      else
      {
        pMyCmd->param1 = 0;       
        pMyCmd->param3 = 1; 
      }

      if (enablePid)      
        pMyCmd->param2 = 1; 
      else
        pMyCmd->param2 = 0;   

      if (printAckCommands)
      {
        Serial.println();
        Serial.println("Sending Tenzo state to Matlab");
        Serial.print(" Take Off: ");
        Serial.println(pMyCmd->param1);
        Serial.print(" Hover:  ");
        Serial.println(pMyCmd->param2);
        Serial.print(" Land: ");
        Serial.print(pMyCmd->param3);
        Serial.println(); 
      } 

      sendCommandToMatlab();  
      sendTenzoState = false;
    }
    else if (storeAccData && accDataReady)
    {
      //////////////////////////// Send values to Matlab /// Done in other way
    }
    else if (sendHoverState && sendTakeOffAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);

      pMyCmd->cmd = takeOffID;

      if (initialized)      
        pMyCmd->param1 = 1; 
      else
        pMyCmd->param1 = 0;   

      sendTakeOffAck = false;
      if (printAckCommands)
      {
        Serial.println();
        Serial.print("Sending Ack to Matlab. TakeOffState:  ");
        Serial.print(pMyCmd->param1);
        Serial.println(); 
      } 

      pMyCmd++;           // Second Message

      // Channel 18: Hovering
      pMyCmd->cmd = iHoverID;
      if (enablePid)      
        pMyCmd->param1 = 1; 
      else
        pMyCmd->param1 = 0; 
      sendHoverState = false;

      if (printAckCommands)
      {
        Serial.println();
        Serial.print("Sending Ack to Matlab: Hover: ");
        Serial.print(pMyCmd->param1);
        Serial.println(); 
      } 

      sendCommandToMatlab();  
    }
    else if (sendTakeOffAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);

      pMyCmd->cmd = takeOffID;

      if (initialized)      
        pMyCmd->param1 = 1; 
      else
        pMyCmd->param1 = 0;   

      sendTakeOffAck = false;
      if (printAckCommands)
      {
        Serial.println();
        Serial.print("Sending Ack to Matlab. TakeOffState:  ");
        Serial.print(pMyCmd->param1);
        Serial.println(); 
      } 
      sendCommandToMatlab();  
    }
    else if (sendHoverState && sendLandAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      // Channel 18: Hovering
      pMyCmd->cmd = iHoverID;
      if (enablePid)      
        pMyCmd->param1 = 1; 
      else
        pMyCmd->param1 = 0; 
      sendHoverState = false;

      if (printAckCommands)
      {
        Serial.println();
        Serial.print("Sending Ack to Matlab: Hover: ");
        Serial.print(pMyCmd->param1);
        Serial.println(); 
      } 

      pMyCmd++;           // Second Message
      // Channel 19: Landing
      pMyCmd->cmd = landID;

      if (initialized)      
        pMyCmd->param1 = 0; 
      else
        pMyCmd->param1 = 1;   
      sendLandAck = false;

      if (printAckCommands)
      {
        Serial.println();
        Serial.print("Sending Ack to Matlab. Land state: 2  ");
        Serial.print(pMyCmd->param1);
        Serial.println(); 
      }      
      // Send two commands to Matlab. numCommandsToSend = 2;
      sendCommandToMatlab();  
    }   
    else if (sendHoverState)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      // Channel 18: Hovering
      pMyCmd->cmd = 18;
      if (enablePid)      
        pMyCmd->param1 = 1; 
      else
        pMyCmd->param1 = 0; 
      sendHoverState = false;
      if (printAckCommands)
      {
        Serial.println();
        Serial.print("Sending Ack to Matlab: Hover: ");
        Serial.print(pMyCmd->param1);
        Serial.println(); 
      }      
      sendCommandToMatlab();  
    }
    else if (sendLandAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      // Channel 19: Landing
      pMyCmd->cmd = landID;

      if (initialized)      
        pMyCmd->param1 = 0; 
      else
        pMyCmd->param1 = 1;   
      sendLandAck = false;
      sendCommandToMatlab();
      if (printAckCommands)
      {
        Serial.println();
        Serial.print("Sending Ack to Matlab. Land state:  ");
        Serial.print(pMyCmd->param1);
        Serial.println(); 
      } 

    }
    else if (sendAccToMatlab && matlab)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      pMyCmd->cmd = accID;

      pMyCmd->param1 = aax*100;   
      pMyCmd->param2 = aay*100;
      pMyCmd->param3 = aaz*100; 

      numCommandToSend++;

      sendCommandToMatlab();  

      if (printAckCommands)
      {
        Serial.println();
        Serial.print("Sending Ack to Matlab. [ax ay az]");
        Serial.println(pMyCmd->param1);
        Serial.println(pMyCmd->param2);
        Serial.println(pMyCmd->param3);
        Serial.println(); 
      } 
      sendAccToMatlab = false;
    }
    else if (sendGyroToMatlab  && matlab)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      pMyCmd->cmd = gyroID;

      pMyCmd->param1 = wx*100;   
      pMyCmd->param2 = wy*100;
      pMyCmd->param3 = wz*100; 
      numCommandToSend++;

      sendCommandToMatlab();  
      if (printAckCommands)
      {
        Serial.println();
        Serial.print("Sending Ack to Matlab. [wx wy wz]");
        Serial.println(pMyCmd->param1);
        Serial.println(pMyCmd->param2);
        Serial.println(pMyCmd->param3);
        Serial.println(); 
      } 
      sendGyroToMatlab = false;
    }
    else if (sendEstToMatlab && sendMagnToMatlab && matlab)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);

      numCommandToSend++;
      pMyCmd->cmd = magnID;

      pMyCmd->param1 = roll1;   
      pMyCmd->param2 = pitch1;
      pMyCmd->param3 = bearing1*100;
      if (printAckCommands)
      {
        Serial.println();
        Serial.print("Sending Ack to Matlab. [xM yM zM]");
        Serial.println(pMyCmd->param1);
        Serial.println(pMyCmd->param2);
        Serial.println(pMyCmd->param3);
        Serial.println(); 
      }

      pMyCmd++;           // moves pointer to next command position in message
      numCommandToSend++;
      pMyCmd->cmd = estID;

      pMyCmd->param1 = (int)yAngle;   
      pMyCmd->param2 = (int)xAngle;
      pMyCmd->param3 = (int)zAngle*100;

      sendCommandToMatlab(); 

      if (printAckCommands)
      {
        Serial.println();
        Serial.print("Sending Ack to Matlab. [xEst yEst zEst]");
        Serial.println(pMyCmd->param1);
        Serial.println(pMyCmd->param2);
        Serial.println(pMyCmd->param3);
        Serial.println(); 
      } 
      sendEstToMatlab = false;
      sendMagnToMatlab = false;
    }
    else if (sendMotorsToMatlab && matlab)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      pMyCmd->cmd = enableMotorsID;
      numCommandToSend++;

      pMyCmd->param1 = throttle; 
      if (printAckCommands)
      {
        Serial.println();
        Serial.print("Sending throttle to Matlab: ");
        Serial.println(pMyCmd->param1);
        Serial.println(); 
      } 
      sendCommandToMatlab();  
      sendMotorsToMatlab = false;
    }
    else if (sendPidState)
    {
      if (sendAltToMatlab)
      {
        MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
        pMyCmd->cmd = 12;
        numCommandToSend++;

        pMyCmd->param1 = consKpAltitude*1000;   
        pMyCmd->param2 = consKdAltitude*1000;
        pMyCmd->param3 = consKiAltitude*1000;
        pMyCmd->param4 = SetpointAltitude;
        if (printAckCommands)
        {
          Serial.println();
          Serial.print("Sending pid vals to Matlab. Alt cons");
          Serial.println(pMyCmd->param1);
          Serial.println(pMyCmd->param2);
          Serial.println(pMyCmd->param3);
          Serial.println(pMyCmd->param4);
          Serial.println(); 
        } 

        pMyCmd++;           // moves pointer to next command position in message

        pMyCmd->cmd = 16;
        numCommandToSend++;      

        pMyCmd->param1 = aggKpAltitude*1000;   
        pMyCmd->param2 = aggKdAltitude*1000;
        pMyCmd->param3 = aggKiAltitude*1000;

        pMyCmd->param4 = SetpointAltitude;
        if (printAckCommands)
        {
          Serial.println();
          Serial.print("Sending pid vals to Matlab. Alt agg");
          Serial.println(pMyCmd->param1);
          Serial.println(pMyCmd->param2);
          Serial.println(pMyCmd->param3);
          Serial.println(pMyCmd->param4);
          Serial.println(); 
        } 

        sendAltToMatlab = false;
      }
      else if (sendRollToMatlab)
      {
        MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
        pMyCmd->cmd = 9;
        numCommandToSend++;

        pMyCmd->param1 = consKpRoll*1000;   
        pMyCmd->param2 = consKdRoll*1000;
        pMyCmd->param3 = consKiRoll*1000;
        pMyCmd->param4 = SetpointRoll;
        if (printAckCommands)
        {
          Serial.println();
          Serial.print("Sending pid vals to Matlab. roll cons: ");
          Serial.println(pMyCmd->param1);
          Serial.println(pMyCmd->param2);
          Serial.println(pMyCmd->param3);
          Serial.println(pMyCmd->param4);
          Serial.println(); 
        } 

        pMyCmd++;           // moves pointer to next command position in message

        pMyCmd->cmd = 13;
        numCommandToSend++;

        pMyCmd->param1 = aggKpRoll*1000;   
        pMyCmd->param2 = aggKdRoll*1000;
        pMyCmd->param3 = aggKiRoll*1000;
        pMyCmd->param4 = SetpointRoll;
        if (printAckCommands)
        {
          Serial.println();
          Serial.print("Sending pid vals to Matlab. roll agg");
          Serial.println(pMyCmd->param1);
          Serial.println(pMyCmd->param2);
          Serial.println(pMyCmd->param3);
          Serial.println(pMyCmd->param4);
          Serial.println(); 
        }
        sendRollToMatlab = false;
      }
      else if (sendPitchToMatlab)
      {
        MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
        pMyCmd->cmd = 10;
        numCommandToSend++;

        pMyCmd->param1 = consKpPitch*1000;   
        pMyCmd->param2 = consKdPitch*1000;
        pMyCmd->param3 = consKiPitch*1000;
        pMyCmd->param4 = SetpointPitch;
        if (printAckCommands)
        {
          Serial.println();
          Serial.print("Sending pid vals to Matlab. pitch cons");
          Serial.println(pMyCmd->param1);
          Serial.println(pMyCmd->param2);
          Serial.println(pMyCmd->param3);
          Serial.println(pMyCmd->param4);
          Serial.println(); 
        } 

        pMyCmd++;           // moves pointer to next command position in message

        pMyCmd->cmd = 14;
        numCommandToSend++;

        pMyCmd->param1 = aggKpPitch*1000;   
        pMyCmd->param2 = aggKdPitch*1000;
        pMyCmd->param3 = aggKiPitch*1000;
        pMyCmd->param4 = SetpointPitch;
        if (printAckCommands)
        {
          Serial.println();
          Serial.print("Sending pid vals to Matlab. pitch agg");
          Serial.println(pMyCmd->param1);
          Serial.println(pMyCmd->param2);
          Serial.println(pMyCmd->param3);
          Serial.println(pMyCmd->param4);
          Serial.println(); 
        } 
        sendPitchToMatlab = false;
      }
      else if (sendYawToMatlab)
      {
        MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
        pMyCmd->cmd = 11;
        numCommandToSend++;

        pMyCmd->param1 = consKpYaw*1000;   
        pMyCmd->param2 = consKiYaw*1000;
        pMyCmd->param3 = consKiYaw*1000;
        pMyCmd->param4 = SetpointYaw;
        if (printAckCommands)
        {
          Serial.println();
          Serial.print("Sending pid vals to Matlab. Yaw cons");
          Serial.println(pMyCmd->param1);
          Serial.println(pMyCmd->param2);
          Serial.println(pMyCmd->param3);
          Serial.println(pMyCmd->param4);
          Serial.println(); 
        } 

        pMyCmd++;           // moves pointer to next command position in message

        pMyCmd->cmd = 15;
        numCommandToSend++;

        pMyCmd->param1 = aggKpYaw*1000;   
        pMyCmd->param2 = aggKdYaw*1000;
        pMyCmd->param3 = aggKiYaw*1000;
        pMyCmd->param4 = SetpointYaw;
        if (printAckCommands)
        {
          Serial.println();
          Serial.print("Sending pid vals to Matlab. yaw agg");
          Serial.println(pMyCmd->param1);
          Serial.println(pMyCmd->param2);
          Serial.println(pMyCmd->param3);
          Serial.println(pMyCmd->param4);
          Serial.println(); 
        } 

        sendYawToMatlab = false;
      }
      sendPidState = false;
      sendCommandToMatlab(); 
    }  
  } 
}

void sendCommandToMatlab()
{
  // Build Header
  pCtrlHdr->srcAddr = 1;
  pCtrlHdr->dstAddr = 2;    // maybe you'll make 2555 a broadcast address? 
  pCtrlHdr->versionX = versionArduinoProtocol;    // possible way to let a receiver know a code version
  pCtrlHdr->numCmds = numCommandToSend;    // how many commands will be in the message
  if (printVerboseSerial)
  {
    Serial.println();
    Serial.print("Sending # commands:");
    Serial.print(numCommandToSend);
    Serial.println();
  }
  pCtrlHdr->hdrLength = sizeof(MyControlHdr );  // tells receiver where commands start
  pCtrlHdr->cmdLength = sizeof(MyCommand );     // tells receiver size of each command 
  // include total length of entire message
  pCtrlHdr->totalLen= sizeof(MyControlHdr ) + (sizeof(MyCommand) * pCtrlHdr->numCmds);
  if (printVerboseSerial)
  {
    Serial.println();
    Serial.print("Total length: ");
    Serial.print(pCtrlHdr->totalLen);
    Serial.println();
  }
  pCtrlHdr->crc = 21;   // dummy temp value

  for (int v=0;v<=sizeof(buffer);v++)
  {
    xbee.write(buffer[v]);  
  }
  numCommandToSend = 0;
}

void testMotor(int x)
{   
  //Serial.print(" Testing motor: ");
  Serial.println(x);
  if (x==1)
  {
    Serial.println('M1');
    //Serial.println(" Increasing angular velocity");
    servo1.write(40);
    delay(500);
    servo1.write(60);
    delay(500);
    servo1.write(100);
    //Serial.println(" Decreasing angular velocity");
    delay(500);
    servo1.write(40);
    delay(500);
    servo1.write(0);
  }
  else if (x==2)
  {
    Serial.println('M2');
    //Serial.println(" Increasing angular velocity");
    servo2.write(40);
    delay(500);
    servo2.write(60);
    delay(500);
    servo2.write(100);
    //Serial.println(" Decreasing angular velocity");
    delay(500);
    servo2.write(40);
    delay(500);
    servo2.write(0);
  }
  else if (x==3)
  {
    Serial.println('M3');
    //Serial.println(" Increasing angular velocity");
    servo3.write(40);
    delay(500);
    servo3.write(60);
    delay(500);
    servo3.write(100);
    //Serial.println(" Decreasing angular velocity");
    delay(500);
    servo3.write(40);
    delay(500);
    servo3.write(0);
  }
  else if (x==4)
  {
    Serial.println('M4');
    //Serial.println(" Increasing angular velocity");
    servo4.write(0);
    delay(500);
    servo4.write(20);
    delay(500);
    servo4.write(40);
    delay(500);
    servo4.write(60);
    delay(500);
    servo4.write(80);
    delay(500);
    servo4.write(100);
    delay(500);
    servo4.write(120);
    //Serial.println(" Decreasing angular velocity");
    delay(100);
    servo4.write(60);
    delay(500);
    servo4.write(40);
    delay(500);
    servo4.write(0);
  }
}

void dispAllPidVals()
{
  Serial.println();
  Serial.print(" Updating PID values");
  Serial.println();
  if (enableRollPid)
  {
    Serial.println(" Roll:  ");
    Serial.print("   Cons [p,d,i]: [");
    Serial.print(consKpRoll);
    Serial.print(" ; ");
    Serial.print(consKdRoll);
    Serial.print(" ; ");
    Serial.print(consKiRoll);
    Serial.print(" ]   Agg: [p,d,i]: [");
    Serial.print(aggKpRoll);
    Serial.print(" ; ");
    Serial.print(aggKdRoll);
    Serial.print(" ; ");
    Serial.print(aggKiRoll);
    Serial.print(" ]   Setpoint: ");
    Serial.println(SetpointRoll);
  }
  if (enablePitchPid)
  {
    Serial.println(" Pitch:  ");
    Serial.print("   Cons [p,d,i]: [");
    Serial.print(consKpPitch);
    Serial.print(" ; ");
    Serial.print(consKdPitch);
    Serial.print(" ; ");
    Serial.print(consKiPitch);
    Serial.print(" ]   Agg: [p,d,i]: [");
    Serial.print(aggKpPitch);
    Serial.print(" ; ");
    Serial.print(aggKdPitch);
    Serial.print(" ; ");
    Serial.print(aggKiPitch);
    Serial.print(" ]   Setpoint: ");
    Serial.println(SetpointPitch);
  }
  if (enableYawPid)
  {
    Serial.println(" Yaw:  ");
    Serial.print("   Cons [p,d,i]: [");
    Serial.print(consKpYaw);
    Serial.print(" ; ");
    Serial.print(consKdYaw);
    Serial.print(" ; ");
    Serial.print(consKiYaw);
    Serial.print(" ]   Agg: [p,d,i]: [");
    Serial.print(aggKpYaw);
    Serial.print(" ; ");
    Serial.print(aggKdYaw);
    Serial.print(" ; ");
    Serial.print(aggKiYaw);
    Serial.print(" ]   Setpoint: ");
    Serial.println(SetpointYaw);
  }
  if (enableAltitudePid)
  {
    Serial.println(" Alt:  ");
    Serial.print("   Cons [p,d,i]: [");
    Serial.print(consKpAltitude);
    Serial.print(" ; ");
    Serial.print(consKdAltitude);
    Serial.print(" ; ");
    Serial.print(consKiAltitude);
    Serial.print(" ]   Agg: [p,d,i]: [");
    Serial.print(aggKpAltitude);
    Serial.print(" ; ");
    Serial.print(aggKdAltitude);
    Serial.print(" ; ");
    Serial.print(aggKiAltitude);
    Serial.print(" ]   Setpoint: ");
    Serial.println(SetpointAltitude);
  }
  Serial.println();
}

void dispActualAllPidVals()
{
  Serial.println();
  Serial.print(" Updating PID values");
  Serial.println();
  if (enableRollPid)
  {
    Serial.println(" Roll:  ");
    Serial.print("   [p,d,i]: [");
    Serial.print(myRollPID.GetKp());
    Serial.print(" ; ");
    Serial.print(myRollPID.GetKd());
    Serial.print(" ; ");
    Serial.print(myRollPID.GetKi());
    Serial.print(" ]   Setpoint: ");
    Serial.println(SetpointRoll);
  }
  if (enablePitchPid)
  {
    Serial.println(" Pitch:  ");
    Serial.print("   [p,d,i]: [");
    Serial.print(myPitchPID.GetKp());
    Serial.print(" ; ");
    Serial.print(myPitchPID.GetKd());
    Serial.print(" ; ");
    Serial.print(myPitchPID.GetKi());
    Serial.print(" ]   Setpoint: ");
    Serial.println(SetpointPitch);
  }
  if (enableYawPid)
  {
    Serial.println(" Yaw:  ");
    Serial.print("  [p,d,i]: [");
    Serial.print(myYawPID.GetKp());
    Serial.print(" ; ");
    Serial.print(myPitchPID.GetKd());
    Serial.print(" ; ");
    Serial.print(myPitchPID.GetKi());
    Serial.print(" ]   Setpoint: ");
    Serial.println(SetpointYaw);
  }
  if (enableAltitudePid)
  {
    Serial.println(" Alt:  ");
    Serial.print("   [p,d,i]: [");
    Serial.print(myAltitudePID.GetKp());
    Serial.print(" ; ");
    Serial.print(myAltitudePID.GetKd());
    Serial.print(" ; ");
    Serial.print(myAltitudePID.GetKi());
    Serial.print(" ]   Setpoint: ");
    Serial.println(SetpointAltitude);
  }
  Serial.println();
}

/*
ISR(TIMER3_COMPB_vect)
{
    accRoutine();
}
*/

void displayValues(int b, int p, int r)
{    
  if (printRawCompass)
  {
    Serial.println();
    Serial.print(" Compass: [R, P, Y] = [");
    Serial.print(r);  
    Serial.print(" , ");
    Serial.print(p);
    Serial.print(" , ");
    Serial.print(b);
    Serial.print(" ]");
    Serial.println();
  }
}

void disableSerialOut()
{
  printSetupInfo = false;
  printAckCommands = false;
  printVerboseSerial = false;
  printPIDVals = false;
  printRawKalman= false;
  printRawAcc = false;
  printAcc = false;
  printRawGyro= false;
  printGyro = false;
  printRawCompass= false;
  printCompass = false;
  printKalman = false; 
  printMotorsVals = false;
  printThrottle = false;
}
