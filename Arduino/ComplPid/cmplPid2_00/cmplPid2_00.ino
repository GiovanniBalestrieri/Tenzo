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
 *
 **/

#include <Wire.h>
#include <Servo.h>
#include <CMPS10.h>
#include <SoftwareSerial.h>
#include "PID_v2.h"

boolean printBlue = false;
boolean processing = false;
boolean printMotorsVals = false;
boolean printPIDVals = false;
boolean printSerialInfo = false;
boolean printSerial = false;
boolean printTimers = false; // true
boolean printAccs = false;
boolean printOmegas = false;
boolean sendBlueAngle = false;
boolean printVerboseSerial = false;

/**
 * Modes
 */
int connAck = 0;
int takeOff = 0;
int hovering = 0;
int landed = 1;
int tracking = 0;
int warning = 0;

byte modeS;

boolean statusChange = false; // remove

/**
 * VTOL settings
 */
// Take Off settings
int rampTill = 1300;
int motorRampDelayFast = 2;
int motorRampDelayMedium = 5;
int motorRampDelaySlow = 15;
int motorRampDelayVerySlow = 20;

// Safe Mode: after timeToLand ms tenzo will land automatically
unsigned long timeToLand = 20000;
boolean autoLand = false;
boolean landing = false;
int landSpeed = 1;
// Landing protocol variables
unsigned long timerStart;
unsigned long checkpoint;

// Keep track of the state
boolean initializing = false;
boolean initialized = false;

/**
 *  Brushless 
 */

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
volatile int throttle = 0;
volatile int motorA, motorB, motorC, motorD;

// Motor constant
int thresholdUp=255, thresholdDown=1;
int maxTest = 1300, minTest = 700;
/** 
 ** Control
 **/
float Kmy = 1, Kw = 3.7;
//float OutputRoll = 0, OutputPitch = 0, OutputYaw = 0, OutputAlt = 0;
/**
 * Pid Controller 
 */
boolean autoEnablePid = true;
boolean enablePid = false;
boolean enableRollPid = false;
boolean enablePitchPid = false;
boolean enableYawPid = false;
boolean enableWRollPid = true;
boolean enableWPitchPid = false;
boolean enableWYawPid = false;
boolean enableAltitudePid = false;

// Define IO and setpoint for control
double SetpointRoll = 0, InputRoll, errorRoll;
double SetpointPitch = 0, InputPitch, errorPitch;
double SetpointYaw = 180, InputYaw, errorYaw;
double SetpointAltitude = 1, InputAltitude, errorAltitude;


// Define IO and setpoint for control -----------  W
double SetpointWRoll = 0, InputWRoll, errorWRoll;
double SetpointWPitch = 0, InputWPitch, errorWPitch;
double SetpointWYaw = 180, InputWYaw, errorWYaw;
//
//volatile double OutputRoll;
//volatile double OutputPitch;
//volatile double OutputYaw;
//volatile double OutputAltitude;

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

// Altitude  ->> *** Add it, just created
double aggKpAltitude=0.2, aggKiAltitude=0.0, aggKdAltitude=0.1;
double consKpAltitude=0.1, consKiAltitude=0, consKdAltitude=0.1;

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

/**
 * Compass
 */

CMPS10 my_compass;
float angPosFilter[3];
int pitch1;
int roll1;
//float bearing1;
int filterAng = 0;

/**
 ** Gyro
 **/

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24


#define STATUS_REG 0x27
#define ZOR_REG 0b01000000
#define ZDA_REG 0b00000100
#define YOR_REG 0b00100000
#define YDA_REG 0b00000010
#define XOR_REG 0b00010000
#define XDA_REG 0b00000001

//use address 104 if CS is not connected
int L3G4200D_Address = 105; 

int zOld, xOld, yOld, xCand, yCand, zCand;
int threshold = 200;

float scale2000 = 70;

float bx= 0,by=0,bz=0;
long bxS,byS,bzS;

unsigned long biasCalcTime = 0;

/**
 ** Acc 
 **/

const float RESOLUTION = 800; //0.8 v/g -> resolucion de 1.5g -> 800mV/g
const float VOLTAGE = 3.3;    //voltage al que está conectado el acelerómetro

const float ZOUT_1G = 850;    // mv Voltage en Zout a 1G

const int NADJ = 50;          // Número de lecturas para calcular el error

// Entradas analógicas donde van los sensores
const int xaxis = 0;
const int yaxis = 1;
const int zaxis = 2;

float XError,YError,ZError;

// Acc Timers
volatile unsigned long accTimer;
volatile unsigned long lastAccTimer;
volatile unsigned long timeToRead = 0;
volatile unsigned long lastTimeToRead = 0;
volatile unsigned long servoTime = 0;

// delta T control the routine frequency
float deltaT = 1;
float timerLoop = 0, timerReading = 0, timerSec = 0;
float timerRoutine = 0, count = 0, countWatch = 0;
float redingTime = 0, samplingTime = 0, calcTime =0, printTime = 0;
float k=0, kM1=0, kMReading = 0, kMRoutine=0, kMLoop=0, secRoutine=0;

byte mode;

// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

/**
 ** Serial 
 **/
int BaudRateSerial = 115200;
// Bluetooth
int BlueRate = 115200; // Slow down in case
// Gps
int BaudRateGps = 4800;

int pinRx = 12 , pinTx = 13; // the pin on Arduino
SoftwareSerial blu(pinRx, pinTx);
byte loBytew1, hiBytew1,loBytew2, hiBytew2;
int loWord,hiWord;

int printBlueAngleCounter = 0;
int printBlueAngleSkip = 5;

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
 * Print sensor's value
 */
boolean printSetupInfo = true;
boolean printAckCommands = true;
boolean printRawKalman= false;
boolean printRawAcc = false;
boolean printRawGyro= false;
boolean printGyro = false;
boolean printRawCompass= false;
boolean printCompass = false;
boolean printKalman = false; 
boolean printThrottle = false;

// Serial Protocol
int versionArduinoProtocol = 6;
boolean matlab = false;
int modeserial = 0;
long cmd1;
long cmd2;
long cmd3;
long cmd4;
int numCommandToSend = 0;
int numFilterValuesToSend = 0;

int inputBuffSize = 26;
int outputBuffSize = 30;
byte bufferBytes[22];

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
int headerLength = 8;//13;

typedef struct mcTag {  // 8 bytes for versions < 6 it was 13 bytes
  unsigned char srcAddr;
  unsigned char dstAddr;
  unsigned char versionX;  //long
  unsigned char numCmds;
  unsigned char hdrLength;
  unsigned char cmdLength;
  unsigned char totalLen; // short
  unsigned char crc;  // short
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


// Volatile vars
volatile int cont = 0;
volatile int countCtrlAction = 0;
volatile int contSamples=0;
volatile int contCalc=0;

volatile float thetaOLD = 0;
volatile float phi=0;
volatile float theta=0;
volatile float psi=0;
volatile int x = 0;
volatile int y = 0;
volatile int z = 0;
volatile int rawAx = 0;
volatile int rawAy = 0;
volatile int rawAz = 0;
volatile int dt=0;

volatile float wF[3];
volatile float aF[3];
volatile boolean filterGyro = false;
volatile boolean filterAcc = false;
volatile boolean initializedSetup = false;

volatile float bearing1;

volatile float angleXAcc;
volatile float angleYAcc;

volatile float aax,aay,aaz;
volatile float alphaA = 0.2, alphaW = 0.8;
volatile float estXAngle = 0, estYAngle = 0, estZAngle = 0;
volatile float kG = 0.98, kA = 0.02, kGZ=0.60, kAZ = 0.40;

void setup()
{  
  Wire.begin();
  Serial.begin(115200); 

  pinMode(xaxis,INPUT);
  pinMode(yaxis,INPUT);
  pinMode(zaxis,INPUT);

  /** 
   ** Servo initialization
   **/

  servo1.attach(3);  
  servo2.attach(5);    
  servo3.attach(22);   
  servo4.attach(9);

  servo1.writeMicroseconds(1000);
  servo2.writeMicroseconds(1000);
  servo3.writeMicroseconds(1000);
  servo4.writeMicroseconds(1000);

  if (!processing && !printBlue)
  {
    Serial.println("Starting up L3G4200D");
  }

  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  delay(1500); //wait for the sensor to be ready   

  biasCalcTime = micros();
  calcBias();
  biasCalcTime = micros() - biasCalcTime;
  /*
  if (!processing && !printBlue)
   {
   Serial.print("Read: [us] ");
   Serial.print(samplingTime);
   Serial.print("      Bias est: [us] ");
   Serial.print(biasCalcTime);
   Serial.print("      Samples: ");
   Serial.println(contSamples);
   }
   */
  XError =  AccelAdjust(xaxis);
  YError =  AccelAdjust(yaxis);
  ZError =  AccelAdjust(zaxis);
  ZError = ZError - ZOUT_1G;

  // Timer settings
  // Initialize Timer
  cli();
  TCCR3A = 0;
  TCCR3B = 0;

  // Set compare match register to the desired timer count
  //OCR3A=77; //16*10^6/(200Hz*1024)-1 = 77 -> 200 Hz 
  OCR3A=193; //16*10^6/(80Hz*1024)-1 = 193 -> 80 Hz 
  //OCR3A=780; //16*10^6/(20Hz*1024)-1 = 780 -> 20 Hz 
  //OCR3A=50; //16*10^6/(308Hz*1024)-1 = 50 -> 308 Hz 

  TCCR3B |= (1 << WGM32);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR3B |= (1 << CS30) | (1 << CS32);
  // enable timer compare interrupt:
  TIMSK3 |= (1 << OCIE3B);

  // enable global interrupts:
  sei(); 

  // ADC Tuning
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library
  // you can choose a prescaler from above.
  // PS_16, PS_32, PS_64 or PS_128
  ADCSRA |= PS_32;    // set our own prescaler to 64 

  initializedSetup = true;
  k = micros();
}

void loop()
{  
  while(true)
  {
    serialRoutine();   
    sendDataSensors(matlab);
    //delay(20);
  }
} 

ISR(TIMER3_COMPB_vect)
{  
  // Mettilo altrimenti non funziona
  sei();
  volatile byte statusflag = readRegister(L3G4200D_Address, STATUS_REG);
  while(!(statusflag & ZDA_REG) && (statusflag & ZOR_REG)&&!(statusflag & YDA_REG) && (statusflag & YOR_REG)&& !(statusflag & XDA_REG) && (statusflag & XOR_REG)) 
  {
    statusflag = readRegister(L3G4200D_Address, STATUS_REG);
  }
  //read values
  getGyroValues();
  getAcc();
  //getCompassValues();
  calcAngle();
  estAngle();

  cont++;
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
      if (printAckCommands && !printBlue)
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
      if (printAckCommands && !printBlue)
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
      if (printAckCommands && !printBlue)
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
      if (printAckCommands && !printBlue)
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

      if (printAckCommands && !printBlue)
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
      if (printAckCommands && !printBlue)
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

      if (printAckCommands && !printBlue)
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
      if (printAckCommands && !printBlue)
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

      if (printAckCommands && !printBlue)
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

      if (printAckCommands && !printBlue)
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
      if (printAckCommands && !printBlue)
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
      if (printAckCommands && !printBlue)
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

      if (printAckCommands && !printBlue)
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

      pMyCmd->param1 = x*100;   
      pMyCmd->param2 = y*100;
      pMyCmd->param3 = z*100; 
      numCommandToSend++;

      sendCommandToMatlab();  
      if (printAckCommands && !printBlue)
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

      pMyCmd->param1 = phi;   
      pMyCmd->param2 = theta;
      pMyCmd->param3 = bearing1*100;
      if (printAckCommands && !printBlue)
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

      pMyCmd->param1 = (int)estYAngle;   
      pMyCmd->param2 = (int)estXAngle;
      pMyCmd->param3 = (int)estZAngle*100;

      sendCommandToMatlab(); 

      if (printAckCommands && !printBlue)
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
      if (printAckCommands && !printBlue)
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
        if (printAckCommands && !printBlue)
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
        if (printAckCommands && !printBlue)
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
        if (printAckCommands && !printBlue)
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
        if (printAckCommands && !printBlue)
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
        if (printAckCommands && !printBlue)
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
        if (printAckCommands  && !printBlue)
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
        if (printAckCommands && !printBlue)
        {
          Serial.println();
          Serial.print("Sending pid vals to client. Yaw cons");
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
        if (printAckCommands && !printBlue)
        {
          Serial.println();
          Serial.print("Sending pid vals to client. yaw agg");
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
  if (!printBlue)
  {
    Serial.println();
    Serial.print("Sending # commands:");
    Serial.print(numCommandToSend);
    Serial.println();
  }
  pCtrlHdr->hdrLength = sizeof(MyControlHdr );  // tells receiver where commands start
  pCtrlHdr->cmdLength = sizeof(MyCommand );     // tells receiver size of each command 
  // include total length of entire message
  pCtrlHdr->totalLen = sizeof(MyControlHdr ) + (sizeof(MyCommand) * pCtrlHdr->numCmds);
  if (!printBlue)
  {
    Serial.println();
    Serial.print("Total length: ");
    Serial.print(pCtrlHdr->totalLen);
    Serial.println();
  }
  pCtrlHdr->crc = 21;   // dummy temp value

  for (int v=0;v<=sizeof(buffer);v++)
  {
    Serial.write(buffer[v]);  
  }
  numCommandToSend = 0;
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
    if(!processing && !printBlue)
    {
      Serial.println();
      Serial.print("Landing protocol started...");
      Serial.print(throttle);
      Serial.print(" ");
    }
    for (int j=throttle; j>700 ;j--)
    {
      motorSpeedPID(j, OutputPitch, OutputRoll, OutputYaw, OutputAltitude);
      //motorSpeed(j);
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
    //Serial.print("   finished");
    landing = false;
    // updateStates
    landed=1;
    takeOff=0;
    hovering=0;
  }
  else
  {
    Serial.println();
    Serial.print("Land command received but Tenzo is not Flying   !! WARNING !!");
    Serial.println();
  }
}

void resetMotorsPidOff()
{
  throttle = 1000;
  motorSpeed(1000);
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

void changePidState(boolean cond)
{
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
    wYawPID.SetMode(AUTOMATIC);
    //SetpointYaw=0;
    //tell the PID to range between 0 and the full throttle
    wYawPID.SetOutputLimits(-500, 500);

    // Enable Pid Actions
    wRollPID.SetMode(AUTOMATIC);
    //tell the PID to range between 0 and the full throttle
    //SetpointRoll = 0;
    wRollPID.SetOutputLimits(-500, 500);

    // Pitch
    wPitchPID.SetMode(AUTOMATIC);
    //SetpointPitch = 0;
    //tell the PID to range between 0 and the full throttle
    wPitchPID.SetOutputLimits(-500, 500);

    // Yaw
    wYawPID.SetMode(AUTOMATIC);
    //SetpointYaw=0;
    //tell the PID to range between 0 and the full throttle
    wYawPID.SetOutputLimits(-500, 500);

    enablePid = true;
  }
  else
  { 
    myRollPID.SetMode(MANUAL);
    myPitchPID.SetMode(MANUAL);
    myYawPID.SetMode(MANUAL);
    wRollPID.SetMode(MANUAL);
    wPitchPID.SetMode(MANUAL);
    wYawPID.SetMode(MANUAL);
    enablePid = false;
  }
}

void motorSpeedPID(int thr, float rollpid, float pitchpid, float yawpid, float altpid)
{
  // compute motor inputs
  motorA = thr + altpid + rollpid - yawpid;
  motorB = thr + altpid + pitchpid + yawpid;
  motorC = thr + altpid - rollpid - yawpid;
  motorD = thr + altpid - pitchpid + yawpid; 

  if (motorA>2000)
    motorA = 2000;
  if (motorB>2000)
    motorB = 2000;
  if (motorC>2000)
    motorC = 2000;
  if (motorD>2000)
    motorD = 2000;

  // send input to motors
  servo1.writeMicroseconds(motorA);
  servo2.writeMicroseconds(motorB);
  servo3.writeMicroseconds(motorC);
  servo4.writeMicroseconds(motorD);

  // Increase control counter 
  countCtrlAction++;
}

void initialize()
{
  if (!initialized)
  {
    if (!processing && !printBlue)
      Serial.println("Initializing");
    initializing = true;
    resetMotors();
    delay(500);
    for (int j=700; j<rampTill; j++)
    {
      motorSpeedPID(j, OutputPitch, OutputRoll, OutputYaw, OutputAltitude);
      //if (!processing)
      if (!printBlue)
        Serial.println(j);
      delay(motorRampDelayFast); 
    }
    throttle=rampTill;

    if (enablePid)
    {  
      changePidState(true);
    }
    else
    {
      changePidState(false);
    }

    checkpoint = millis();
    initialized = true;    
    initializing = false;
    // updateStates
    takeOff = 1;
    landed = 0;
    hovering = 1;
  }
  else
  {
    Serial.println();
    Serial.print("First Land ortherwise Tenzo will crash");
    Serial.println();
  }
}

void initializeFast()
{
  for (int j=1000; j<rampTill;j++)
  {
    motorSpeed(j);
    //Serial.println(j);
    delay(motorRampDelayFast); 
  }
  throttle=rampTill;
}

void motorSpeed(int x)
{    
  servo1.writeMicroseconds(x);      
  servo2.writeMicroseconds(x); 
  servo3.writeMicroseconds(x); 
  servo4.writeMicroseconds(x); 
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

void getCompassValues()
{
  bearing1 =  my_compass.bearing();
}

void getAcc() //ISR
{
  rawAx=analogRead(xaxis);
  rawAy=analogRead(yaxis);
  rawAz=analogRead(zaxis);

  aax = (((rawAx*5000.0)/1023.0)-XError)/RESOLUTION;
  aay = (((rawAy*5000.0)/1023.0)-YError)/RESOLUTION;
  aaz = (((rawAz*5000.0)/1023.0)-ZError)/RESOLUTION;

  if (filterAcc)
  {      
    aF[0] = aax;
    aF[1] = aay;
    aF[2] = aaz;
    aFilter(aF);
  }
  // gets the value sample time
  accTimer = micros() - lastAccTimer;
  // updates last reading timer
  lastAccTimer = micros();  
}

void calcAngle() //ISR
{
  // From Gyro
  dt = micros()-k;
  if (!filterGyro)
  {
    phi=phi+(float) x*/*scale2000/1000**/(float)dt/1000000.0;
    /*
    Serial.println();
     Serial.print("phi: ");
     Serial.println(phi);
     */
    thetaOLD = theta; 
    theta=theta+(float) y*/**scale2000/1000**/ (float) dt/1000000.0;

    psi=psi+(float) z*/*scale2000/1000**/ (float) dt/1000000.0; 
  }
  else if (filterGyro)
  {  
    phi=phi+wF[0]*/*(float)scale2000/1000**/ (float)dt/1000000.0;

    thetaOLD = theta;
    theta=theta+wF[1]*/*(float)scale2000/1000 **/(float) dt/1000000.0;

    psi=psi+wF[2]*/*(float)scale2000/1000**/ (float) dt/1000000.0;  
  }

  if (filterAcc)
  {
    angleXAcc = (atan2(-aF[0],aF[2])) * RAD_TO_DEG;
    angleYAcc = (atan2(aF[1],aF[2])) * RAD_TO_DEG;
  }
  else
  {
    // From Acc
    angleXAcc = (atan2(-aax,aaz)) * RAD_TO_DEG;
    angleYAcc = (atan2(aay,aaz)) * RAD_TO_DEG;
  }
  k=micros();  
}

void estAngle()
{
  estXAngle = (estXAngle + x/**scale2000/1000*/*(float)dt/1000000.0)*kG + angleXAcc*kA;
  estYAngle = (estYAngle + y/**scale2000/1000*/*(float)dt/1000000.0)*kG + angleYAcc*kA;
  //estZAngle = psi*KG + yaw*KA;
}

void wFilter(volatile float val[])
{
  val[0] = (1-alphaW)*x + alphaW*val[0];
  val[1] = (1-alphaW)*y + alphaW*val[1];
  val[2] = (1-alphaW)*z + alphaW*val[2];
}

void aFilter(volatile float val[])
{
  val[0] = (1-alphaA)*aax + alphaW*val[0];
  val[1] = (1-alphaA)*aay + alphaW*val[1];
  val[2] = (1-alphaA)*aaz + alphaW*val[2];
}

void sendStatus()
{
  if (connAck && printBlue)
  {
    Serial.print("s,");
    Serial.print(takeOff);
    Serial.print(",");
    Serial.print(landed);
    Serial.print(",");
    if (enablePid)
      Serial.print(1);
    else if (!enablePid)  
      Serial.print(0);      
    Serial.print(",");
    Serial.println(warning);    
  }
}

void sendPidStatus(int type, int control, int action)
{
  if (connAck && printBlue)
  {
    Serial.print("s,");
    // Send
    //  1: pid
    //  0: General
    Serial.print("1,");
    Serial.print(type);    
    Serial.print(",");
    Serial.print(control);
    Serial.print(",");    
    Serial.println(action);
  }
}

void sendPidStatus(int type, int control, int action,float kp, float ki, float kd)
{
  if (connAck && printBlue)
  {
    Serial.print("s,");
    // Send
    //  1: pid
    //  0: General
    Serial.print("1,");
    Serial.print(type);    
    Serial.print(",");
    Serial.print(control);
    Serial.print(",");    
    Serial.print(action);
    Serial.print(",");    
    Serial.print(kp);
    Serial.print(",");    
    Serial.print(ki);
    Serial.print(",");    
    Serial.print(kd);
    Serial.println(","); 
  }
}

void resetStatus()
{
  sendBlueAngle = false;
}

void watchdog()
{
  Serial.println("w"); 
}

void serialRoutine()
{
  countWatch += 1;
  if (countWatch >= 100000 && matlab)
  {
    countWatch = 0;
    watchdog();
  }
  if (Serial.available()) //inputBuffSize
  {                
    Serial.print("Received: ");
    Serial.println(Serial.available());
    for (int j=0;j<=inputBuffSize;j++)
    {
      bufferBytes[j] = Serial.read();

      if (printVerboseSerial && !printBlue)
      {
        //Serial.print("Received: ");
        Serial.print(bufferBytes[j]); // Ok that works
        Serial.println();
        delay(3);
      }
    }
    //Serial.println("K");


    for (int j=0;j<=inputBuffSize;j++)
    {
      Serial.println(bufferBytes[j]);
      delay(10);
    }  

    boolean temp = false;
    if (bufferBytes[0] == 2 && bufferBytes[1]==1 && temp)
    {        
      Serial.println("K");
      /**
       * Message has been correctly sent by Android and delivered to the Arduino 
       * Decoding Header
       **/

      // Assembling VERSION long skip first two bytes
      // previous version was 2 bytes value
      // now, just one 
      //hiBytew2 = bufferBytes[4];
      ////Serial.println(hiBytew2,BIN);
      //loBytew2 = bufferBytes[5];
      ////Serial.println(loBytew2,BIN);
      //hiWord = word(hiBytew2, loBytew2);
      ////versionProtocol = makeLong( hiWord, loWord);
      //int versionProtocol = hiWord;
      int versionProtocol = bufferBytes[2];
      //Serial.println(versionProtocol);
      //  number cmd
      int numCmd = bufferBytes[3];
      //Serial.println(bufferBytes[3]);
      //Serial.println(versionProtocol);
      int headL = bufferBytes[4];
      //Serial.println(headL);
      int cmdL = bufferBytes[5];
      //Serial.println(cmdL);
      //Serial.println(bufferBytes[5]);
      //  total Length
      //hiBytew2 = bufferBytes[9];
      ////Serial.println(hiBytew2,BIN);
      //loBytew2 = bufferBytes[10];
      //Serial.println(loBytew2,BIN);
      //short totL = word(hiBytew2, loBytew2);
      int totL = bufferBytes[6];
      //Serial.println(totL);
      // CRC
      //hiBytew2 = bufferBytes[11];
      //// Serial.println(hiBytew2,BIN);
      //loBytew2 = bufferBytes[12];
      //// Serial.println(loBytew2,BIN);
      //short crc = word(hiBytew2, loBytew2);
      int crc = bufferBytes[7];
      // Serial.println(crc);
      /**
       * Decoding Command
       **/
      int type = bufferBytes[8];
      Serial.print("type  :");
      Serial.println(type);
      if (printVerboseSerial && !printBlue)
      {
        Serial.println();
        Serial.print("Version");
        Serial.print(versionProtocol);
        if (versionProtocol != versionArduinoProtocol && !printBlue)
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
        //Serial.println("Connection");
        // Connection channel
        int alto = bufferBytes[9];
        Serial.println(alto);
        int basso = bufferBytes[10];
        Serial.println(basso);
        hiBytew2 = bufferBytes[11];
        Serial.println(hiBytew2);
        loBytew2 = bufferBytes[12];
        Serial.println(loBytew2);
        float conAck = *((float*)(bufferBytes + 9));

        //float  conAck1 = (speedLong << 8) | speedArray[i]
        Serial.print("RCon 1:  "); 
        Serial.println(conAck); 
        //conAck = word(hiBytew2, loBytew2);
        //Serial.println(conAck); 

        if (conAck == 1)
        {
          Serial.println("ConAck = 1");
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
            if (printVerboseSerial && !printBlue)
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
          Serial.println("ConAck 3");
          // Communication problems
          if (!matlab)
          {
            sendConnAck = true;
            sendTenzoState = true;
            if (printVerboseSerial && !printBlue)
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
            if (printVerboseSerial && !printBlue)
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
          Serial.println("ConAck 0");
          if (matlab)
          {
            sendDisconnAck = true;
          }
          else 
          {
            sendDisconnAck = true;
            Serial.println("Sending DisconnAck");
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
          if (printVerboseSerial)
          {
            Serial.println();
            Serial.println(" Warning!! Third Condition.");
            Serial.println();
          }
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
  else if (Serial.available())
  {
    char modeS = Serial.read(); 

    if (modeS == 'a')
    {
      initialize();
      Serial.println("initialize");
    }
    if (modeS == 'b')
    {
      Serial.println('K');   
    }    
    if (modeS == 'K')
    {
      connAck = 1;
      printBlue = true; 
      // Send Tenzo Status to mobile app
      sendStatus();
      // Set sendBlueAngle to false
      resetStatus();
    }
    if (modeS == 'L')
    {
      Serial.println(" Land ");
    }
    if (modeS == 't')
    {
      char t = Serial.read();
      if (t=='e')
        sendBlueAngle = true;
      else if (t=='d')
      {
        Serial.println('A');    
        sendBlueAngle = false;
      }   
    }
    if (modeS == 'p')
    {
      char t = Serial.read();
      if (t=='e')
      {
        enablePid = true;
        Serial.println("c,p,");
        changePidState(enablePid);
      }
      else if (t == 'd')
      {
        enablePid = false;
        Serial.println("c,n,");
        changePidState(enablePid);
      }     
      else if (t=='a')
      { 
        // en/disable each single angular pid
        char x = Serial.read();
        if (x=='r')
        {
          char y = Serial.read();
          if (y=='e' || y=='r')
          {
            enableRollPid = true;
            sendPidStatus(1,0,1,consKpRoll,consKiRoll,consKdRoll);
          }
          else if (y=='d')
          {
            enableRollPid = false;
            sendPidStatus(1,0,0,0,0,0);
          }
          else if (y=='s')
          {
            // TODO V1.2
            // Set consKpRoll to received value 
            float integerValue = 0;
            if (Serial.read()==',')
            {
              for(int i = 0;i<2;i++)
              {
                char incomingByte = Serial.read();
                integerValue *= 10;
                integerValue = ((incomingByte - 48) + integerValue);
              }
              consKpRoll = integerValue/100;
              //Serial.println(consKpRoll);
              integerValue = 0;
              if (Serial.read()==',')
              {
                for(int i = 0;i<2;i++)
                {
                  char incomingByte = Serial.read();
                  integerValue *= 10;
                  integerValue = ((incomingByte - 48) + integerValue);
                }
                consKiRoll = integerValue/100;
                //Serial.println(consKiRoll);
                integerValue = 0;
                if (Serial.read()==',')
                {
                  for(int i = 0;i<2;i++)
                  {
                    char incomingByte = Serial.read();
                    integerValue *= 10;
                    integerValue = ((incomingByte - 48) + integerValue);
                  }
                  consKdRoll = integerValue/100;
                  //Serial.println(consKdRoll);
                  integerValue = 0;
                }
              }
            }            
            sendPidStatus(1,0,1,consKpRoll,consKiRoll,consKdRoll);
          }         
        }
        if (x=='p')
        {
          char y = Serial.read();

          if (y=='e' || y=='r')
          {
            enablePitchPid = true;
            sendPidStatus(1,1,1,consKpPitch,consKiPitch,consKdPitch);
          }
          else if (y=='d')
          {
            enablePitchPid = false;
            sendPidStatus(1,1,0,0,0,0);
          }
          else if (y=='s')
          {
            // TODO V1.2
            // Set consKpRoll to received value 
            float integerValue = 0;
            if (Serial.read()==',')
            {
              for(int i = 0;i<2;i++)
              {
                char incomingByte = Serial.read();
                integerValue *= 10;
                integerValue = ((incomingByte - 48) + integerValue);
              }
              consKpPitch = integerValue/100;
              integerValue = 0;
              if (Serial.read()==',')
              {
                for(int i = 0;i<2;i++)
                {
                  char incomingByte = Serial.read();
                  integerValue *= 10;
                  integerValue = ((incomingByte - 48) + integerValue);
                }
                consKiPitch = integerValue/100;
                integerValue = 0;
                if (Serial.read()==',')
                {
                  for(int i = 0;i<2;i++)
                  {
                    char incomingByte = Serial.read();
                    integerValue *= 10;
                    integerValue = ((incomingByte - 48) + integerValue);
                  }
                  consKdPitch = integerValue/100;
                  integerValue = 0;
                }
              }
            }            
            sendPidStatus(1,1,1,consKpPitch,consKiPitch,consKdPitch);
          }       
        }
        if (x=='y')
        {
          char y = Serial.read();

          if (y=='e' || y=='r')
          {
            enableYawPid = true;
            sendPidStatus(1,2,1,consKpYaw,consKiYaw,consKdYaw);
          }
          else if (y=='d')
          {
            enableYawPid = false;
            sendPidStatus(1,2,0,0,0,0);
          }
          else if (y=='s')
          {
            // TODO V1.2
            // Set consKpRoll to received value 
            float integerValue = 0;
            if (Serial.read()==',')
            {
              for(int i = 0;i<2;i++)
              {
                char incomingByte = Serial.read();
                integerValue *= 10;
                integerValue = ((incomingByte - 48) + integerValue);
              }
              consKpYaw = integerValue/100;
              integerValue = 0;
              if (Serial.read()==',')
              {
                for(int i = 0;i<2;i++)
                {
                  char incomingByte = Serial.read();
                  integerValue *= 10;
                  integerValue = ((incomingByte - 48) + integerValue);
                }
                consKiYaw = integerValue/100;
                integerValue = 0;
                if (Serial.read()==',')
                {
                  for(int i = 0;i<2;i++)
                  {
                    char incomingByte = Serial.read();
                    integerValue *= 10;
                    integerValue = ((incomingByte - 48) + integerValue);
                  }
                  consKdYaw = integerValue/100;
                  integerValue = 0;
                }
              }
            }            
            sendPidStatus(1,2,1,consKpYaw,consKiYaw,consKdYaw);
          } 
        }
      } 
      else if (t=='w')
      { 
        // en/disable each single w ang pid
        char x = Serial.read();
        if (x=='r')
        {
          char y = Serial.read();

          if (y=='e' || y=='r')
          {
            enableWRollPid = true;
            sendPidStatus(2,0,1,consKpWRoll,consKiWRoll,consKdWRoll);
          }
          else if (y=='d')
          {
            enableWRollPid = false;
            sendPidStatus(2,0,0,0,0,0);
          }
          else if (y=='s')
          {
            // TODO V1.2
            float integerValue = 0;
            if (Serial.read()==',')
            {
              for(int i = 0;i<2;i++)
              {
                char incomingByte = Serial.read();
                integerValue *= 10;
                integerValue = ((incomingByte - 48) + integerValue);
              }
              consKpWRoll = integerValue/100;
              integerValue = 0;
              if (Serial.read()==',')
              {
                for(int i = 0;i<2;i++)
                {
                  char incomingByte = Serial.read();
                  integerValue *= 10;
                  integerValue = ((incomingByte - 48) + integerValue);
                }
                consKiWRoll = integerValue/100;
                integerValue = 0;
                if (Serial.read()==',')
                {
                  for(int i = 0;i<2;i++)
                  {
                    char incomingByte = Serial.read();
                    integerValue *= 10;
                    integerValue = ((incomingByte - 48) + integerValue);
                  }
                  consKdWRoll = integerValue/100;
                  integerValue = 0;
                }
              }
            }            
            sendPidStatus(2,0,1,consKpWRoll,consKiWRoll,consKdWRoll);       
          }    
        }
        if (x=='p')
        {
          char y = Serial.read();

          if (y=='e' || y=='r')
          {
            enableWPitchPid = true;
            sendPidStatus(2,1,1,consKpWPitch,consKiWPitch,consKdWPitch);
          }
          else if (y=='d')
          {
            enableWPitchPid = false;
            sendPidStatus(2,1,0,0,0,0);
          }
          else if (y=='s')
          {
            // TODO V1.2
            float integerValue = 0;
            if (Serial.read()==',')
            {
              for(int i = 0;i<2;i++)
              {
                char incomingByte = Serial.read();
                integerValue *= 10;
                integerValue = ((incomingByte - 48) + integerValue);
              }
              consKpWPitch = integerValue/100;
              integerValue = 0;
              if (Serial.read()==',')
              {
                for(int i = 0;i<2;i++)
                {
                  char incomingByte = Serial.read();
                  integerValue *= 10;
                  integerValue = ((incomingByte - 48) + integerValue);
                }
                consKiWPitch = integerValue/100;
                integerValue = 0;
                if (Serial.read()==',')
                {
                  for(int i = 0;i<2;i++)
                  {
                    char incomingByte = Serial.read();
                    integerValue *= 10;
                    integerValue = ((incomingByte - 48) + integerValue);
                  }
                  consKdWPitch = integerValue/100;
                  integerValue = 0;
                }
              }
            }            
            sendPidStatus(2,1,1,consKpWPitch,consKiWPitch,consKdWPitch);  
          }         
        }
        if (x=='y')
        {
          char y = Serial.read();

          if (y=='e' || y=='r')
          {
            enableWYawPid = true;
            sendPidStatus(2,2,1,consKpWYaw,consKiWYaw,consKdWYaw);
          }
          else if (y=='d')
          {
            enableWYawPid = false;
            sendPidStatus(2,2,0,0,0,0);
          }
          else if (y=='s')
          {
            // TODO V1.2
            float integerValue = 0;
            if (Serial.read()==',')
            {
              for(int i = 0;i<2;i++)
              {
                char incomingByte = Serial.read();
                integerValue *= 10;
                integerValue = ((incomingByte - 48) + integerValue);
              }
              consKpWYaw = integerValue/100;
              integerValue = 0;
              if (Serial.read()==',')
              {
                for(int i = 0;i<2;i++)
                {
                  char incomingByte = Serial.read();
                  integerValue *= 10;
                  integerValue = ((incomingByte - 48) + integerValue);
                }
                consKiWYaw = integerValue/100;
                integerValue = 0;
                if (Serial.read()==',')
                {
                  for(int i = 0;i<2;i++)
                  {
                    char incomingByte = Serial.read();
                    integerValue *= 10;
                    integerValue = ((incomingByte - 48) + integerValue);
                  }
                  consKdWYaw = integerValue/100;
                  integerValue = 0;
                }
              }
            }            
            sendPidStatus(2,2,1,consKpWYaw,consKiWYaw,consKdWYaw);  
          }     
        }
      }
      else if (t=='q')
      { 
        char z = Serial.read();
        if (z=='e')
        {  
          enableAltitudePid = true; 
          sendPidStatus(0,0,1);
        } 
        else if (z=='d')
        {
          enableAltitudePid = false;
          sendPidStatus(0,0,0); 
        }
      }
    }
    else if (modeS== '1')
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
    else if (modeS == 'L')
    {
      if (!processing && printBlue)
        Serial.println("Landing");
      land();
    }

    else if (modeS == 'w')
    {
      if (throttle>thresholdDown)
      {
        throttle = throttle + 5;
      }
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
    /*
    if (modeS == 'p')
     {
     changePidState(true);
     enablePid = true;
     }
     else if (modeS == 'l')
     {
     changePidState(false);
     enablePid = false;
     }
     else if(modeS == 'L')
     {        
     if (!processing)
     Serial.println("Landing");
     land();
     }
     */
    else if (modeS == 'm')
    {
      printMotorsVals = !printMotorsVals; 
    }
    else if (modeS == 'v')
    {
      printPIDVals = !printPIDVals; 
    }    
    else if (modeS == 't')
    {
      printTimers = !printTimers; 
    }
    else if (modeS == 'x')
    {
      printAccs = !printAccs; 
    }
    else if (modeS == 'y')
    {
      printOmegas = !printOmegas; 
    }
  }

  timerSec = micros()-secRoutine;
  lastTimeToRead = micros();

  if (timerSec >= 1000000)
  {
    secRoutine = micros();
    if (!processing && printTimers && !printBlue)
    {
      //Serial.print(cont);
      Serial.print("[sample/sec] ");
      Serial.print(contSamples);
      Serial.print("    Ctrl: ");
      Serial.println(countCtrlAction);
      Serial.print("    tservo: ");
      Serial.println(servoTime);
      Serial.println();
    }
    cont=0;      
    contSamples=0;      
    contCalc=0; 
    countCtrlAction=0;
  }

  timerRoutine = micros()-kMRoutine;

  // The following loop runs every 5ms
  if (timerRoutine >= deltaT*1000) 
  {      
    kMRoutine = micros();
    count += 1;
    if (count >= 1)
    {
      count = 0;

      //control();  
      controlW();

      //servoTime = micros();
      motorSpeedPID(throttle, OutputWPitch, OutputWRoll, OutputWYaw, OutputAltitude);
      //servoTime = micros() - servoTime;
      if (processing && printSerial)
      {
        printSerialAngle();
      }
      if (printBlue && sendBlueAngle)
        printSerialAngleBlue();
      if (printAccs)
        printAcc();
      if (printOmegas)
        printOmega();
      //if (printTimers)
      // printT();
      if (printPIDVals)
        printPidValues();
      if (printMotorsVals)
        printMotorsValues();
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
    if (printAckCommands && !printBlue)
    {
      Serial.println();
      Serial.print("[Sync] Tutto a porno");
      Serial.println();
    }
  }
  else
  {
    sendTenzoState = true;
    if (printAckCommands && !printBlue)
    {
      Serial.println();
      Serial.print("[Sync] state problem between Arduino (local) and Matlab (remote). Sending states... ");
      Serial.println();
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
      //if(errorRoll<thresholdRoll)
      //{  //we're close to setpoint, use conservative tuning parameters
      myRollPID.SetTunings(consKpRoll, consKiRoll, consKdRoll);
      //}
      //else
      //{
      //we're far from setpoint, use aggressive tuning parameters
      //myRollPID.SetTunings(aggKpRoll, aggKiRoll, aggKdRoll);
      //}
    }

    // Pitch PID
    if (enablePitchPid)
    {
      //if(errorPitch<thresholdPitch)
      //{  //we're close to setpoint, use conservative tuning parameters
      myPitchPID.SetTunings(consKpPitch, consKiPitch, consKdPitch);
      //}
      //else
      //{
      //we're far from setpoint, use aggressive tuning parameters
      // myPitchPID.SetTunings(aggKpPitch, aggKiPitch, aggKdPitch);
      //}
    }

    // Yaw PID
    if (enableYawPid)
    {
      //if(errorYaw<thresholdYaw)
      //{
      //we're close to setpoint, use conservative tuning parameters
      myYawPID.SetTunings(consKpYaw, consKiYaw, consKdYaw);
      //}
      //else
      //{
      //we're far from setpoint, use aggressive tuning parameters
      //myYawPID.SetTunings(aggKpYaw, aggKiYaw, aggKdYaw);
      //} 
    }
    if (printPIDVals)
    {
      //dispActualAllPidVals();     
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

void getGyroValues()
{  
  //starting samplingTimer
  samplingTime = micros();

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);

  y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);

  if (initializedSetup)
    removeBias();

  samplingTime = micros()- samplingTime;
  contSamples++;
}


void printOmega()
{
  if (filterGyro && !printBlue)
  {
    Serial.print("       Wx:");
    Serial.print(wF[0]);

    Serial.print("       Wy:");
    Serial.print(wF[1]);

    Serial.print("       Wz:");
    Serial.print(wF[2]);
  }

  if (!printBlue)
  {
    Serial.print("      wx:");
    Serial.print(x);

    Serial.print("       wy:");
    Serial.print(y);

    Serial.print("       Wz:");
    Serial.println(z);
  }
}

void printAcc()
{
  if (!printBlue)
  {
    Serial.println();
    Serial.print(aax);
    Serial.print(",");
    Serial.print(aay);
    Serial.print(",");
    Serial.print(aaz);
    Serial.print(",");
    Serial.println("E");
  }
}

void printSerialAngle()
{
  if (!printBlue)
  { 
    Serial.print(phi);
    Serial.print(",");
    Serial.print(theta);
    Serial.print(",");
    Serial.print(psi);
    Serial.print(",");
    Serial.print(angleXAcc);
    Serial.print(",");
    Serial.print(angleYAcc);
    Serial.print(",");
    Serial.print(estXAngle);
    Serial.print(",");
    Serial.print(estYAngle);
    Serial.print(",");
    Serial.println(bearing1);
  }
}

/* 
 * The following function prints via bluetooth the euler angles 
 * based on the printBlueAngleSkip parameter
 */
void printSerialAngleBlue()
{ 
  printBlueAngleCounter += 1;
  if (printBlueAngleCounter >= printBlueAngleSkip)
  {
    printBlueAngleCounter = 0;
    if (printBlue)
    { 
      Serial.print("o");
      Serial.print(",");
      Serial.print(estXAngle);
      Serial.print(",");
      Serial.print(estYAngle);
      Serial.print(",");
      Serial.print(bearing1);
      Serial.println(",");
    }
  }  
}

void removeBias()
{
  x = (x - bx)*scale2000/1000;
  y = (y - by)*scale2000/1000;
  z = (z - bz)*scale2000/1000;
} 

void calcBias()
{
  if (!processing &&  !printBlue)
    Serial.println("Bias");
  int c = 2000;
  for (int i = 0; i<c; i++)
  {
    delay(1);
    getGyroValues(); 
    bxS = bxS + x;
    byS = byS + y;
    bzS = bzS + z;
  }

  bx = bxS / c;
  by = byS / c;
  bz = bzS / c;

  if (!processing && !printBlue)
  {
    Serial.println(bx);
    Serial.println(by);
    Serial.println(bz);
  }  
}

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

void writeRegister(int deviceAddress, byte address, byte val) 
{
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

  while(!Wire.available()) 
  {
    // waiting
    // Serial.println("No Data");
  }

  v = Wire.read();
  return v;
}

int setupL3G4200D(int scale)
{
  //From  Jim Lindblom of Sparksfun's code

    // Enable x, y, z and turn off power down:
  //writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);
  // 400Hz and 1Hz cutoff frq of the HPF
  //writeRegister(L3G4200D_Address, CTRL_REG1, 0b01011111);
  // 200Hz 
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b01001111);

  // If you'd like to adjust/use the HPF:
  // High pass filter cut off frecuency configuration
  // ODR 200Hz, cutoff 0.02Hz 1001
  //writeRegister(L3G4200D_Address, CTRL_REG2, 0b00001001);
  // ODR 200Hz, cutoff 1 Hz 0100
  //writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000100);
  // ODR 200Hz, cutoff 0.02Hz 1001
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00001001);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  //writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250)
  {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }
  else if(scale == 500)
  {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }
  else
  {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00010000);
  //writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void testMotor(int x)
{   
  //Serial.print(" Testing motor: ");
  Serial.println(x);
  if (x==1)
  {
    Serial.println('M1');
    //Serial.println(" Increasing angular velocity");
    for (int i = minTest; i<maxTest;i++)
    {
      servo1.writeMicroseconds(i);
      Serial.println(i);  
      delay(2);     
    }
    for (int i = maxTest; i>minTest;i--)
    {
      servo1.writeMicroseconds(i);
      Serial.println(i);  
      delay(2);      
    }
  }
  else if (x==2)
  {
    Serial.println('M2');
    //Serial.println(" Increasing angular velocity");//Serial.println(" Increasing angular velocity");
    for (int i = minTest; i<maxTest;i++)
    {
      servo2.writeMicroseconds(i);
      Serial.println(i);  
      delay(2);     
    }
    for (int i = maxTest; i>minTest;i--)
    {
      servo2.writeMicroseconds(i);
      Serial.println(i);  
      delay(2);      
    }
  }
  else if (x==3)
  {
    Serial.println('M3');
    //Serial.println(" Increasing angular velocity");
    //Serial.println(" Increasing angular velocity");
    for (int i = minTest; i<maxTest;i++)
    {
      servo3.writeMicroseconds(i);
      Serial.println(i);  
      delay(2);     
    }
    for (int i = maxTest; i>minTest;i--)
    {
      servo3.writeMicroseconds(i);
      Serial.println(i);  
      delay(2);      
    }  
  }
  else if (x==4)
  {
    Serial.println('M4');
    //Serial.println(" Increasing angular velocity");
    //Serial.println(" Increasing angular velocity");
    for (int i = minTest; i<maxTest;i++)
    {
      servo4.writeMicroseconds(i);
      Serial.println(i);  
      delay(2);     
    }
    for (int i = maxTest; i>minTest;i--)
    {
      servo4.writeMicroseconds(i);
      Serial.println(i);  
      delay(2);      
    }
  }
}

void control()
{
  if (enablePid)
  {
    // Roll PID
    if (enableRollPid)
    {
      //Serial.println("    ZAK ");
      InputRoll = estXAngle;
      //Serial.println("    ZAK ");
      errorRoll = abs(SetpointRoll - estXAngle); //distance away from setpoint
      //if(errorRoll<thresholdRoll)
      //{  //we're close to setpoint, use conservative tuning parameters
      myRollPID.SetTunings(Kmy*consKpRoll, Kmy*consKiRoll, Kmy*consKdRoll);
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
      /*
      if (printPIDVals && !printBlue)
       {
       Serial.println();
       Serial.print("INPUT ");
       Serial.print(InputRoll);
       Serial.print("ErrorRoll:");
       Serial.print(errorRoll);
       Serial.print("Roll PID: ");
       Serial.print(OutputRoll);
       Serial.println();
       }
       */
    }
    else
    {
      Serial.println();
      Serial.println("SS");
      Serial.println();
      OutputRoll = 0;
    }

    // Pitch PID1
    if (enablePitchPid)
    {
      InputPitch = estYAngle;
      errorPitch = abs(SetpointPitch - estYAngle); //distance away from setpoint

      //if (errorPitch<thresholdPitch)
      //{  //we're close to setpoint, use conservative tuning parameters
      myPitchPID.SetTunings(Kmy*consKpPitch, Kmy*consKiPitch, Kmy*consKdPitch);
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
      /*
      if (printPIDVals && !printBlue)
       { 
       Serial.print("E:  ");
       Serial.print(errorPitch);
       Serial.print(" A:");
       Serial.print(OutputPitch);
       Serial.println();
       }
       */
    }  
    else
    {
      Serial.println();
      Serial.println("SS");
      Serial.println();
      OutputPitch = 0;
    }

    /*
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
     
     if (printPIDVals && !printBlue)
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
     */
    if (printPIDVals && !printBlue)
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

void controlW()
{
  //Serial.println("K1");
  if (enablePid)
  {
    //Serial.println("K2");
    // Roll W PID
    if (enableWRollPid)
    {
      InputWRoll = y;
      errorWRoll = abs(SetpointWRoll - y); 

      wRollPID.SetTunings(Kw*consKpWRoll, Kw*consKiWRoll, Kw*consKdWRoll);

      wRollPID.Compute();
    }
    else
    {
      OutputWRoll = 0;
    }

    // Pitch PID1
    if (enablePitchPid)
    {
      InputWPitch = x;
      errorWPitch = abs(SetpointWPitch - x);

      wPitchPID.SetTunings(Kw*consKpWPitch, Kw*consKiWPitch, Kw*consKdWPitch);

      wPitchPID.Compute(); // Computes outputPitch


    }  
    else
    {
      OutputWPitch = 0;
    }

    /*
    // Yaw PID
     if (enableYawPid)
     {
     if (filterAng == 1)
     {
     InputWYaw = z;
     errorWYaw = abs(SetpointWYaw - z); //distance away from setpoint
     }
     
     if(errorYaw<thresholdYaw)
     {
     //we're close to setpoint, use conservative tuning parameters
     wYawPID.SetTunings(Kw*consKpWYaw, Kw*consKiWYaw, Kw*consKdWYaw);
     }   
     wYawPID.Compute(); 
     
     }
     else
     {
     OutputYaw=0;
     }
     */
    if (printPIDVals)
    {
      //Serial.println();      
    }
  }
  else
  {
    OutputWRoll = 0;
    OutputWPitch = 0;    
    OutputWYaw = 0;
  }
}

void printPidValues()
{
  if (enableWRollPid && !printBlue)
  {
    Serial.println();
    //        Serial.print("INPUT ANGULAR PID ROLL ");
    //        Serial.print(InputWRoll);
    Serial.print("     ErrWR:  ");
    Serial.print(errorWRoll);
    Serial.print(" RW PID:  ");
    Serial.print(OutputWRoll);
    Serial.println();
  }  

  if (enableWPitchPid && !printBlue)
  { 
    Serial.print(" Omega err: ");
    Serial.print(errorWPitch);
    Serial.print(" Omega Pid:");
    Serial.print(OutputWPitch);
    Serial.println();
  }

  if (enableWYawPid &&  !printBlue)
  {
    Serial.print(" ErrWY: ");
    Serial.print(errorWYaw);
    Serial.print(" YWpid: ");
    Serial.print(OutputWYaw);
    Serial.println();
  }  
}

void printMotorsValues()
{
  if (!printBlue)
  {
    Serial.println();
    Serial.print(" Mot:[ ");
    Serial.print(motorA);
    Serial.print(" ; ");
    Serial.print(motorB);
    Serial.print(" ; ");
    Serial.print(motorC);
    Serial.print(" ; ");
    Serial.print(motorD);
    Serial.println("   ]"); 
    /* 
     Serial.print("  PID:[R ");
     Serial.print(OutputRoll);
     Serial.print(" ;P  ");
     Serial.print(OutputPitch);
     Serial.print("  ;Y  ");
     Serial.print(OutputYaw);
     Serial.print(" ; T ");
     Serial.print(throttle);
     Serial.print("   ]");
     */
    Serial.println();
  }
}


