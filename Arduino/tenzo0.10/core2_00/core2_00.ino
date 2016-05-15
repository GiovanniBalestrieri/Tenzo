#include <Servo.h>
#include <Wire.h>
#include "Propulsion.h"
#include "Ux.h"
#include "PID_v2.h"
#include "NonLinearPid.h"
#include "MedianFilter.h"
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

Ux sakura;
Propulsion tenzoProp(sakura.getM(1),sakura.getM(2),sakura.getM(3),sakura.getM(4));

//NonLinearPid controlCascade(SetpointRoll,SetpointPitch,SetpointYaw,SetpointAltitude);

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();
 
volatile float angles[3];

/*
 * deltaT : Control loop frequency (Verbose_motors /NonVerbose)
 * 15: 50Hz
 * 8 : 75HZ/44Hz
 * 7 : 110Hz/56Hz
 */
float deltaT = 15;  

char readAnswer, readChar, readCh;

byte modeS;

/**
 * Modes
 */
int connAck = 0;
int takeOff = 0;
int hovering = 0;
int landed = 1;
int tracking = 0;
int warning = 0;

/**
 * VTOL settings
 */
 // Take Off settings
int rampTill = 1100; // rampTill = 1270;
int idle = 1000;
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
boolean sendStatesRemote = false;

/** 
 ** Control
 **/
 
// W constant (necessary?)
float Kmy = 1;
/**
 * Pid Controller 
 */
 
// Rimesso - check validity
 
boolean autoEnablePid = true;
volatile boolean enablePid = false;

// theta
volatile boolean enableRollPid = true;
volatile boolean enablePitchPid = true;
volatile boolean enableYawPid = false;
// w
volatile boolean enableWRollPid = true;
volatile boolean enableWPitchPid = false;
volatile boolean enableWYawPid = false;
volatile boolean enableAltitudePid = false;

volatile int limitPidMax = 750;
 
boolean inConsRoll = false; 
boolean inConsPitch = false;
boolean verbosePidValuesFeedback = true;
boolean verboseFilterAccMatlab = true;
// Define IO and setpoint for control
//double SetpointRoll = 0, InputRoll, errorRoll;
// Define the aggressive and conservative Tuning Parameters
        
           
        /*
         * Cascade Pid & settings
         */
         
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

NonLinearPid cascadeRollPid(consKpCascRoll, consKiCascRoll, consKdCascRoll);
NonLinearPid cascadeRollPidW(consKpCascRollW, consKiCascRollW, consKdCascRollW);
NonLinearPid cascadePitchPid(consKpCascPitch, consKiCascPitch, consKdCascPitch);
NonLinearPid cascadePitchPidW(consKpCascPitchW, consKiCascPitchW, consKdCascPitchW);
NonLinearPid cascadeYawPid(consKpCascYaw, consKiCascYaw, consKdCascYaw);
NonLinearPid cascadeYawPidW(consKpCascYawW, consKiCascYawW, consKdCascYawW);
NonLinearPid cascadeAltPid(consKpCascAlt, consKiCascAlt, consKdCascAlt);

// Control Pid Cascade
/*
PID cascadeRollPid(&InputCascRoll, &OutputCascRoll, &SetpointCascRoll, consKpCascRoll, consKiCascRoll, consKdCascRoll, DIRECT);
PID cascadeRollPidW(&InputCascRollW, &OutputCascRollW, &SetpointCascRollW, consKpCascRollW, consKiCascRollW, consKdCascRollW, DIRECT);

PID cascadePitchPid(&InputCascPitch, &OutputCascPitch, &SetpointCascPitch, consKpCascPitch, consKiCascPitch, consKdCascPitch, DIRECT);
PID cascadePitchPidW(&InputCascPitchW, &OutputCascPitchW, &SetpointCascPitchW, consKpCascPitchW, consKiCascPitchW, consKdCascPitchW, DIRECT);

PID cascadeYawPid(&InputCascYaw, &OutputCascYaw, &SetpointCascYaw, consKpCascYaw, consKiCascYaw, consKdCascYaw, DIRECT);
PID cascadeYawPidW(&InputCascYawW, &OutputCascYawW, &SetpointCascYawW, consKpCascYawW, consKiCascYawW, consKdCascYawW, DIRECT);

PID cascadeAltPid(&InputCascAlt, &OutputCascAlt, &SetpointCascAlt, consKpCascAlt, consKiCascAlt, consKdCascAlt, DIRECT);
/*
        
/**
 * Compass
 */

// Defines address of CMPS10
#define ADDRESS 0x60      
//CMPS10 my_compass;
float angPosFilter[3];
int pitch1;
int roll1;
int filterAng = 0;


byte highByte, lowByte, fine;    
char pitch, roll;                
int bearing;   

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

int scale2000 = 70;

float bx= 0,by=0,bz=0;
long bxS,byS,bzS;

unsigned long biasCalcTime = 0;

// Median Filter
MedianFilter medianGyroX(3,0);
MedianFilter medianGyroY(3,0);
MedianFilter medianGyroZ(3,0);

/**
 ** Acc 
 **/

const float RESOLUTION=800; //0.8 v/g -> resolucion de 1.5g -> 800mV/g
const float VOLTAGE=3.3;  //voltage al que está conectado el acelerómetro

const float ZOUT_1G = 850;   // mv Voltage en Zout a 1G

const int NADJ  = 50;        // Número de lecturas para calcular el error

// Define accelerometer's pins
const int xaxis = 0;
const int yaxis = 1;
const int zaxis = 2;

// 3.3 Fully loaded Tenzo V2.1
//int xRawMin = 425;
//int xRawMax = 280;
//// 
//int yRawMin = 410;
//int yRawMax = 275;
//// 
//int zRawMin = 428;
//int zRawMax = 289;


// 3.3 Fully loaded Tenzo V2.2
int xRawMin = 409;
int xRawMax = 278;
// 
int yRawMin = 404;
int yRawMax = 272;
// 
int zRawMin = 419;
int zRawMax = 288;

float XError,YError,ZError;

// Acc Timers
unsigned long accTimer;
unsigned long lastAccTimer;
unsigned long timeToRead = 0;
unsigned long lastTimeToRead = 0;
volatile long servoTime = 0;
volatile long servoTimeTot = 0;



float timerLoop = 0, timerReading = 0, timerSec = 0;
float timerRoutine = 0, count = 0;
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
 
// Gps
int BaudRateGps = 4800;
byte loBytew1, hiBytew1,loBytew2, hiBytew2;
int loWord,hiWord;

int printBlueAngleCounter = 0;
int printBlueAngleSkip = 5;

String inString = "";
String inComingString = "";



// Volatile vars
volatile int cont = 0;
volatile int countCtrlAction = 0;
volatile int contGyroSamples=0;
volatile int contCalc=0;

volatile float thetaOLD = 0;
volatile float phi=0;
volatile float theta=0;
volatile float psi=0;
volatile int x=0;
volatile int y = 0;
volatile int z= 0;
volatile float wVal[3] = {0,0,0};
volatile int rawAx = 0;
volatile int rawAy = 0;
volatile int rawAz = 0;
volatile int dt=0;

volatile float wF[3] = {0,0,0};
volatile float aF[3] = {0,0,0};
volatile boolean filterGyro = true;
volatile boolean filterAcc = true;
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


// Serial remote gains PID change

char kReadChar;
char k1ReadChar;
int k3ReadInt;
float readPropVal,readIntVal,readDerVal,readSetVal;
int readChar2;

void setupTimerInterrupt()
{
  // Timer settings
  // Initialize Timer
  cli();
  TCCR3A = 0;
  TCCR3B = 0;

  // Set compare match register to the desired timer count
  //OCR3A=77; //16*10^6/(200Hz*1024)-1 = 77 -> 200 Hz 
  //OCR3A=193; //16*10^6/(80Hz*1024)-1 = 193 -> 80 Hz 
  //OCR3A=103; //16*10^6/(150Hz*1024)-1 = 103 -> 150 Hz 
  OCR3A=143; //16*10^6/(150Hz*1024)-1 = 143 -> 109 Hz 
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
  if (!sakura.getProcessing())
  {
     Serial.println("[ OK ] Init Timers"); 
  }
}

void setupCtx()
{
  initializedSetup = true;
  k = micros();
  if (!sakura.getProcessing())
  {
     Serial.println("[ OK ] Init ctx"); 
  }
}

void setupCommunication()
{
  Wire.begin();
  Serial.begin(115200); 
  //Serial.begin(sakura.getBaudRate()); 
  if (!sakura.getProcessing())
  {
    Serial.println("[ Ok ] InitCOM ");
  }
}

void setupIMU()
{ 
  sixDOF.init(); //init the Acc and Gyro
  delay(5);
  if (!sakura.getProcessing())
  {
    Serial.println("[ Ok ] IMU ");
  }
}

/// TILL HERE

void setup() {
  setupCommunication();
  setupIMU();
//  setupAcceleromter();
//  setupGyro();
  setupTimerInterrupt();  
  setupCtx();  
  sakura.welcome();
  tenzoProp.calibrateOnce();
  tenzoProp.init();
}

void loop() {  
  SerialRoutine();
}


void calcAngle() //ISR
{
  // From Gyro
  dt = micros()-k;
  if (!filterGyro)
  {
    phi=phi+(float) x*(float)dt/1000000.0;
    
    thetaOLD = theta; 
    theta=theta+(float) y*(float) dt/1000000.0;

    psi=psi+(float) z*(float) dt/1000000.0; 
  }
  else if (filterGyro)
  {  
    phi=phi+wF[0]*(float)dt/1000000.0;

    thetaOLD = theta;
    theta=theta+wF[1]*(float) dt/1000000.0;

    psi=psi+wF[2]*(float) dt/1000000.0;  
  }
  
  if (filterAcc)
  {
    angleXAcc = (atan2(-aF[0],-aF[2])) * RAD_TO_DEG;
    angleYAcc = (atan2(-aF[1],-aF[2])) * RAD_TO_DEG;
  }
  else 
  {
    // From Acc
    angleXAcc = (atan2(-aax,-aaz)) * RAD_TO_DEG;
    angleYAcc = (atan2(-aay,-aaz)) * RAD_TO_DEG;
  }
  k=micros();  
}

void wFilter(volatile float val[])
{
  val[0] = (1-alphaW)*val[0] + alphaW*wxm1;
  val[1] = (1-alphaW)*val[1] + alphaW*wym1;
  val[2] = (1-alphaW)*val[2] + alphaW*wzm1;
  
  wxm1 = val[0];
  wym1 = val[1];
  wzm1 = val[2];
}

void aFilter(volatile float val[])
{
  val[0] = (1-alphaA)*val[0] + alphaA*axm1;
  val[1] = (1-alphaA)*val[1] + alphaA*aym1;
  val[2] = (1-alphaA)*val[2] + alphaA*azm1;
  
  axm1 = val[0];
  aym1 = val[1];
  azm1 = val[2];
}


ISR(TIMER3_COMPB_vect)
{ 
  // Updates counters
  servoTime = micros();
  
  sei();          
  
  // [max] 8700 us [avg] 4450 us
  sixDOF.getYawPitchRoll(angles);  
  acquireGyro();
  contGyroSamples++;    
    
  // [max] 5000 us [avg] 3000 us      
  controlCascade();  
    
  //calcAngle();
  //estAngle();
  
  cont++;  
  
  // Compute isr duration
  servoTime = micros() - servoTime;
  servoTimeTot = servoTimeTot + servoTime;
  //cli();
}

void acquireGyro() // ISR
{     
  sixDOF.getGyroValues(wVal);
  if (filterGyro)
  {    
    medianGyroX.in(wVal[0]);
    wVal[0] = medianGyroX.out();   
   
    medianGyroY.in(wVal[1]);
    wVal[1] = medianGyroY.out();   
   
    medianGyroZ.in(wVal[2]);
    wVal[2] = medianGyroZ.out();    
  }  
}

void SerialRoutine()
{
  if (Serial.available())
  {
      char t = Serial.read();
      
      //if (t == '1')
      //  tenzoProp.stopAll();      
      if (t == 'c')
      {
        // Serial communication        
        Serial.println("K");
        sendStatesRemote = true;
      }      
      if (t == 'X')
      {
        // Serial communication        
        Serial.println("X");
        sendStatesRemote = true;
      } 
      else if (t == 'r')
        tenzoProp.resetMotors();
      else if (t == 'q')
      {
        tenzoProp.setThrottle(tenzoProp.getThrottle() + 10);
        if (!sakura.getProcessing())
        {
          Serial.print("m,");
          Serial.print(tenzoProp.getThrottle());
          Serial.println(",z");
        }
      }
      else if (t == 'a')
      {
        tenzoProp.setThrottle(tenzoProp.getThrottle() - 10);
        
        if (!sakura.getProcessing())
        {
          Serial.print("m,");
          Serial.print(tenzoProp.getThrottle());
          Serial.println(",z");
        }
      }
      // PID values modification from remote
      else if (t == 'u')
      { 
         k1ReadChar = Serial.read();
         // Read Pid values Requested csv format
         if (k1ReadChar == 44)
         {
           readChar2 = Serial.parseInt();
           
           if (readChar2 == 31)
           {
             // show roll Cons
             sendPidVal(0,0);           
           }
           else if (readChar2 == 32)
           {
             // show roll agg              
             sendPidVal(0,1);              
           }
           else if (readChar2 == 33)
           {
             // w Roll cons 
             sendPidVal(3,0);  
           } 
           else if (readChar2 == 34)
           {
             // show pitch cons  
             sendPidVal(1,0);              
           }
           else if (readChar2 == 35)
           {
             // pitch agg        
             sendPidVal(1,1);      
           }
           else if (readChar2 == 36)
           {           
             // w pitch cons 
             sendPidVal(4,0); 
           }  
           else if (readChar2 == 37)
           {
             // yaw cons
             sendPidVal(2,0);   
           }
           else if (readChar2 == 38)
           {
             // yaw agg        
             sendPidVal(2,1);      
           }
           else if (readChar2 == 39)
           {
             // w yaw cons
             sendPidVal(5,0); 
           } 
           // Setting pid values from remote #pid #remote
           // u,4,kp,ki,kd,set,z
           else if (readChar2 == 4)
           {
             // comma
             kReadChar = Serial.read();
             if (kReadChar == 44)
             {
               //Serial.println("KODIO");
               // read mode
               k3ReadInt = Serial.parseInt();
               //Serial.println(k3);
               
               kReadChar = Serial.read();
               if (kReadChar == 44)
               {
                 // read kp
                 readPropVal = Serial.parseFloat();
               //Serial.println(p);
                 
                 kReadChar = Serial.read();
                 if (kReadChar == 44)
                 {
                   // read ki
                   readIntVal = Serial.parseFloat();
               //Serial.println(i);
                   
                   kReadChar = Serial.read();
                   if (kReadChar == 44)
                   {
                     // read kd
                     readDerVal = Serial.parseFloat();
               //Serial.println(d);
                     
                     kReadChar = Serial.read();
                     if (kReadChar == 44)
                     {
                       // read new SetPoint
                       readSetVal = Serial.parseFloat();
               //Serial.println(set);
                     }
                   }
                 }
               }
               
               // Security Check [ IMPORTANT ]
               if (readPropVal>20)
                  readPropVal=20;
               if (readPropVal<0)
                  readPropVal=0;
               if (readDerVal>6)
                  readDerVal=6;
               if (readDerVal<0)
                  readDerVal=0;
               if (readIntVal>6)
                  readIntVal=6;
               if (readIntVal<0)
                  readIntVal=0;
                  // ADD setpoint check pensaci un po'
              
               if (k3ReadInt == 31)
               {
                 // show roll Cons
                 consKpCascRoll = readPropVal;
                 consKdCascRoll = readDerVal;
                 consKiCascRoll = readIntVal;
                 SetpointCascRoll = readSetVal;
                 
                 // Send actual Vals     
                 sendPidVal(0,0);           
               }
               else if (k3ReadInt == 32)
               {
                 // show roll agg  
                 aggKpCascRoll = readPropVal;
                 aggKdCascRoll = readDerVal;
                 aggKiCascRoll = readIntVal;
                 SetpointCascRoll = readSetVal;
                 
                 // Send actual Vals            
                 sendPidVal(0,1);              
               }
               else if (k3ReadInt == 33)
               {
                 // w Roll cons
                 consKpCascRollW = readPropVal;
                 consKdCascRollW = readDerVal;
                 consKiCascRollW = readIntVal;
                 SetpointCascRollW = readSetVal;
                 
                 // Send actual Vals  
                 sendPidVal(3,0);  
               } 
               else if (k3ReadInt == 34)
               {
                 // show pitch cons  
                 consKpCascPitch = readPropVal;
                 consKdCascPitch = readDerVal;
                 consKiCascPitch = readIntVal;
                 SetpointCascPitch = readSetVal;
                 
                 // Send actual Vals 
                 sendPidVal(1,0);              
               }
               else if (k3ReadInt == 35)
               {
                 // pitch agg 
                 aggKpCascPitch = readPropVal;
                 aggKdCascPitch = readDerVal;
                 aggKiCascPitch = readIntVal;
                 SetpointCascPitch = readSetVal;
                 
                 // Send actual Vals        
                 sendPidVal(1,1);      
               }
               else if (k3ReadInt == 36)
               {           
                 // w pitch cons 
                 consKpCascPitchW = readPropVal;
                 consKdCascPitchW = readDerVal;
                 consKiCascPitchW = readIntVal;
                 SetpointCascPitchW = readSetVal;
                 
                 // Send actual Vals 
                 sendPidVal(4,0); 
               }  
               else if (k3ReadInt == 37)
               {
                 // yaw cons
                 consKpCascYaw = readPropVal;
                 consKdCascYaw = readDerVal;
                 consKiCascYaw = readIntVal;
                 SetpointCascYaw = readSetVal;
                 
                 // Send actual Vals 
                 sendPidVal(2,0);   
               }
               else if (k3ReadInt == 38)
               {
                 // yaw agg  
                 aggKpCascYaw = readPropVal;
                 aggKdCascYaw = readDerVal;
                 aggKiCascYaw = readIntVal;
                 SetpointCascYaw = readSetVal;      
                 sendPidVal(2,1);      
               }
               else if (k3ReadInt == 39)
               {
                 // w yaw cons 
                 consKpCascYawW = readPropVal;
                 consKdCascYawW = readDerVal;
                 consKiCascYawW = readIntVal;
                 SetpointCascYawW = readSetVal;
                 
                 // Send actual Vals
                 sendPidVal(5,0); 
               }
             }             
           }
         }
      }
      else if (t == 'v')
      {        
        //if (!sakura.getProcessing())
        //{
          //Serial.println(tenzoProp.getThrottle());
          Serial.println();
          Serial.print("V,W - P|I|D: ");
          Serial.print(consKpCascRollW);
          Serial.print(" | ");
          Serial.print(consKiCascRollW);
          Serial.print(" | ");
          Serial.println(consKdCascRollW);
          Serial.print("V,A - P|I|D: ");
          Serial.print(consKpCascRoll);
          Serial.print(" | ");
          Serial.print(consKiCascRoll);
          Serial.print(" | ");
          Serial.println(consKdCascRoll);
        //}
      }      
      else if (t == 'x')
      {
        resetMotorsPidOff();
        
        tenzoProp.detachAll();
        
        // To add in method
      }    
      else if (t == 'i')
      {
        initialize();
      }
      else if (t == 'L')
      {
        land();
      }
      else if (t == 'm')
      {
         sakura.setPrintMotorValsUs(!sakura.getPrintMotorValsUs()); 
      }
      else if (t == 'n')
      {
         sakura.setPrintAccs(!sakura.getPrintAccs()); 
      }
      else if (t == 'o')
      {
         //sakura.setPrintOmegas(!sakura.getPrintOmegas()); 
         printOmega();
      }
      else if (t == 'e')
      {
         //sakura.setPrintOmegas(!sakura.getPrintOmegas()); 
         printSerialAngleFus();
      }
      else if (t == 'z')
      {
        //sakura.setPrintPIDVals(!sakura.getPrintPIDVals());
      }
      else if (t == 'p')
      {
        enablePid = !enablePid;
        changePidState(enablePid);
        //if (enablePid)
          //Serial.println("ce,z");
        //else if (!enablePid)
          //Serial.println("cd,z");
        sendStatesRemote = true;
      }
      else if (t == 'l')
      {
        printAcc();
      }         
      else if (t == 'b')
      {
        char k = Serial.read();
        if (k=='e')
          sakura.setSendBlueAngle(true);
        else if (k=='d')
          sakura.setSendBlueAngle(false);
      }         
      else if (t == ',')
      {
        /*
        consKpWRoll = consKpWRoll + 0.02;
        Serial.print("\t\t\t\t\t\t\t\t\t");
        Serial.println(consKpWRoll);
        */
      }         
      else if (t == ';')
      {
        /*
        consKpWRoll = consKpWRoll - 0.02;
        if (consKpWRoll<=0)
          consKpWRoll = 0;
        Serial.print("\t\t\t\t\t\t\t\t\t");
        Serial.println(consKpWRoll);
        */
      }         
      else if (t == '.')
      {
        /*
        consKiWRoll = consKiWRoll + 0.05;
        Serial.print("\t\t\t\t\t\t\t\t\t");
        Serial.println(consKiWRoll);
        */
      }         
      else if (t == ':')
      {
        /*
        consKiWRoll = consKiWRoll - 0.05;
        if (consKiWRoll<=0)
          consKiWRoll = 0;
        Serial.print("\t\t\t\t\t\t\t\t\t");
        Serial.println(consKiWRoll);
        */
      }         
      else if (t == '-')
      {
        /*
        consKdWRoll = consKdWRoll + 0.02;
        Serial.print("\t\t\t\t\t\t\t\t\t");
        Serial.println(consKdWRoll);
        */
      }         
      else if (t == '_')
      {
        /*
        consKdWRoll = consKdWRoll - 0.02;
        if (consKdWRoll<=0)
          consKdWRoll = 0;
        Serial.print("\t\t\t\t\t\t\t\t\t");
        Serial.println(consKdWRoll);
        */
      }
      else if (t == 'ò')
      {
        // request pid val
        SetpointCascRoll = SetpointCascRoll - 1;
        Serial.print("V,SetpointCascRoll:  ");
        Serial.println(SetpointCascRoll);
      }      
      else if (t == 'à')
      {
        SetpointCascRoll = SetpointCascRoll + 1;
        Serial.print("V,SetpointCascRoll:  ");
        Serial.println(SetpointCascRoll);
      }               
      else if (t == 't')
      {
        Serial.println("\t\t\tPorco Dia ");
      }         
      else if (t == 'y')
      {
        SetpointCascRoll = SetpointCascRoll + 1;
        Serial.print("V,SetpointCascRoll:  ");
        Serial.println(SetpointCascRoll);
      }    
      else if (t == 'd')
      {
        consKpCascRoll = consKpCascRoll - 0.05;
        if (consKpCascRoll<0)
          consKpCascRoll = 0;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(0,0);
        }          
      }         
      else if (t == 'D')
      {
        consKpCascRoll = consKpCascRoll + 0.05;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(0,0);
        }
      }     
      else if (t == 'f')
      {
        consKiCascRoll = consKiCascRoll - 0.05;
        if (consKiCascRoll<0)
          consKiCascRoll = 0;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(0,0); 
        }
      }         
      else if (t == 'F')
      {
        consKiCascRoll = consKiCascRoll + 0.05;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(0,0);
        }
      }     
      else if (t == 'g')
      {
        consKdCascRoll = consKdCascRoll - 0.05;
        if (consKdCascRoll<0)
          consKdCascRoll = 0;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(0,0);
        }
      }         
      else if (t == 'G')
      {
        consKdCascRoll = consKdCascRoll + 0.05;
        if (verbosePidValuesFeedback)
        {
           sendPidVal(0,0); 
        }
      }    
      else if (t == 'h')
      {
        consKpCascRollW = consKpCascRollW - 0.05;
        if (consKpCascRollW<0)
          consKpCascRollW = 0;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(3,0);
        } 
      }         
      else if (t == 'H')
      {
        consKpCascRollW = consKpCascRollW + 0.05;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(3,0);
        } 
      }     
      else if (t == 'j')
      {
        consKiCascRollW = consKiCascRollW - 0.05;
        if (consKiCascRollW<0)
          consKiCascRollW = 0;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(3,0);
        } 
      }         
      else if (t == 'J')
      {
        consKiCascRollW = consKiCascRollW + 0.05;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(3,0);
        } 
      }     
      else if (t == 'k')
      {
        consKdCascRollW = consKdCascRollW - 0.05;
        if (consKdCascRollW<0)
          consKdCascRollW = 0;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(3,0);
        } 
      }         
      else if (t == 'K')
      {
        consKdCascRollW = consKdCascRollW + 0.05;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(3,0);
        } 
      }  
      else if (t == 'w')
      {
        alphaW = alphaW + 0.05;
        if (alphaW>=1)
          alphaW = 1;
        if (verboseFilterAccMatlab)
        {
          sendAlphaW();
        } 
      }     
      else if (t == 'W')
      {
        alphaW = alphaW + 0.01;
        if (alphaW>=1)
          alphaW = 1;
        if (verboseFilterAccMatlab)
        {
          sendAlphaW();
        } 
      }     
      else if (t == 's')
      {
        alphaW = alphaW - 0.05;
        if (alphaW<=0)
          alphaW = 0;
        if (verboseFilterAccMatlab)
        {
          sendAlphaW();
        } 
      }             
      else if (t == 'S')
      {
        alphaW = alphaW - 0.01;
        if (alphaW<=0)
          alphaW = 0;
        if (verboseFilterAccMatlab)
        {
          sendAlphaW();
        } 
      }       
      /*      
      else if (t == 'w')
      {
        alphaA = alphaA + 0.001;
        if (alphaA>=1)
          alphaA = 1;
        if (verboseFilterAccMatlab)
        {
          sendAlphaAcc();
        } 
      }     
      else if (t == 'W')
      {
        alphaA = alphaA + 0.01;
        if (alphaA>=1)
          alphaA = 1;
        if (verboseFilterAccMatlab)
        {
          sendAlphaAcc();
        } 
      }     
      else if (t == 's')
      {
        alphaA = alphaA - 0.001;
        if (alphaA<=0)
          alphaA = 0;
        if (verboseFilterAccMatlab)
        {
          sendAlphaAcc();
        } 
      }             
      else if (t == 'S')
      {
        alphaA = alphaA - 0.01;
        if (alphaA<=0)
          alphaA = 0;
        if (verboseFilterAccMatlab)
        {
          sendAlphaAcc();
        } 
      }       
      */
  }
  timerSec = micros()-secRoutine;
  //lastTimeToRead = micros();
   
  // Runs @ 1 Hz
  if (timerSec >= 1000000)
  {
    secRoutine = micros();
    
    // Compute average imu readings time
    servoTimeTot = servoTimeTot/countCtrlAction;
    
    printTimers();
    //cont=0;         
    contCalc=0; 
    countCtrlAction=0;    
    contGyroSamples=0;   
    servoTimeTot = 0;
  }

  timerRoutine = micros()-kMRoutine;
  
  // The following loop runs every 1s
  if (timerRoutine >= deltaT*1000) 
  {      
    kMRoutine = micros();    
    
    printRoutine();
  }
}

// Send accelerometer filter value to Matlab for dynamic change
void sendAlphaAcc()
{
  Serial.print("f,");
  Serial.print(alphaA);
  Serial.println(",z");
}

// Send gyro filter value to Matlab for dynamic change
void sendAlphaW()
{
  Serial.print("f,");
  Serial.print(alphaW);
  Serial.println(",z");
}

// Send pid value feedback to App
void sendPidVal(int which,int mode)
{
  /*           WHICH:
   * Roll = 0
   * Pitch = 1 
   * Yaw = 2
   * W Roll = 3
   * W Pitch = 4 
   * W Yaw = 5
   *           MODE
   * Cons = 0
   * Agg = 1
   */
  if (which == 0)
  {
    if (mode == 0)
    {
      Serial.print("rc,");
      Serial.print(consKpCascRoll);
      Serial.print(",");
      Serial.print(consKiCascRoll);
      Serial.print(",");
      Serial.print(consKdCascRoll);
      Serial.print(",");
      Serial.print(SetpointCascRoll);
      Serial.println(",z");
    } 
    else if (mode = 1)
    {
      Serial.print("ra,");
      Serial.print(aggKpCascRoll);
      Serial.print(",");
      Serial.print(aggKiCascRoll);
      Serial.print(",");
      Serial.print(aggKdCascRoll);
      Serial.print(",");
      Serial.print(SetpointCascRoll);
      Serial.println(",z");
    }                     
  }
  if (which == 1)
  {
    if (mode == 0)
    {
      Serial.print("pc,");
      Serial.print(consKpCascPitch);
      Serial.print(",");
      Serial.print(consKiCascPitch);
      Serial.print(",");
      Serial.print(consKdCascPitch);
      Serial.print(",");
      Serial.print(SetpointCascPitch);
      Serial.println(",z");
    } 
    else if (mode = 1)
    {
      Serial.print("pa,");
      Serial.print(aggKpCascPitch);
      Serial.print(",");
      Serial.print(aggKiCascPitch);
      Serial.print(",");
      Serial.print(aggKdCascPitch);
      Serial.print(",");
      Serial.print(SetpointCascPitch);
      Serial.println(",z");
    }                     
  }
  if (which == 2)
  {
    if (mode == 0)
    {
      Serial.print("yc,");
      Serial.print(consKpCascYaw);
      Serial.print(",");
      Serial.print(consKiCascYaw);
      Serial.print(",");
      Serial.print(consKdCascYaw);
      Serial.print(",");
      Serial.print(SetpointCascYaw);
      Serial.println(",z");
    } 
    else if (mode = 1)
    {
      Serial.print("ya,");
      Serial.print(aggKpCascYaw);
      Serial.print(",");
      Serial.print(aggKiCascYaw);
      Serial.print(",");
      Serial.print(aggKdCascYaw);
      Serial.print(",");
      Serial.print(SetpointCascYaw);
      Serial.println(",z");
    }                     
  }
  if (which == 3)
  {
    if (mode == 0)
    {
      Serial.print("rw,");
      Serial.print(consKpCascRollW);
      Serial.print(",");
      Serial.print(consKiCascRollW);
      Serial.print(",");
      Serial.print(consKdCascRollW);
      Serial.print(",");
      Serial.print(SetpointCascRollW);
      Serial.println(",z");
    }     
  }
  if (which == 4)
  {
    if (mode == 0)
    {
      Serial.print("pw,");
      Serial.print(consKpCascPitchW);
      Serial.print(",");
      Serial.print(consKiCascPitchW);
      Serial.print(",");
      Serial.print(consKdCascPitchW);
      Serial.print(",");
      Serial.print(SetpointCascPitchW);
      Serial.println(",z");
    } 
  }
  if (which == 5)
  {
    if (mode == 0)
    {
      Serial.print("yw,");
      Serial.print(consKpCascYawW);
      Serial.print(",");
      Serial.print(consKiCascYawW);
      Serial.print(",");
      Serial.print(consKdCascYawW);
      Serial.print(",");
      Serial.print(SetpointCascYawW);
      Serial.println(",z");
    } 
  }
}

void printTimers()
{
    if (sakura.getPrintTimers())
    {
      // Print Samples rate: [sample/sec] 
      Serial.print("t,");
      Serial.print(contGyroSamples);
      // Print     ControlInput: 
      Serial.print(",");
      Serial.print(countCtrlAction);
      // Print    timeservo: 
      Serial.print(",");
      Serial.print(servoTimeTot);
      Serial.println(",z");
      Serial.println();
    }
}

void printRoutine()
{  
  /*  // #doing 
  */
  if (sakura.getPrintMotorValsUs())
  {
    Serial.print("V,  ");
    Serial.print(tenzoProp.getwUs1());
    Serial.print(" | ");
    Serial.print(tenzoProp.getwUs2());
    Serial.print(" | ");
    Serial.print(tenzoProp.getwUs3());
    Serial.print(" | ");
    Serial.println(tenzoProp.getwUs4());
  }
    
  if (sakura.getPrintAccs())
    printAcc();
  if (sakura.getPrintOmegas())
    printOmega();
    
  if (sakura.getSendBlueAngle())
  {
   printSerialAngleFus();
   //printSerialAngleNew();
  }
  
  if (sendStatesRemote)
    sendStates();
  //printT();
}

void sendStates()
{
  Serial.print("S,");
  Serial.print(initialized);
  Serial.print(",");
  Serial.print(hovering);
  Serial.print(",");
  Serial.print(landed);
  Serial.println(",z");
  
  sendStatesRemote = false;
}

void landFast()
{
  for (int j=tenzoProp.getThrottle(); j>40 ;j--)
  {
    tenzoProp.setSpeedWUs(j);
    //Serial.println(j);
    // Kind or brutal land
    delay(motorRampDelayFast);
  }  
  tenzoProp.idle();
  landed = 0;
}

void initializeFast()
{
  if (tenzoProp.getThrottle() == MIN_SIGNAL)
  {
   for (int j=MIN_SIGNAL; j<rampTill;j++)
   {
      tenzoProp.setSpeedWUs(j);
      //Serial.println(j);
      delay(motorRampDelayFast); 
   }
  }
  else if (tenzoProp.getThrottle()<=rampTill)
  {
   for (int j=tenzoProp.getThrottle() ; j<rampTill;j++)
   {
      tenzoProp.setSpeedWUs(j);
      //Serial.println(j);
      delay(motorRampDelayFast); 
   }
  }
  else if (tenzoProp.getThrottle()<=rampTill)
  {
   Serial.println("V,TODO: initialize FAST called unexpectedly");
  }
  tenzoProp.setThrottle(rampTill);
}


void initialize()
{
  if (!initialized)
  {
    if (!sakura.getProcessing())
        Serial.println("V,Initializing");
    initializing = true;
    tenzoProp.resetMotors();
    delay(500);
    for (int j=MIN_SIGNAL; j<rampTill;j++)
    {
      tenzoProp.setSpeeds(j, OutputCascPitchW, OutputCascRollW, OutputCascYawW, OutputCascAlt);
      //motorSpeed(j);
      if (!sakura.getProcessing())
        Serial.println(j);
      delay(motorRampDelayMedium); 
    }
    tenzoProp.setThrottle(rampTill);
    
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
    hovering = 0;
    landed = 0;
  }
  else
  {
    Serial.println();
    Serial.print("V,First Land ortherwise Tenzo will crash");
    Serial.println();
  }
  sendStatesRemote = true;
}


void resetMotorsPidOff()
{
  tenzoProp.setThrottle(MIN_SIGNAL);
  tenzoProp.setSpeedWUs(MIN_SIGNAL);
  // Sets timerStart to 0
  timerStart = 0;
  checkpoint = 0;
  // Disable Pid when motors are off
  if (enablePid)
  {
    // Change only if PID is enabled 
    // Tenzo has landed, no need to control
    changePidState(false);
  }
} 



void land()
{
  if (initialized)
  {
    landing = true;
    if(!sakura.getProcessing())
    {
      Serial.println();
      Serial.print("V,Landing protocol started...");
      Serial.print(tenzoProp.getThrottle());
      Serial.print(" ");
    }
    for (int j=tenzoProp.getThrottle(); j>MIN_SIGNAL ;j--)
    //for (int j=throttle; j>0 ;j--)
    {      
      tenzoProp.setSpeeds(j, OutputCascPitchW, OutputCascRollW, OutputCascYawW, OutputCascAlt);
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
    landing = false;;
    // updateStates
    landed=1;
    takeOff=0;
    hovering=0;
    sendStatesRemote = true;
  }
  else
  {
    Serial.println();
    Serial.print("V,Land command received but Tenzo is not Flying   !! WARNING !!");
    Serial.println();
    sendStatesRemote = true;
  }
}


void protocol1()
{  
  if (autoLand)
  {
    // If motors are on updates timer
    if (initialized)
      timerStart = millis() - checkpoint;
    // if Tenzo has already been initialized for a timeToLand period, then land
    if (initialized && timerStart>=timeToLand)
    {
      Serial.println("V,Time to Land my friend!");
      land();
    }
  }
}

void printOmega()
{
  Serial.print("o,");
  Serial.print(wVal[0]);
  Serial.print(",");
  Serial.print(wVal[1]);
  Serial.print(",");
  Serial.print(wVal[2]);
  Serial.println(",z");
}

void printAcc()
{
  if (!filterAcc)
  {
    Serial.print("a,");
    Serial.print(aax);
    Serial.print(",");
    Serial.print(aay);
    Serial.print(",");
    Serial.print(aaz);
    Serial.println(",z");
  }
  else
  {
    Serial.print("a,");
    Serial.print(aF[0]);
    Serial.print(",");
    Serial.print(aF[1]);
    Serial.print(",");
    Serial.print(aF[2]);
    Serial.println(",z");
  }
  
  /*
  Serial.print("a,");
  Serial.print(angleXAcc);
  Serial.print(",");
  Serial.print(angleYAcc);
  Serial.print(",");
  Serial.print(estXAngle);
  Serial.print(",");
  Serial.print(estYAngle);
  Serial.println(",z");
  */
  
}

void printSerialAngle()
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
  Serial.print(bearing1);
  Serial.println(",z");
}

void printSerialAngleNew()
{
  Serial.print("P");
  Serial.print(",");
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
  Serial.print(bearing1);
  Serial.println(",z");
}

void printSerialAngleFus()
{
  Serial.print("e");
  Serial.print(",");
  Serial.print(angles[0]);
  Serial.print(",");
  Serial.print(angles[1]);
  Serial.print(",");
  Serial.print(angles[2]);
  Serial.println(",z");
}

void controlCascade()  // ISR
{
  if (enablePid)
  {
    /*
     *
     *  Cascade Roll 
     *
     */
    if (enableRollPid)
    {
      InputCascRoll = angles[0];
      errorCascRoll = abs(SetpointCascRoll - angles[0]);
      
      if (inConsRoll)
      {
        if (errorCascRoll<=(thresholdRoll + 3))
        {
          cascadeRollPid.SetTunings(consKpCascRoll, consKiCascRoll, consKdCascRoll);
          inConsRoll = true; 
        }
        else if (errorCascRoll>thresholdRoll)
        {
          cascadeRollPid.SetTunings(aggKpCascRoll, aggKiCascRoll, aggKdCascRoll);
          inConsRoll = false;
        }
      }
      else
      {
        if (errorCascRoll<=(thresholdRoll))
        {
          cascadeRollPid.SetTunings(consKpCascRoll, consKiCascRoll, consKdCascRoll);
          inConsRoll = true; 
        }
        else if (errorCascRoll>thresholdRoll)
        {
          cascadeRollPid.SetTunings(aggKpCascRoll, aggKiCascRoll, aggKdCascRoll); 
          inConsRoll = false;
        }
      }
      
      OutputCascRoll = cascadeRollPid.Compute(SetpointCascRoll,InputCascRoll); 
      
      InputCascRollW = x;
      SetpointCascRollW = OutputCascRoll;
      //////////////////////////////////////
      //qSetpointCascRollW = 0;
      errorCascRollW = SetpointCascRollW - x; 

      cascadeRollPidW.SetTunings(consKpCascRollW, consKiCascRollW, consKdCascRollW);

      OutputCascRollW = cascadeRollPidW.Compute(SetpointCascRollW,InputCascRollW);     
      /*
      if (sakura.getPrintPIDVals())
      {
        Serial.print("V,I: ");
        Serial.print(InputCascRoll);
        Serial.print(" O: ");
        Serial.print(OutputCascRoll);
        Serial.print(" Iw: ");
        Serial.print(InputCascRollW);
        Serial.print(" Ew: ");
        Serial.print(errorCascRollW);
        Serial.print(" O: ");
        Serial.print(OutputCascRollW);
        Serial.println();
      }
      */
    }
    else
    {
      OutputCascRollW = 0;
    }
    
    /*
     *
     *  Cascade Pitch 
     *
     */
    if (enablePitchPid)
    {
      InputCascPitch = angles[1];
      errorCascPitch = abs(SetpointCascPitch - angles[1]);
      
      if (inConsPitch)
      {
        if (errorCascPitch<=(thresholdPitch + 3))
        {
          cascadePitchPid.SetTunings(consKpCascPitch, consKiCascPitch, consKdCascPitch);
          inConsPitch = true; 
        }
        else if (errorCascPitch>thresholdRoll)
        {
          cascadePitchPid.SetTunings(aggKpCascPitch, aggKiCascPitch, aggKdCascPitch);
          inConsPitch = false;
        }
      }
      else
      {
        if (errorCascPitch<=(thresholdPitch))
        {
          cascadePitchPid.SetTunings(consKpCascPitch, consKiCascPitch, consKdCascPitch);
          inConsPitch = true; 
        }
        else if (errorCascPitch>thresholdPitch)
        {
          cascadePitchPid.SetTunings(aggKpCascPitch, aggKiCascPitch, aggKdCascPitch); 
          inConsPitch = false;
        }
      }
      
      OutputCascPitch = cascadePitchPid.Compute(SetpointCascPitch,InputCascPitch); 
      
      InputCascPitchW = y;
      SetpointCascPitchW = OutputCascPitch;
      //SetpointCascPitchW = 0;
      errorCascPitchW = SetpointCascPitchW - y; 

      cascadePitchPidW.SetTunings(consKpCascPitchW, consKiCascPitchW, consKdCascPitchW);

      OutputCascPitchW = cascadePitchPidW.Compute(SetpointCascPitchW,InputCascPitchW);     
      
      /*
      if (sakura.getPrintPIDVals())
      {
        Serial.print("I: ");
        Serial.print(InputCascRoll);
        Serial.print(" O: ");
        Serial.print(OutputCascRoll);
        Serial.print(" Iw: ");
        Serial.print(InputCascRollW);
        Serial.print(" Ew: ");
        Serial.print(errorCascRollW);
        Serial.print(" O: ");
        Serial.print(OutputCascPitchW);
        Serial.println();
      }
      */
    }
    else
    {                        
      OutputCascPitchW = 0;
    }
  }
  
  // Set computed speed
  tenzoProp.setSpeeds(tenzoProp.getThrottle(), OutputCascPitchW, OutputCascRollW, OutputCascYawW, OutputCascAlt);
  // update counter control  
  countCtrlAction++;
}


void changePidState(boolean cond)
{
  if (cond)
  {
    // Enable Cascade
    cascadeRollPid.restart();
    cascadeRollPid.SetOutputLimits(-limitPidMax, limitPidMax);
    cascadeRollPidW.restart();
    cascadeRollPidW.SetOutputLimits(-limitPidMax, limitPidMax);
    
    cascadePitchPid.restart();
    cascadePitchPid.SetOutputLimits(-limitPidMax, limitPidMax);
    cascadePitchPidW.restart();
    cascadePitchPidW.SetOutputLimits(-limitPidMax, limitPidMax);
    
    
    cascadeYawPid.restart();
    cascadeYawPid.SetOutputLimits(-limitPidMax, limitPidMax);
    cascadeYawPidW.restart();
    cascadeYawPidW.SetOutputLimits(-limitPidMax, limitPidMax);
    
    cascadeAltPid.restart();
    cascadeAltPid.SetOutputLimits(-limitPidMax, limitPidMax);
    

    enablePid = true;
    hovering = 1;
  }
  else
  { 
    // Cascade
    cascadeRollPid.pause();
    cascadeRollPidW.pause();
    cascadePitchPid.pause();
    cascadePitchPidW.pause();
    cascadeYawPid.pause();
    cascadeYawPidW.pause();
    cascadeAltPid.pause();
    
    OutputCascPitch = 0,OutputCascPitchW = 0;
    OutputCascRoll = 0, OutputCascRollW = 0;
    OutputCascYaw = 0, OutputCascYawW = 0;
    OutputCascAlt = 0;
    
    enablePid = false;
    hovering = 0;
  } 
}
