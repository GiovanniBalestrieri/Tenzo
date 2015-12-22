#include <Servo.h>
#include <Wire.h>
#include "Propulsion.h"
#include "Ux.h"
#include "PID_v2.h"
#include "NonLinearPid.h"
#include "MedianFilter.h"

Ux sakura;
Propulsion tenzoProp(sakura.getM(1),sakura.getM(2),sakura.getM(3),sakura.getM(4));

//NonLinearPid controlCascade(SetpointRoll,SetpointPitch,SetpointYaw,SetpointAltitude);


/*
 * deltaT : Control loop frequency (Verbose_motors /NonVerbose)
 * 15: 50Hz
 * 8 : 75HZ/44Hz    
 * 7 : 110Hz/56Hz
 */
float deltaT = 15;  

char readAnswer, readChar, readCh;

// REMOVE

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
int rampTill = 1270;
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
boolean enablePid = false;

// theta
boolean enableRollPid = true;
boolean enablePitchPid = false;
boolean enableYawPid = false;
// w
boolean enableWRollPid = true;
boolean enableWPitchPid = false;
boolean enableWYawPid = false;
boolean enableAltitudePid = false;

int limitPidMax = 250;
 
boolean inConsRoll = false; 
boolean inConsPitch = false;
boolean verbosePidValuesFeedback = true;
boolean verboseFilterAccMatlab = true;
// Define IO and setpoint for control
//double SetpointRoll = 0, InputRoll, errorRoll;
// Define the aggressive and conservative Tuning Parameters
        
        
        // Angular position
        float aggKpRoll=1.0, aggKiRoll=0, aggKdRoll=0.00; 
        float consKpRoll=2.5, consKiRoll=1.4, consKdRoll=0.4;
//        float consKpRoll=3.00, consKiRoll=2, consKdRoll=0.03;
        float farKpRoll=0.5, farKiRoll=0.17, farKdRoll=0.00;
        
        // Angle Pitch
        float aggKpPitch=0.07, aggKiPitch=0.06, aggKdPitch=0.04;
        float consKpPitch=2.00, consKiPitch=3.00, consKdPitch=0.00;
        float farKpPitch=0.02, farKiPitch=0.09,  farKdPitch=0.02;
        
        // Angle Yaw
        double aggKpYaw=0.3, aggKiYaw=0.0, aggKdYaw=0.1;
        double consKpYaw=0.3, consKiYaw=0, consKdYaw=0.0;
        
        
        // Altitude  
        double aggKpAltitude=0.2, aggKiAltitude=0.0, aggKdAltitude=0.1;
        double consKpAltitude=0.1, consKiAltitude=0, consKdAltitude=0.1;
        
        ///////////////////////  WWWWWWWWWWWWWWW  //////////////////////
        
        // W Roll
        float aggKpWRoll=0.02, aggKiWRoll=0.2, aggKdWRoll=0.00;
        //float consKpWRoll=0.95, consKiWRoll=1.2, consKdWRoll=0.13;  50 HZ
        float consKpWRoll=0.50, consKiWRoll=0.80, consKdWRoll=0.14;    
        float farKpWRoll=0.05, farKiWRoll=0.06, farKdWRoll=0.03;
        
        // W Pitch
        float aggKpWPitch=0.07, aggKiWPitch=0.06, aggKdWPitch=0.04;
        float consKpWPitch=1, consKiWPitch=2, consKdWPitch=0.0;
        float farKpWPitch=0.02, farKiWPitch=0.09,  farKdWPitch=0.02;
        
        // W Yaw
        double aggKpWYaw=0.3, aggKiWYaw=0.0, aggKdWYaw=0.1;
        double consKpWYaw=0.3, consKiWYaw=0, consKdWYaw=0.0;
           
        /*
         * Cascade Pid & settings
         */
         
        // Roll

        // Aggressive settings theta >= thre     
        float aggKpCascRoll=1.1, aggKiCascRoll=0.00, aggKdCascRoll=0.00;
        // Conservative settings theta < thre
        float consKpCascRoll=1.1, consKiCascRoll=0.00, consKdCascRoll=0.00; //1.5 / 3.2 0.6 0.4
        
        // W part   
        float consKpCascRollW=0.90, consKiCascRollW=1.65, consKdCascRollW=0.20;   
        
        // Pitch
        
        
        // Aggressive settings theta >= thre     
        float aggKpCascPitch=1.1, aggKiCascPitch=0.00, aggKdCascPitch=0.00;
        // Conservative settings theta < thre
        float consKpCascPitch=1.1, consKiCascPitch=0.00, consKdCascPitch=0.00; //1.5 / 3.2 0.6 0.4
        
        // W part   
        float consKpCascPitchW=0.9, consKiCascPitchW=1.65, consKdCascPitchW=0.3;   
        
        // Yaw
        
        // Aggressive settings theta >= thre     
        float aggKpCascYaw=0, aggKiCascYaw=0.00, aggKdCascYaw=0.00;
        // Conservative settings theta < thre
        float consKpCascYaw=0, consKiCascYaw=0.00, consKdCascYaw=0.00; //1.5 / 3.2 0.6 0.4
        
        // W part   
        float consKpCascYawW=0.7, consKiCascYawW=0.01, consKdCascYawW=0.5; 
        
        // Aggressive settings theta >= thre     
        float consKpCascAlt=2.5, consKiCascAlt=0.00, consKdCascAlt=0.00;
                 
        double SetpointCascRoll = 0, InputCascRoll, errorCascRoll;
        double SetpointCascRollW = 0, InputCascRollW, errorCascRollW;        
        double SetpointCascPitch = 0, InputCascPitch, errorCascPitch;       
        double SetpointCascPitchW = 0, InputCascPitchW, errorCascPitchW;
        double SetpointCascYaw = 180, InputCascYaw, errorCascYaw;
        double SetpointCascYawW = 180, InputCascYawW, errorCascYawW;
        double SetpointCascAlt = 1, InputCascAlt, errorCascAlt, OutputCascAlt = 0;    
        
        
        double OutputCascRoll = 0;
        double OutputCascPitch = 0;
        double OutputCascYaw = 0;
        double OutputCascAltitude = 0;
        double OutputCascRollW = 0;
        double OutputCascPitchW = 0;
        double OutputCascYawW = 0;
        
         

        PID cascadeRollPid(&InputCascRoll, &OutputCascRoll, &SetpointCascRoll, consKpCascRoll, consKiCascRoll, consKdCascRoll, DIRECT);
        PID cascadeRollPidW(&InputCascRollW, &OutputCascRollW, &SetpointCascRollW, consKpCascRollW, consKiCascRollW, consKdCascRollW, DIRECT);
        
        
        PID cascadePitchPid(&InputCascPitch, &OutputCascPitch, &SetpointCascPitch, consKpCascPitch, consKiCascPitch, consKdCascPitch, DIRECT);
        PID cascadePitchPidW(&InputCascPitchW, &OutputCascPitchW, &SetpointCascPitchW, consKpCascPitchW, consKiCascPitchW, consKdCascPitchW, DIRECT);
        
        
        PID cascadeYawPid(&InputCascYaw, &OutputCascYaw, &SetpointCascYaw, consKpCascYaw, consKiCascYaw, consKdCascYaw, DIRECT);
        PID cascadeYawPidW(&InputCascYawW, &OutputCascYawW, &SetpointCascYawW, consKpCascYawW, consKiCascYawW, consKdCascYawW, DIRECT);
        
        
        PID cascadeAltPid(&InputCascAlt, &OutputCascAlt, &SetpointCascAlt, consKpCascAlt, consKiCascAlt, consKdCascAlt, DIRECT);
        
        
        // Define IO and setpoint for control
        double SetpointRoll = -1, InputRoll, errorRoll;        
        double SetpointPitch = 0, InputPitch, errorPitch;
        double SetpointYaw = 180, InputYaw, errorYaw;
        double SetpointAltitude = 1, InputAltitude, errorAltitude;                
        // Define IO and setpoint for control -----------  W
        double SetpointWRoll = 0, InputWRoll = 0, errorWRoll;
        double SetpointWPitch = 0, InputWPitch = 0, errorWPitch;
        double SetpointWYaw = 180, InputWYaw = 0, errorWYaw;
        
        double OutputRoll = 0;
        double OutputPitch = 0;
        double OutputYaw = 0;
        double OutputAltitude = 0;
        double OutputWRoll = 0;
        double OutputWPitch = 0;
        double OutputWYaw = 0;



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

//Specify the links and initial tuning parameters
PID rollPid(&InputRoll, &OutputRoll, &SetpointRoll, consKpRoll, consKiRoll, consKdRoll, DIRECT);
PID pitchPid(&InputPitch, &OutputPitch, &SetpointPitch, consKpPitch, consKiPitch, consKdPitch, DIRECT);
PID yawPid(&InputYaw, &OutputYaw, &SetpointYaw, consKpYaw, consKiYaw, consKdYaw, DIRECT);
PID altitudePid(&InputAltitude, &OutputAltitude, &SetpointAltitude, consKpAltitude, consKiAltitude, consKdAltitude, DIRECT);


//Specify the links and initial tuning parameters
PID wrollPid(&InputWRoll, &OutputWRoll, &SetpointWRoll, consKpWRoll, consKiWRoll, consKdWRoll, DIRECT);
PID wpitchPid(&InputWPitch, &OutputWPitch, &SetpointWPitch, consKpWPitch, consKiWPitch, consKdWPitch, DIRECT);
PID wyawPid(&InputWYaw, &OutputWYaw, &SetpointWYaw, consKpWYaw, consKiWYaw, consKdWYaw, DIRECT);

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
unsigned long servoTime = 0;



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



// Volatile vars
volatile int cont = 0;
int countCtrlAction = 0;
volatile int contSamples=0;
volatile int contCalc=0;

volatile float thetaOLD = 0;
volatile float phi=0;
volatile float theta=0;
volatile float psi=0;
volatile int x=0;
volatile int y = 0;
volatile int z= 0;
volatile int rawAx = 0;
volatile int rawAy = 0;
volatile int rawAz = 0;
volatile int dt=0;

volatile float wF[3];
volatile float aF[3] = {0,0,0};
volatile boolean filterGyro = false;
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
volatile float alphaA= 0.995, alphaW = 0.8;
volatile float estXAngle = 0, estYAngle = 0, estZAngle = 0;
volatile float kG = 0.975, kA = 0.025, kGZ=0.60, kAZ = 0.40;

void setupAcceleromter()
{
  pinMode(xaxis,INPUT);
  pinMode(yaxis,INPUT);
  pinMode(zaxis,INPUT);
  
//  #ifdef MMA7820Q
//    XError =  AccelAdjust(xaxis);
//    YError =  AccelAdjust(yaxis);
//    ZError =  AccelAdjust(zaxis);
//    ZError = ZError - ZOUT_1G;
//  #endif
  if (!sakura.getProcessing())
  {
    Serial.println("[OK] Initializing accelerometer");
  }
}

/** 
 ** Servo initialization
 **/
 

void setupGyro()
{
  if (!sakura.getProcessing())
  {
    Serial.println("       Init Gyro");
  }
  
  // Configure L3G4200  - 250, 500 or 2000 deg/sec
  setupL3G4200D(2000); 
  //wait for the sensor to be ready   
  delay(1500); 
  
  biasCalcTime = micros();
  calcBias();
  biasCalcTime = micros() - biasCalcTime;
  
  if (!sakura.getProcessing())
  {
    Serial.print("Readings: [us] ");
    Serial.print(samplingTime);
    Serial.print("      Bias est: [us] ");
    Serial.print(biasCalcTime);
    Serial.print("      Samples: ");
    Serial.println(contSamples);
  }
  if (!sakura.getProcessing())
  {
    Serial.println("[ OK ] Init Gyro");
  }
}

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
  OCR3A=103; //16*10^6/(150Hz*1024)-1 = 103 -> 150 Hz 
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
    Serial.println("[Ok] InitCOM ");
  }
}


/// TILL HERE



void setup() {
  setupCommunication();
  setupAcceleromter();
  setupGyro();
  setupTimerInterrupt();  
  setupCtx();  
  sakura.welcome();
  tenzoProp.calibrateOnce();
  tenzoProp.init();
}

void loop() {  
  SerialRoutine();
  tenzoProp.setSpeedWUs(tenzoProp.getThrottle());
}

void getAcc() //ISR
{
   rawAx=analogRead(xaxis);
   rawAy=analogRead(yaxis);
   rawAz=analogRead(zaxis);
  
  // OLD con MMA7260Q
//   aax = (((rawAx*5000.0)/1023.0)-XError)/RESOLUTION;
//   aay = (((rawAy*5000.0)/1023.0)-YError)/RESOLUTION;
//   aaz = (((rawAz*5000.0)/1023.0)-ZError)/RESOLUTION;

  float xScaled = map(rawAx, xRawMin, xRawMax, -1000, 1000);
  float yScaled = map(rawAy, yRawMin, yRawMax, -1000, 1000);
  float zScaled = map(rawAz, zRawMin, zRawMax, -1000, 1000);
  
  //float zScaled = zm;
  
  // re-scale to fractional Gs
  aax = xScaled / 1000.0;
  aay = yScaled / 1000.0;
  aaz = zScaled / 1000.0;
   
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

void estAngle() // ISR
{
  estXAngle = (estXAngle + x*(float)dt/1000000.0)*kG + angleXAcc*kA;
  estYAngle = (estYAngle + y*(float)dt/1000000.0)*kG + angleYAcc*kA;
  //estZAngle = (estZAngle + z*(float)dt/1000000.0)*0.02 + bearing1*0.98;
  //psi*KG + yaw*KA;
}

void wFilter(volatile float val[])
{
  val[0] = (1-alphaW)*x + alphaW*val[0];
  val[1] = (1-alphaW)*y + alphaW*val[1];
  val[2] = (1-alphaW)*z + alphaW*val[2];
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
  getCompassValues();
  calcAngle();
  estAngle();
  cont++;
}

void SerialRoutine()
{
  if (Serial.available())
  {
      char t = Serial.read();
      
      if (t == '1')
        tenzoProp.stopAll();      
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
          Serial.print("V, A - P|I|D: ");
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
        Serial.println("i,z");
      }
      else if (t == 'L')
      {
        land();
        sendStatesRemote = true;
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
        sakura.setPrintPIDVals(!sakura.getPrintPIDVals());
      }
      else if (t == 'p')
      {
        enablePid = !enablePid;
        changePidState(enablePid);
        if (enablePid)
          Serial.println("ce,z");
        else if (!enablePid)
          Serial.println("cd,z");
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
        sakura.setPrintMotorValsUs(!sakura.getPrintMotorValsUs());
      }      
      else if (t == 't')
      {
        SetpointCascRoll = SetpointCascRoll - 1;
        Serial.print("V,SetpointCascRoll:  ");
        Serial.println(SetpointCascRoll);
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
          //Serial.print("consKpCascRoll:  ");
          //Serial.println(consKpCascRoll);
          sendPidVal(0,0);
        }          
      }         
      else if (t == 'D')
      {
        consKpCascRoll = consKpCascRoll + 0.05;
        if (verbosePidValuesFeedback)
        {
          //Serial.print("consKpCascRoll:  ");
          //Serial.println(consKpCascRoll);
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
          //Serial.print("consKICascRoll:  ");
          //Serial.println(consKiCascRoll);
          sendPidVal(0,0); 
        }
      }         
      else if (t == 'F')
      {
        consKiCascRoll = consKiCascRoll + 0.05;
        if (verbosePidValuesFeedback)
        {
          //Serial.print("consKiCascRoll:  ");
          //Serial.println(consKiCascRoll);
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
          //Serial.print("consKDCascRoll:  ");
        //Serial.println(consKdCascRoll);
          sendPidVal(0,0);
        }
      }         
      else if (t == 'G')
      {
        consKdCascRoll = consKdCascRoll + 0.05;
        if (verbosePidValuesFeedback)
        {
          ///Serial.print("consKDCascRoll:  ");
          //Serial.println(consKdCascRoll);
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
          //Serial.print("consKpCascRoll W:  ");
          //Serial.println(consKpCascRollW);
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
        alphaA = alphaA + 0.001;
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
  }
  timerSec = micros()-secRoutine;
  lastTimeToRead = micros();
   
  if (timerSec >= 1000000)
  {
    secRoutine = micros();
    printTimers();
    cont=0;      
    contSamples=0;      
    contCalc=0; 
    countCtrlAction=0;
  }

  timerRoutine = micros()-kMRoutine;
  
  // The following loop runs every 1ms
  if (timerRoutine >= deltaT*1000) 
  {      
    kMRoutine = micros();    
    count += 1;
    if (count >= 1)
    {
      count = 0;
      
      //control();  
      controlCascade();
      //tenzoProp.setSpeeds(tenzoProp.getThrottle(), OutputWPitch, OutputWRoll, OutputWYaw, OutputAltitude);
      //controlW();
      tenzoProp.setSpeeds(tenzoProp.getThrottle(), OutputCascPitchW, OutputCascRollW, OutputCascYawW, OutputCascAlt);
      
      countCtrlAction++;
      printRoutine();
      
      // Updates counters
      servoTime = micros();
      servoTime = micros() - servoTime;
    }
  }
}

// Send accelerometer filter value to Matlab for dynamic change
void sendAlphaAcc()
{
  Serial.print("f,");
  Serial.print(alphaA);
  Serial.println(",z");
}

// Send pid value feedback to App
void sendPidVal(int which,int mode)
{
  /*           WHICH:
   * Roll = 0
   * Pitch = 1 
   * Yaw = 2
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
      Serial.print("rcw,");
      Serial.print(consKpCascRollW);
      Serial.print(",");
      Serial.print(consKiCascRollW);
      Serial.print(",");
      Serial.print(consKdCascRollW);
      Serial.print(",");
      Serial.print(SetpointCascRoll);
      Serial.println(",z");
    }     
  }
  if (which == 4)
  {
    if (mode == 0)
    {
      Serial.print("pcw,");
      Serial.print(consKpCascPitchW);
      Serial.print(",");
      Serial.print(consKiCascPitchW);
      Serial.print(",");
      Serial.print(consKdCascPitchW);
      Serial.print(",");
      Serial.print(SetpointCascPitch);
      Serial.println(",z");
    } 
  }
  if (which == 5)
  {
    if (mode == 0)
    {
      Serial.print("ycw,");
      Serial.print(consKpCascYawW);
      Serial.print(",");
      Serial.print(consKiCascYawW);
      Serial.print(",");
      Serial.print(consKdCascYawW);
      Serial.print(",");
      Serial.print(SetpointCascYaw);
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
      Serial.print(contSamples);
      // Print     ControlInput: 
      Serial.print(",");
      Serial.println(countCtrlAction);
      // Print    timeservo: 
      Serial.print(",");
      Serial.print(servoTime);
      Serial.println(",z");
      Serial.println();
    }
}

void printRoutine()
{  
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

void getCompassValues()
{
  requestCompassData();
  getCompassData();
  // bearing1 =  my_compass.bearing();
  bearing1 = (float) -(bearing + fine*0.10);
  // workaround
  bearing1 = -bearing1;
}

void requestCompassData()
{
   //starts communication with CMPS10
   Wire.beginTransmission(ADDRESS); 
   //Sends the register we wish to start reading from   
   Wire.write(2);                 
   Wire.endTransmission();             

   // Request 4 bytes from CMPS10
   Wire.requestFrom(ADDRESS, 4); 
   // Wait for bytes to become available   
   while(Wire.available() < 4);              
   highByte = Wire.read();           
   lowByte = Wire.read();            
   pitch = Wire.read();              
   roll = Wire.read();               
}

void getCompassData()
{
   // Calculate full bearing
   bearing = ((highByte<<8)+lowByte)/10;  

   // Calculate decimal place of bearing   
   fine = ((highByte<<8)+lowByte)%10;   
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
      tenzoProp.setSpeeds(j, OutputPitch, OutputRoll, OutputYaw, OutputAltitude);
      //motorSpeed(j);
      //if (!processing)
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
  }
  else
  {
    Serial.println();
    Serial.print("V,First Land ortherwise Tenzo will crash");
    Serial.println();
  }
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
    // Tenzo has landed, no need of controls
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
      tenzoProp.setSpeeds(j, OutputWPitch, OutputWRoll, OutputWYaw, OutputAltitude);
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
  }
  else
  {
    Serial.println();
    Serial.print("V,Land command received but Tenzo is not Flying   !! WARNING !!");
    Serial.println();
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


void getGyroValues()
{  
  // Get Data if available
  volatile byte statusflag = readRegister(L3G4200D_Address, STATUS_REG);
  while(!(statusflag & ZDA_REG) && (statusflag & ZOR_REG)&&!(statusflag & YDA_REG) && (statusflag & YOR_REG)&& !(statusflag & XDA_REG) && (statusflag & XOR_REG)) 
  {
    statusflag = readRegister(L3G4200D_Address, STATUS_REG);
  }
  
  //starting samplingTimer
  samplingTime = micros();

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  int xC = ((xMSB << 8) | xLSB);
  medianGyroX.in(xC);
  x = medianGyroX.out();    
  //x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  int yC = ((yMSB << 8) | yLSB);
  medianGyroY.in(yC);
  y = medianGyroY.out();    

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  int zC = ((zMSB << 8) | zLSB);
  medianGyroZ.in(zC);
  z = medianGyroZ.out();    
  
  if (initializedGyroCalib)
    removeBiasAndScale();

  samplingTime = micros()- samplingTime;
  contSamples++;
}


void printOmega()
{
  if (filterGyro)
  {
    Serial.print("o,");
    Serial.print(wF[0]);
  
    Serial.print(",");
    Serial.print(wF[1]);
  
    Serial.print(",");
    Serial.print(wF[2]);
    Serial.print(",z");
  }
  else
  {
    Serial.print("o,");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.print(z);
    Serial.println(",z");
  }
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
  Serial.print(estXAngle);
  Serial.print(",");
  Serial.print(estYAngle);
  Serial.print(",");
  Serial.print(bearing1);
  Serial.println(",z");
}

void removeBiasAndScale()
{
  x = (x - bx)*scale2000/1000;
  y = (y - by)*scale2000/1000;
  z = (z - bz)*scale2000/1000;
} 

void calcBias()
{
  if (!sakura.getProcessing())
    Serial.println("V,Decting Bias ...");
  int c = 500;
  for (int i = 0; i<c; i++)
  {
    delay(5);
    getGyroValues(); 
    bxS = bxS + x;
    byS = byS + y;
    bzS = bzS + z;
    //Serial.println(i);
  }

  bx = bxS / c;
  by = byS / c;
  bz = bzS / c;

  if (!sakura.getProcessing())
  {
    Serial.println(bx);
    Serial.println(by);
    Serial.println(bz);
  }  
  initializedGyroCalib = true;
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
  //From  Jim Lindblom of Sparkfun's code

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
      
        rollPid.SetTunings(consKpRoll, consKiRoll, consKdRoll);
        
      rollPid.Compute(); // Computes outputRoll

      if (sakura.getPrintPIDVals())
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
    }
    else
    {
      Serial.println();
      Serial.println("Roll SS");
      Serial.println();
      OutputRoll = 0;
    }

    // Pitch PID1
    if (enablePitchPid)
    {
      InputPitch = estYAngle;
      errorPitch = abs(SetpointPitch - estYAngle); //distance away from setpoint
      pitchPid.SetTunings(consKpPitch, consKiPitch, consKdPitch);
      pitchPid.Compute(); // Computes outputPitch

      if (sakura.getPrintPIDVals())
      { 
        Serial.print("V,E:  ");
        Serial.print(errorPitch);
        Serial.print(" A:");
        Serial.print(OutputPitch);
        Serial.println();
      }
    }  
    else
    {
      Serial.println();
      Serial.println("V,SS Pitch");
      Serial.println();
      OutputPitch = 0;
    }

    if (sakura.getPrintPIDVals())
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

void controlCascade()
{
  if (enablePid)
  {
    if (enableRollPid)
    {
      InputCascRoll = estXAngle;
      errorCascRoll = abs(SetpointCascRoll - estXAngle);
      
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
      
      cascadeRollPid.Compute(); 
      
      InputCascRollW = x;
      SetpointCascRollW = -OutputCascRoll;
      //////////////////////////////////////77
      SetpointCascRollW = 0;
      errorCascRollW = SetpointCascRollW - x; 

      cascadeRollPidW.SetTunings(consKpCascRollW, consKiCascRollW, consKdCascRollW);

      cascadeRollPidW.Compute();     
     
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
    }
    else
    {
      Serial.println();
      Serial.println("V,Warning");
      Serial.println();
      OutputRoll = 0;
    }
    if (enablePitchPid)
    {
      InputCascPitch = estYAngle;
      errorCascPitch = abs(SetpointCascPitch - estYAngle);
      
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
      
      cascadePitchPid.Compute(); 
      
      InputCascPitchW = y;
      //SetpointCascPitchW = - OutputCascPitch;
      SetpointCascPitchW = 0;
      errorCascPitchW = SetpointCascPitchW - y; 

      cascadePitchPidW.SetTunings(consKpCascPitchW, consKiCascPitchW, consKdCascPitchW);

      cascadePitchPidW.Compute();     
      
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
        Serial.print(OutputCascRollW);
        Serial.println();
      }
      */
    }
    else
    {                                     // Cambia outputPitch forse con w
      Serial.println();
      Serial.println("V,Warning");
      Serial.println();
      OutputPitch = 0;
    }
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
      InputWRoll = x;
      errorWRoll = SetpointWRoll - x; 
      
        wrollPid.SetTunings(consKpWRoll, consKiWRoll, consKdWRoll);

      wrollPid.Compute();
      
      
      if (sakura.getPrintPIDVals())
      { 
        Serial.print("V,E:  ");
        Serial.print(errorWRoll);
        Serial.print(" A:");
        Serial.print(OutputWRoll);
        Serial.println();
      }
    }
    else
    {
      OutputWRoll = 0;
    }
    

    // Pitch PID1
    if (enableWPitchPid)
    {
      InputWPitch = y;
      errorWPitch = abs(SetpointWPitch - y);

      wpitchPid.SetTunings(consKpWPitch,consKiWPitch, consKdWPitch);

      wpitchPid.Compute();
    }  
    else
    {
      OutputWPitch = 0;
    }

    /*
    // Yaw PID
     if (enableWYawPid)
     {
     if (filterAng == 1)
     {
     InputWYaw = z;
     errorWYaw = abs(SetpointWYaw - z); //distance away from setpoint
     }
     
     if(errorYaw<thresholdYaw)
     {
     //we're close to setpoint, use conservative tuning parameters
     wyawPid.SetTunings(Kw*consKpWYaw, Kw*consKiWYaw, Kw*consKdWYaw);
     }   
     wyawPid.Compute(); 
     
     }
     else
     {
     OutputYaw=0;
     }
     */
    //if (printPIDVals)
    //{
      //Serial.println();      
    //}
  }
  else
  {
    OutputWRoll = 0;
    OutputWPitch = 0;    
    OutputWYaw = 0;
  }
}

void changePidState(boolean cond)
{
  if (cond)
  {
    // Enable Cascade
    cascadeRollPid.SetMode(AUTOMATIC);
    cascadeRollPid.SetOutputLimits(-limitPidMax, limitPidMax);
    cascadeRollPidW.SetMode(AUTOMATIC);
    cascadeRollPidW.SetOutputLimits(-limitPidMax, limitPidMax);
    
    cascadePitchPid.SetMode(AUTOMATIC);
    cascadePitchPid.SetOutputLimits(-limitPidMax, limitPidMax);
    cascadePitchPidW.SetMode(AUTOMATIC);
    cascadePitchPidW.SetOutputLimits(-limitPidMax, limitPidMax);
    
    
    cascadeYawPid.SetMode(AUTOMATIC);
    cascadeYawPid.SetOutputLimits(-limitPidMax, limitPidMax);
    cascadeYawPidW.SetMode(AUTOMATIC);
    cascadeYawPidW.SetOutputLimits(-limitPidMax, limitPidMax);
    
    
    cascadeAltPid.SetMode(AUTOMATIC);
    cascadeAltPid.SetOutputLimits(-limitPidMax, limitPidMax);
    
    // MONO
    rollPid.SetMode(AUTOMATIC);
    rollPid.SetOutputLimits(-limitPidMax, limitPidMax);

    // Pitch
    pitchPid.SetMode(AUTOMATIC);
    pitchPid.SetOutputLimits(-limitPidMax, limitPidMax);

    // Yaw
    yawPid.SetMode(AUTOMATIC);
    yawPid.SetOutputLimits(-limitPidMax, limitPidMax);

    // Enable Pid Actions
    wrollPid.SetMode(AUTOMATIC);
    wrollPid.SetOutputLimits(-limitPidMax, limitPidMax);

    // Pitch
    wpitchPid.SetMode(AUTOMATIC);
    wpitchPid.SetOutputLimits(-limitPidMax, limitPidMax);

    // Yaw
    wyawPid.SetMode(AUTOMATIC);
    wyawPid.SetOutputLimits(-limitPidMax, limitPidMax);

    enablePid = true;
  }
  else
  { 
    // Cascade
    cascadeRollPid.SetMode(MANUAL);
    cascadeRollPidW.SetMode(MANUAL);
    cascadePitchPid.SetMode(MANUAL);
    cascadePitchPidW.SetMode(MANUAL);
    cascadeYawPid.SetMode(MANUAL);
    cascadeYawPidW.SetMode(MANUAL);
    cascadeAltPid.SetMode(MANUAL);
    
    OutputCascPitch = 0,OutputCascPitchW = 0;
    OutputCascRoll = 0, OutputCascRollW = 0;
    OutputCascYaw = 0, OutputCascYawW = 0;
    OutputCascAlt = 0;
    
    
    // Mono
    rollPid.SetMode(MANUAL);
    pitchPid.SetMode(MANUAL);
    yawPid.SetMode(MANUAL);
    wrollPid.SetMode(MANUAL);
    wpitchPid.SetMode(MANUAL);
    wyawPid.SetMode(MANUAL);
    
    OutputWPitch = 0;
    OutputWRoll = 0;
    OutputWYaw = 0;
    OutputPitch = 0;
    OutputRoll = 0;
    OutputYaw = 0;
    OutputAltitude = 0;
    
    enablePid = false;
  } 
}
