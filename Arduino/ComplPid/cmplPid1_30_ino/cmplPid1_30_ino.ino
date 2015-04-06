
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

boolean processing = false;
boolean printMotorsVals = true;
boolean printPIDVals = true;
boolean printSerialInfo = true;
boolean printSerial = false;

byte modeS;

/**
 * VTOL settings
 */
// Take Off settings
int rampTill = 1300;
int motorRampDelayFast = 15;
int motorRampDelayMedium = 50;
int motorRampDelaySlow = 100;
int motorRampDelayVerySlow = 200;

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

// Motor constant
int thresholdUp=255, thresholdDown=1;
int maxTest = 1300, minTest = 700;
/** 
 ** Control
 **/
float Kmy = 1, Kw = 1;
//float OutputRoll = 0, OutputPitch = 0, OutputYaw = 0, OutputAlt = 0;
/**
 * Pid Controller 
 */
boolean autoEnablePid = true;
boolean enablePid = false;
boolean enableRollPid = true;
boolean enablePitchPid = false;
boolean enableYawPid = false;
boolean enableWRollPid = true;
boolean enableWPitchPid = true;
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
float consKpWRoll=0.1500, consKiWRoll=0.14, consKdWRoll=0.00025;
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

const float RESOLUTION=800; //0.8 v/g -> resolucion de 1.5g -> 800mV/g
const float VOLTAGE=3.3;  //voltage al que está conectado el acelerómetro

const float ZOUT_1G = 850;   // mv Voltage en Zout a 1G

const int NADJ  = 50;        // Número de lecturas para calcular el error

// Entradas analógicas donde van los sensores
const int xaxis = 0;
const int yaxis = 1;
const int zaxis = 2;

float XError,YError,ZError;

// Acc Timers
unsigned long accTimer;
unsigned long lastAccTimer;
unsigned long timeToRead = 0;
unsigned long lastTimeToRead = 0;
unsigned long servoTime = 0;

// delta T control the routine frequency
float deltaT = 1;
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
// Bluetooth
int BaudRateSerial = 115200;
int BluRateSerial = 115200; // Slow down in case
// Gps
int BaudRateGps = 4800;

uint8_t pinRx = 12 , pinTx = 13; // the pin on Arduino
SoftwareSerial blu(pinRx, pinTx);
byte loBytew1, hiBytew1,loBytew2, hiBytew2;
int loWord,hiWord;

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

int inputBuffSize = 30;
int outputBuffSize = 30;
byte bufferBytes[30];

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
  //blu.begin(115200); // 9600 bps
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

  if (!processing)
  {
    Serial.println("Starting up L3G4200D");
    //Serial.println("Setting up timer3");
  }

  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  delay(1500); //wait for the sensor to be ready   

  biasCalcTime = micros();
  calcBias();
  biasCalcTime = micros() - biasCalcTime;
  if (!processing)
  {
    Serial.print("Readings: [us] ");
    Serial.print(samplingTime);
    Serial.print("      Bias est: [us] ");
    Serial.print(biasCalcTime);
    Serial.print("      Samples: ");
    Serial.println(contSamples);
  }
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
  OCR3A=77; //16*10^6/(200Hz*1024)-1 = 77 -> 200 Hz 
  //OCR3A=193; //16*10^6/(80Hz*1024)-1 = 193 -> 80 Hz 
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
  serialRoutine();
  delay(20);
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
    if(!processing)
    {
      Serial.println();
      Serial.print("Landing protocol started...");
      Serial.print(throttle);
      Serial.print(" ");
    }
    for (int j=throttle; j>1000 ;j--)
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
  int motorA, motorB, motorC, motorD;

  // compute motor inputs
  motorA = thr + altpid + rollpid - yawpid;
  motorB = thr + altpid + pitchpid + yawpid;
  motorC = thr + altpid - rollpid - yawpid;
  motorD = thr + altpid - pitchpid + yawpid; 

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
}

void initialize()
{
  if (!initialized)
  {
    if (!processing)
      Serial.println("Initializing");
    initializing = true;
    resetMotors();
    delay(500);
    for (int j=700; j<rampTill; j++)
    {

      motorSpeedPID(j, OutputPitch, OutputRoll, OutputYaw, OutputAltitude);
      //motorSpeed(j);
      //if (!processing)
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

void serialRoutine()
{  
  if (blu.available())
  {
   if (printSerialInfo)
   {
      Serial.print("- Received Tot: ");
      Serial.println(blu.available());
   } 
   /*
   if (blu.available() >= inputBuffSize)
   {  
      for (int j=0;j<=inputBuffSize && printSerialInfo;j++)
      {
        bufferBytes[j]=blu.read();
        Serial.print("Received: ");
        Serial.print(bufferBytes[j]);
        Serial.println();
      }
      if (bufferBytes[0] == 3 && bufferBytes[1]==1)
      {        
        // Message has been correctly sent by Android and delivered to the Arduino 
        // Decoding Header

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
        
        // Decoding Command
         
        int type = bufferBytes[13];
        if (printSerialInfo)
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
      }
    }
    else
    {
      if (printSerialInfo)
      {
        Serial.print(" Serial.b ");
      } 
    }
    */    
    char toSend = (char) blu.read();
    Serial.print(toSend);    
  }
  
  if (Serial.available())
  {
    modeS = Serial.read(); 
    //Serial.println("ok");
    //blu.println(toSend);
    
    if (modeS == 'a')
    {
      //Serial.print('b');      
      initialize();
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
      if (!processing)
        Serial.println("Landing");
      land();
    }
    else if (modeS == 'm')
    {
      printMotorsVals = !printMotorsVals; 
    }
    else if (modeS == 'v')
    {
      printPIDVals = !printPIDVals; 
    }
  }

  timerSec = micros()-secRoutine;
  lastTimeToRead = micros();

  if (timerSec >= 1000000)
  {
    secRoutine = micros();
    if (!processing)
    {
      //Serial.print(cont);
      Serial.print("Samples rate: [sample/sec] ");
      Serial.print(contSamples);
      Serial.print("    ControlInput: ");
      Serial.println(countCtrlAction);
      Serial.print("    timeservo: ");
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
      if (processing && printSerial)
      {
        printSerialAngle();
      }
      //control();  
      controlW();
      motorSpeedPID(throttle, OutputWPitch, OutputWRoll, OutputWYaw, OutputAltitude);

      //servoTime = micros();
      //servoTime = micros() - servoTime;
      //printAcc();
      //printOmega();
      //printT();
      countCtrlAction++;
    }
  }
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
  if (filterGyro)
  {
    Serial.print("       Wx:");
    Serial.print(wF[0]);

    Serial.print("       Wy:");
    Serial.print(wF[1]);

    Serial.print("       Wz:");
    Serial.print(wF[2]);
  }

  Serial.print("      wx:");
  Serial.print(x);

  Serial.print("       wy:");
  Serial.print(y);

  Serial.print("       Wz:");
  Serial.println(z);
}

void printAcc()
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
  Serial.println(bearing1);
}

void removeBias()
{
  x = (x - bx)*scale2000/1000;
  y = (y - by)*scale2000/1000;
  z = (z - bz)*scale2000/1000;

  /*
 Serial.println();
   Serial.print(x-bx);
   Serial.print("  ");
   Serial.print(y-by);
   Serial.print("  ");
   Serial.print(z-bz);
   Serial.println();
   */
} 

void calcBias()
{
  if (!processing)
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

  if (!processing)
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

      if (printPIDVals)
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
     */
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

void controlW()
{
  if (enablePid)
  {
    // Roll W PID
    if (enableWRollPid)
    {
      InputWRoll = y;
      errorWRoll = abs(SetpointWRoll - y); 

      wRollPID.SetTunings(Kw*consKpWRoll, Kw*consKiWRoll, Kw*consKdWRoll);

      wRollPID.Compute();

      if (printPIDVals)
      {
        Serial.println();
        //        Serial.print("INPUT ANGULAR PID ROLL ");
        //        Serial.print(InputWRoll);
        Serial.print("     ErrorWWWRoll:  ");
        Serial.print(errorWRoll);
        Serial.print("     Roll WWW PID:  ");
        Serial.print(OutputWRoll);
        Serial.println();
      }
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

      if (printPIDVals)
      { 
        Serial.print(" Angular error:  ");
        Serial.print(errorWPitch);
        Serial.print(" Angular Pid Action:");
        Serial.print(OutputWPitch);
        Serial.println();
      }
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
     
     if (printPIDVals)
     {
     Serial.print(" ErrorWYaw: ");
     Serial.print(errorWYaw);
     Serial.print(" Yaw  WW pid: ");
     Serial.print(OutputWYaw);
     Serial.println();
     }     
     }
     else
     {
     OutputYaw=0;
     }
     */
    if (printPIDVals)
    {
      Serial.println();      
    }
  }
  else
  {
    OutputWRoll = 0;
    OutputWPitch = 0;    
    OutputWYaw = 0;
  }
}

