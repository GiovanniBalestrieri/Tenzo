/*****************************************************************
 *              Arduino & L3G4200D gyro & KALMAN & PID           *
 *                  running I2C mode, MMA7260Q Accelerometer     *
 *                   Gps & XBEE & MATLAB communication           *
 *                        by Balestrieri Giovanni                *
 *                          AKA UserK, 2013                      *
 *****************************************************************/

/**
 *  Features:
 *  
 *  - Bias substraction
 *  - Std filter for measurements
 *  - XBee connectivity
 *  - Matlab-Ready
 *  - Kalman filter
 *  - Complementary Filter
 *  - Motors control
 *  - FreeIMU Ready
 **/

#include <Servo.h>
#include <Wire.h>
#include <CMPS10.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include "Kalman.h"
#include "FreeIMU.h"
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#if ARDUINO >= 100
#include <SoftwareSerial.h>
#endif
#include <PID_v1.h>

/**
 * Print sensor's value
 */
boolean printOrientationFreeIMU = true; 
boolean printRawAcc = false; 
boolean printAcc = true; 
boolean printRawGyro= false; 
boolean printGyro = true; 
boolean printRawCompass= true; 
boolean printCompass = false; 
boolean printRawKalman= true; 
boolean printKalman = false; 
boolean printPIDVals = false;


/**
 * Pid Controller 
 */
boolean enablePid = true;
boolean enableRollPid = true;
boolean enablePitchPid =true;
boolean enableYawPid = false;
boolean enableAltitudePid = false;
// Define IO and setpoint for control
double SetpointRoll, InputRoll, errorRoll, OutputRoll;
double SetpointPitch, InputPitch, errorPitch, OutputPitch;
double SetpointYaw, InputYaw, errorYaw, OutputYaw;
double SetpointAltitude, InputAltitude, errorAltitude, OutputAltitude;

// Define the aggressive and conservative Tuning Parameters
// Roll
double aggKpRoll=0.5, aggKiRoll=0, aggKdRoll=0.3;
double consKpRoll=0.6, consKiRoll=0, consKdRoll=1.1;

// Pitch
double aggKpPitch=0.3, aggKiPitch=0, aggKdPitch=0.1;
double consKpPitch=0.3, consKiPitch=0, consKdPitch=0.2;

// Yaw
double aggKpYaw=4, aggKiYaw=0.2, aggKdYaw=1;
double consKpYaw=0.5, consKiYaw=0, consKdYaw=0.25;

// Altitude  ->> *** Add it, judst created
double aggKpAltitude=4, aggKiAltitude=0.2, aggKdAltitude=1;
double consKpAltitude=0.5, consKiAltitude=0, consKdAltitude=0.25;

//Specify the links and initial tuning parameters
PID myRollPID(&InputRoll, &OutputRoll, &SetpointRoll, consKpRoll, consKiRoll, consKdRoll, DIRECT);
PID myPitchPID(&InputPitch, &OutputPitch, &SetpointPitch, consKpPitch, consKiPitch, consKdPitch, DIRECT);
PID myYawPID(&InputYaw, &OutputYaw, &SetpointYaw, consKpYaw, consKiYaw, consKdYaw, DIRECT);
PID myAltitudePID(&InputAltitude, &OutputAltitude, &SetpointAltitude, consKpAltitude, consKiAltitude, consKdAltitude, DIRECT);

// initialize pid outputs
int rollPID = 0;
int pitchPID = 0;
int yawPID = 0;

// initial conditions
int w0X = 90;
int w0Y = 90;

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):

//#if ARDUINO >= 100
SoftwareSerial mySerial(10, 11);
//#else
//NewSoftSerial mySerial(10, 11);
//#endif
Adafruit_GPS GPS(&mySerial);

// Create compass object
CMPS10 my_compass;

/**
 * FreeImu library variables
 */
boolean enableIMUfiltering = false;
boolean filterIMUData = true;
// store yaw pitch roll
float ypr[3];
float iq0, iq1, iq2, iq3;
float exInt, eyInt, ezInt;  // scaled integral error
volatile float twoKp;      // 2 * proportional gain (Kp)
volatile float twoKi;      // 2 * integral gain (Ki)
volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx,  integralFBy, integralFBz;
unsigned long lastUpdate, now; // sample period expressed in milliseconds
float sampleFreq;

//FreeIMU imu;
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;

// angle estimated from gyro
double gyroXAngle = 0;
double gyroYAngle = 0;

// angle estimated from kalman
double xAngle;
double yAngle;
double zAngle;

//Offset Kalman
int kalmanXOffset = -0;
int kalmanYOffset = 0;
int kalmanZOffset = 0;

double compAngleX = 0;
double compAngleY = 0;

float complementaryConstant = 0.03;
// Xbee 
uint8_t pinRx = 12 , pinTx = 13; // the pin on Arduino
long BaudRate = 9600;
char GotChar, getData;

// Xbee SoftwareSerial initialization
SoftwareSerial xbee(pinRx, pinTx); // RX, TX

// If using hardware serial (e.g. Arduino Mega), comment
// out the above six lines and enable this line instead:
//Adafruit_GPS GPS(&Serial1);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

/*
 * Boolean variables initialization
 */

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;

//Initialize condition, true if 'i' sent via serial/xbee
boolean initializing = false;
boolean initialized = false;

// Matlab plot
boolean plotting = false;
boolean matlab = false;
boolean pid = false;
boolean manual = false;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int rampTill = 50;
int motorRampDelay = 200;

void useInterrupt(boolean); 

//I2C address of the L3G4200D
#define gyroAddress 105 

// Gyro registers definition
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

#define num 3

#define codeVersion 1.85

/*
 * Filter parameters
 * 
 */
//
//// Gyro filter constant 60 HZ
//float alphaW = 0.02; // Exp(-2PI*Fcutoff/SamplingTime) 

// Gyro filter constant 30 HZ
float alphaW = 0.9; // Exp(-2PI*Fcutoff/SamplingTime)
//
//// Gyro filter constant 25 HZ
//float alphaW = 0.20; // Exp(-2PI*Fcutoff/SamplingTime)
//
//// Accelerometer filter constant 60 HZ
//float alphaA_xy = 0.34; // Sampling freq 350HZ
//float alphaA_z  = 0.08; // Sampling freq 150HZ

// Accelerometer filter constant 30  Hz
//float alphaA_xy = 0.28;
//float alphaA_z  = 0.58;

// Accelerometer filter constant good
float alphaA_xy = 0.1;  //use 'a' to increse and 'z' to decrease
float alphaA_z  = 0.10;  //use 'h' to increase 'n' to decrease

//// Accelerometer filter constant 25 HZ
//float alphaA_xy = 0.35;
//float alphaA_z  = 0.638;

// Euler Angles filter costant
float alphaAK = 0.3;
// Magnetometer filter constant
#define alphaAP 0.2

// threshold gyro 
#define gyrothresholdLow 800
#define gyrothresholdMid 1500

#define gyrothresholdHigh 4000




// Accelerometer parameter definition
#define modeG 0.800 // [mV/G]
#define midV 336*0.0049 // [V]
#define gX0 362
#define gY0 363
#define gZ0 364

const float ZOUT_1G = 850;
const int NADJ  = 50;

float XError,YError,ZError;
float xd,yd,zd,z,x,y;

float AccelAdjust(int axis)
{
 float acc = 0;

 for (int j=0;j<NADJ;j++)
 {
   float lettura=analogRead(axis);
   acc = acc + ((lettura*5000)/1023.0);
   delay(11); 
 }
 return acc/NADJ;
}

//  Create gyro object              
int wx;
int wxCandidate;
int wx_past = 0;
int wy;
int wyCandidate;
int wy_past = 0;
int wz; 
int wzCandidate;
int wz_past = 0;
float Wfilter[3];
float accfilter[3];
float angK[3];
float angPosFilter[3];

// W filter count
int wCont;
long wxT;
long wyT;
long wzT; 

// Defining Gyro bias
int biasWx = 10;
int biasWy = 0;//-12;
int biasWz = 1;

// Filtering options
int filterAng = 0;

// Defining pin layout
int accPinX = A0;
int accPinY = A1;
int accPinZ = A2;
// Filter omega param
float gxF=0;
float gyF=0;
float gzF=0;

// Filter acc params
float xF=0;
float yF=0;
float zF=0;

// Filter compass params
float APxF=0;
float APyF=0;
float APzF=0;

// Filter Kalman angles
float AKxF=0;
float AKyF=0;
float AKzF=0;

float accX=0;
float accY=0;
float accZ=0;

double angleXAcc;
double angleYAcc;
double angleZAcc;

// Pressure Params
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
float seaLevelPressure = 1006;
int pitchK;
int rollK;
int yaw;
int pitch1;
int roll1;
float bearing1;

// Motor constant
int thresholdUp=255, thresholdDown=1;
// Motor speed;
int throttle=0;
int omega=0;

unsigned long timerMu;

/** 
*  Xbee - Matlab pid values communication 
*  Expected command: 
*  X,[0-9],[0-9],[0-9],[1.2f],X
**/

byte byteRead;
double num1, num2;
double complNum,answer,counter;
int numOfDec,optCount=0,letterCount=0;
boolean mySwitch=false;
boolean cmplx = false;
int opt1,opt2,opt3;
char* options1[4] = {"Roll","pitch","Yaw","Altitude"};
char* options2[2]  = {"Conservative","Aggressive"};
char* options3[3]  = {"proportional","Derivative","Integral"};

void setup()
{
  Serial.begin(9600);
  // Initializes I2C
  Wire.begin();    
  Serial.print(" CPanel ");
  Serial.print(codeVersion);

  Serial.println("Pressure Sensor Test");
  Serial.println("");
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  //  Reset W counters
  wxT=0,wyT=0,wzT=0;  
  wCont=0;

  //turn the PID on
  // Roll
  
//  myRollPID.SetMode(AUTOMATIC);
//  //tell the PID to range between 0 and the full throttle
//  SetpointRoll = 0;
//  myRollPID.SetOutputLimits(-500, 500);
//  
//  // Pitch
//  myPitchPID.SetMode(AUTOMATIC);
//  SetpointPitch = 0;
//  //tell the PID to range between 0 and the full throttle
//  myPitchPID.SetOutputLimits(-500, 500);
//  
//  // Yaw
//  myYawPID.SetMode(AUTOMATIC);
//  SetpointYaw=0;
//  //tell the PID to range between 0 and the full throttle
//  myYawPID.SetOutputLimits(-500, 500);

  GPS.begin(4800);
  Serial.println(" Testing Gps...");
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data for high update rates!
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // uncomment this line to turn on all the available data - for 9600 baud you'll want 1 Hz rate
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  // Set the update rate:
  // 1 Hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC or RMCGGA only (see above)
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  // Ask for firmware version
  Serial.println(PMTK_Q_RELEASE);

  Serial.print(" Accelerometer Filtering amplitude: X,Y Axis:");
  Serial.println((float) alphaA_xy);  
  Serial.print(" Z Axis: ");
  Serial.println((float) alphaA_z);
  Serial.print(" Gyroscope Filtering amplitude: ");
  Serial.println((float) alphaW);
  
  
  XError =  AccelAdjust(0);
  YError =  AccelAdjust(1);
  ZError =  AccelAdjust(2);
  ZError = ZError - ZOUT_1G;
  
  Serial.println("Ajustado acelerometro eje X");
  Serial.print("Error de X= ");
  Serial.println(XError);

  Serial.println("Ajustado acelerometro eje Y");
  Serial.print("Error de Y= ");
  Serial.println(YError);

  Serial.println("Ajustado acelerometro eje Z");
  Serial.print("Error de Z= ");
  Serial.println(ZError);
  
  setupL3G4200D(250); // Configure L3G4200  - 250, 500 or 2000 deg/sec

  // Initialize xbee serial communication
  xbee.begin( BaudRate );
  //xbee.println("Setup Completed!");

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

  //kalmanX.setRmeasure(0.003); // Measurement noise variance

  kalmanX.setRmeasure(0.5); // Measurement noise variance  
  kalmanY.setRmeasure(0.5); // Measurement noise variance
  kalmanZ.setRmeasure(0.5); // Measurement noise variance


  timerMu = micros();

  // Motors initialization
  servo1.attach(3);  
  servo2.attach(5);    
  servo3.attach(6);   
  servo4.attach(9);

  servo1.write(0);  
  servo2.write(0); 
  servo3.write(0); 
  servo4.write(0);

  getBmpValues();
  // Xbee communication variables
  num1=0;
  num2=0;
  complNum=0;
  counter=1;
  numOfDec=0;  
  
  //wait for the sensors to be ready 
  delay(2000); 
}

unsigned long timer = millis();
void loop()
{
  getAccValues(); 
  getCompassValues(); 
  getGyroValues(); 
  estimateAngle();
  gpsRoutine();
  xbeeRoutine();
  detOrientation();
  //if (initialized)
  control();
  sendDataSensors(matlab);
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
      if(errorRoll<5)
      {  //we're close to setpoint, use conservative tuning parameters
        myRollPID.SetTunings(consKpRoll, consKiRoll, consKdRoll);
      }
      else
      {
         //we're far from setpoint, use aggressive tuning parameters
         myRollPID.SetTunings(aggKpRoll, aggKiRoll, aggKdRoll);
      }
    
      myRollPID.Compute(); // Computes outputRoll
      Serial.println("");
      Serial.print("Roll PID: ");
      Serial.print(OutputRoll);
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
  //    Serial.println();
  //        Serial.println();
  //        Serial.print(errorPitch);
  //            Serial.println();
      if(errorPitch<5)
      {  //we're close to setpoint, use conservative tuning parameters
        myPitchPID.SetTunings(consKpPitch, consKiPitch, consKdPitch);
      }
      else
      {
         //we're far from setpoint, use aggressive tuning parameters
         myPitchPID.SetTunings(aggKpRoll, aggKiPitch, aggKdPitch);
      }
    
      myPitchPID.Compute(); // Computes outputPitch
      Serial.print("  Pitch: ");
      Serial.print(OutputPitch);
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
       errorYaw = abs(SetpointYaw - bearing1); //distance away from setpoint
     }
      
      if(errorYaw<5)
      {  //we're close to setpoint, use conservative tuning parameters
        myYawPID.SetTunings(consKpYaw, consKiYaw, consKdYaw);
      }
      else
      {
         //we're far from setpoint, use aggressive tuning parameters
         myYawPID.SetTunings(aggKpYaw, aggKiYaw, aggKdYaw);
      }
    
      myYawPID.Compute(); // Resturns outputYaw
      Serial.print(" Yawpid: ");
      Serial.print(OutputYaw);
      Serial.println("");
      Serial.println("");
    }
    else
    {
      OutputYaw=0;
    }
  }
}


// FreeIMU orientation - TODO ** More work needed **
void detOrientation()
{
  if (enableIMUfiltering)
  {
   getYawPitchRoll(ypr); 
   if (printOrientationFreeIMU)
   {
     Serial.println();
     Serial.print("Yaw: ");
     Serial.print(ypr[0]);
     Serial.print(" Pitch: ");
     Serial.print(ypr[1]);
     Serial.print(" Roll: ");
     Serial.print(ypr[2]);
     Serial.println("");
   }
  }
}

void getCompassValues()
{
   bearing1 =  my_compass.bearing();
   pitch1 = my_compass.pitch();
   roll1 = my_compass.roll();
   
   /*  Create float array of angular positions
    *  Roll  --  X
    *  Pitch --  Y
    *  Yaw   --  Z
    */
   angPosFilter[0] = roll1;
   angPosFilter[1] = pitch1;
   angPosFilter[2] = bearing1;
   //  Filter data  
   if (filterAng == 1)
   {
     angPosFilterExpMA(angPosFilter); 
     displayValues(angPosFilter[2],angPosFilter[1],angPosFilter[0]);
   }
   else if (filterAng == 0)
   {
     displayValues(bearing1,pitch1,roll1);
   }
   //delay(100); 
}

void xbeeRoutine()
{
  if (Serial.available()) 
  {
    GotChar = Serial.read();

    if(GotChar == 'P')
    {  	 
      plotting = true;
      Serial.println(" Sending plot data from now on.");
    } 
    if(GotChar == 'L')
    {  	
      plotting = false; 
      Serial.println(" Stop sentind data to Matlab.");
    } 

    if(GotChar == 'i')
    {  	
      initialize();
    } 
    else if(GotChar == 'M')
    {  	
      matlab = true;
    } 
    else if(GotChar == 'N')
    {  	
      matlab = false;
    }
    else if(GotChar == 'r')
    {  	
      resetMotors();
    } 
    else if(GotChar == 'z')
    { 
      if (alphaA_xy>0.01)
      {  	
        alphaA_xy = alphaA_xy - 0.01;
        Serial.println();
        Serial.print(alphaA_xy);
        Serial.println();
      }
    } 
    else if(GotChar == 'a')
    { 
      if (alphaA_xy<1)
      { 	
        alphaA_xy = alphaA_xy + 0.01;
        Serial.println();
        Serial.print(alphaA_xy);
        Serial.println();
      }
    } 
    else if(GotChar == 'o')
    { 
      if (alphaW<1)
      {  	
        alphaW = alphaW + 0.01;
        Serial.println();
        Serial.print(alphaW);
        Serial.println();
      }
    } 
    else if(GotChar == 'l')
    { 
      if (alphaW>0.01)
      {  	
        alphaW = alphaW - 0.01;
        Serial.println();
        Serial.print(alphaW);
        Serial.println();
      }
    }
    else if(GotChar == 'h')
    { 
      if (alphaA_z<1)
      {  	
        alphaA_z = alphaA_z + 0.01;
        Serial.println();
        Serial.print(alphaA_z);
        Serial.println();
      }
    } 
    else if(GotChar == 'n')
    { 
      if (alphaA_z>0.01)
      {  	
        alphaA_z = alphaA_z - 0.01;
        Serial.println();
        Serial.print(alphaA_z);
        Serial.println();
      }
    }
    else if(GotChar == 'j')
    { 
      if (alphaAK<1)
      {  	
        alphaAK = alphaAK + 0.01;
        Serial.println();
        Serial.print(alphaAK);
        Serial.println();
      }
    } 
    else if(GotChar == 'm')
    { 
      if (alphaAK>0.01)
      {  	
        alphaAK = alphaAK - 0.01;
        Serial.println();
        Serial.print(alphaAK);
        Serial.println();
      }
    }
    // Complementary filter Constant
    else if(GotChar == 'c')
    { 
      if (complementaryConstant>0.01)
      {  	
        complementaryConstant = complementaryConstant - 0.01;
        Serial.println();
        Serial.print(complementaryConstant);
        Serial.println();
      }
    }
    else if(GotChar == 'v')
    { 
      if (complementaryConstant<1)
      {  	
        complementaryConstant = complementaryConstant + 0.01;
        Serial.println();
        Serial.print(complementaryConstant);
        Serial.println();
      }
    }  
    if (GotChar == 'w')
    {
      if (throttle<thresholdUp)
      {
        throttle = throttle + 5;
      }
    }
    else if (GotChar == 's')
    {
      if (throttle>thresholdDown)
      {
        throttle = throttle - 5;
      }
    }

    if (GotChar == 'e')
    {
      if (throttle<thresholdUp)
      {
        throttle = throttle + 1;
      }
    }
    else if (GotChar == 'd')
    {
      if (throttle>thresholdDown)
      {
        throttle = throttle - 1;
      }
    }

    if (GotChar== '1')
    {
      testMotor(1);
    }
    else if (GotChar== '2')
    {
      testMotor(2);
    }
    else if (GotChar== '3')
    {
      testMotor(3);
    }
    else if (GotChar== '4')
    {
      testMotor(4);
    }
  }

  if(xbee.available())
  {  
    //is there anything to read
    getData = xbee.read();  //if yes, read it  

    Serial.println(getData);
    // Establishing Matlab Communication
    if(getData == 'T')
    {  	 
      xbee.print('K');
      Serial.println(" Sending K to Matlab...");
    } 
    if(getData == 'Y')
    {  	 
      Serial.println(" Matlab communication established. ");
    } 
    if(getData == 'N')
    {  	 
      Serial.println(" [Warning] Matlab communication problem. ");
    } 

    // Plotting command evaluation
    if(getData == 'P')
    {  	 
      plotting = true;
      Serial.println(" Sending plot data from now on.");
    } 
    if(getData == 'L')
    {  	
      plotting = false; 
      Serial.println(" Stop sentind data to Matlab.");
    }
    if (getData == 'M')
    {    
      matlab=true;
    }
    if (getData == 'S')
    {    
      matlab=false;
    }
    // Pid remote activation
    if (getData == 'p')
    {
      pid = true;
    }
    // Manual motor speed
    if (getData == 'w')
    {
      Serial.println('W');
      if (throttle<thresholdUp)
      {
        throttle = throttle + 5;
      }
    }
    else if (getData == 's')
    {
      Serial.println('S');
      if (throttle>thresholdDown)
      {
        throttle = throttle - 5;
      }
    }
    else if (getData == 'E')
    {
      if (throttle<thresholdUp)
      {
        throttle = throttle + 1;
      }
    }
    else if (getData == 'D')
    {
      if (throttle>thresholdDown)
      {
        throttle = throttle - 1;
      }
    }
    if (getData == 'X')
    {
       if (!cmplx)
       {
         // begin of the string
         cmplx =true;
       }
       else
       {
         // end of the string - reset values
         cmplx=false;
         optCount=0;
         /* Create the double from num1 and num2 */
         complNum=num1+(num2/(counter));
         /* Reset the variables for the next round */
         Serial.print("     opt1: ");
         Serial.print(options1[opt1]); 
         
         Serial.print("     opt2: ");
         Serial.print(options2[opt2]); 
                   
         Serial.print("     opt3: ");
         Serial.print(options3[opt3]); 
         
         Serial.print("    Value: ");
         Serial.print(complNum);
                 
         // Roll 
         if (opt1==0 && opt2==0 && opt3 ==0)
            consKpRoll = complNum;
         else if (opt1==0 && opt2==0 && opt3 ==1)
            consKdRoll = complNum;
         else if (opt1==0 && opt2==0 && opt3 ==2)
            consKiRoll = complNum;
        else if (opt1==0 && opt2==1 && opt3 ==0)
            aggKpRoll = complNum;
        else if (opt1==0 && opt2==1 && opt3 ==1)
            aggKdRoll = complNum;
        else if (opt1==0 && opt2==1 && opt3 ==2)
            aggKiRoll = complNum;
        // Pitch
        else if (opt1==1 && opt2==0 && opt3 ==0)
            consKpPitch = complNum;
        else if (opt1==1 && opt2==0 && opt3 ==1)
            consKdPitch = complNum;
        else if (opt1==1 && opt2==0 && opt3 ==2)
            consKiPitch = complNum;
        else if (opt1==1 && opt2==1 && opt3 ==0)
            aggKpPitch = complNum;
        else if (opt1==1 && opt2==1 && opt3 ==1)
            aggKdPitch = complNum;
        else if (opt1==1 && opt2==1 && opt3 ==2)
            aggKiPitch = complNum;  
      
        // Yaw
        else if (opt1==2 && opt2==0 && opt3 ==0)
            consKpYaw = complNum;
        else if (opt1==2 && opt2==0 && opt3 ==1)
            consKdYaw = complNum;
        else if (opt1==2 && opt2==0 && opt3 ==2)
            consKiYaw = complNum;
        else if (opt1==2 && opt2==1 && opt3 ==0)
            aggKpYaw = complNum;
        else if (opt1==2 && opt2==1 && opt3 ==1)
            aggKdYaw = complNum;
        else if (opt1==2 && opt2==1 && opt3 ==2)
            aggKiYaw = complNum;   
         
         // Altitude
        else if (opt1==3 && opt2==0 && opt3 ==0)
            consKpAltitude = complNum;
        else if (opt1==3 && opt2==0 && opt3 ==1)
            consKdAltitude = complNum;
        else if (opt1==3 && opt2==0 && opt3 ==2)
            consKiAltitude = complNum;
        else if (opt1==3 && opt2==1 && opt3 ==0)
            aggKpAltitude = complNum;
        else if (opt1==3 && opt2==1 && opt3 ==1)
            aggKdAltitude = complNum;
        else if (opt1==3 && opt2==1 && opt3 ==2)
            aggKiAltitude = complNum;   
         
         num1=0;
         num2=0;
         complNum=0;
         counter=1;
         mySwitch=false;
         numOfDec=0;
       }
    }
    
    if (getData==44)
    {
       // Comma
       optCount++;
       Serial.println();
       Serial.print("virgola numero: ");
       Serial.println(optCount);
       letterCount = 0;
    }
  
     //listen for numbers between 0-9
     if(getData>47 && getData<58)
     {
        //number found
        if (cmplx)
        {
          if (optCount == 1)
          {
           opt1 = getData - 48; 
           Serial.print(opt1);
          }
          else if (optCount == 2)
          {
           opt2 = getData - 48; 
           Serial.print(opt2);
          }
          else if (optCount == 3)
          {
           opt3 = getData - 48; 
           Serial.print(opt3);
          }
          if (optCount == 4)
          {
            /* If mySwitch is true, then populate the num1 variable
            otherwise populate the num2 variable*/
            if(!mySwitch)
            {
              num1=(num1*10)+(getData-48);
            }
            else
            {
              num2=(num2*10)+(getData-48);           
              /* These counters are important */
              counter=counter*10;
              numOfDec++;
            }
          }
        }
     }
  
    if (getData==46)
    {
        mySwitch=true;
    }
    

    else if(getData == 'r')
    {
      resetMotors();
    }
    else if (getData== '1')
    {
      testMotor(1);
    }
    else if (getData== '2')
    {
      testMotor(2);
    }
    else if (getData== '3')
    {
      testMotor(3);
    }
    else if (getData== '4')
    {
      testMotor(4);
    }
    else if (getData== 'i')
    {
      initialize();
    }
  }
  if (initialized||matlab)
  {
    //motorSpeed(throttle);
    motorSpeedPID(throttle,OutputPitch ,OutputRoll, OutputYaw);
    Serial.println(throttle);
  }
}

void resetMotors()
{
  throttle = 0;
  motorSpeed(0);
  // Disable Pid when motors are off
  myRollPID.SetMode(MANUAL);
  myPitchPID.SetMode(MANUAL);
  myYawPID.SetMode(MANUAL);
}

void initialize()
{
  initializing = true;
  resetMotors();
  delay(2000);
  for (int j=0; j<rampTill;j++)
  {
    motorSpeed(j);
    Serial.println(j);
    delay(motorRampDelay); 
  }
  throttle=rampTill;
  initialized = true;
  
  // Enable Pid Actions
  myRollPID.SetMode(AUTOMATIC);
  //tell the PID to range between 0 and the full throttle
  SetpointRoll = 0;
  myRollPID.SetOutputLimits(-500, 500);
  
  // Pitch
  myPitchPID.SetMode(AUTOMATIC);
  SetpointPitch = 0;
  //tell the PID to range between 0 and the full throttle
  myPitchPID.SetOutputLimits(-500, 500);
  
  // Yaw
  myYawPID.SetMode(AUTOMATIC);
  SetpointYaw=0;
  //tell the PID to range between 0 and the full throttle
  myYawPID.SetOutputLimits(-500, 500);
}

void motorSpeedPID(int thr, int rollpid, int pitchpid, int yawpid)
{
  int motorA, motorB, motorC, motorD;
   
  // compute motor inputs
  motorA = thr + rollpid - yawpid;
  motorB = thr + pitchpid + yawpid;
  motorC = thr - rollpid - yawpid;
  motorD = thr - pitchpid + yawpid; 
  if (printPIDVals)
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
    Serial.print("   ]");  
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

void motorSpeed(int x)
{    
  servo1.write(x);      
  servo2.write(x); 
  servo3.write(x); 
  servo4.write(x); 
}

void sendDataSensors(boolean device)
{
  if (device)
  {    
    Serial.println("       M    ");
    // Gathering and Sending Sensors data
    xbee.print('R');
    xbee.print(',');
    xbee.print(roll1);
    xbee.print(',');
    xbee.print('P');
    xbee.print(',');
    xbee.print(pitch1);
    xbee.print(',');
    xbee.print('Y');
    xbee.print(',');
    xbee.print(bearing1);
    // Send Kalman Filer Angular position values
    xbee.print(',');
    xbee.print('KR');
    xbee.print(',');
    xbee.print((int)yAngle);
    xbee.print(',');
    xbee.print('KP');
    xbee.print(',');
    xbee.print((int)xAngle);
    // Send the rest      
    xbee.print('W');
    xbee.print(',');
    //xbee.print(wx);
    xbee.print(Wfilter[0]);
    xbee.print(',');
    xbee.print('W');
    xbee.print(',');
    //xbee.print(wy);
    xbee.print(Wfilter[1]);
    xbee.print(',');
    xbee.print('W');
    xbee.print(',');
    //xbee.print(wz);
    xbee.print(Wfilter[2]);
    xbee.print(',');
    xbee.print('A');
    xbee.print(',');
    xbee.print(accX);
    xbee.print(',');
    xbee.print('A');
    xbee.print(',');
    xbee.print(accY);
    xbee.print(',');
    xbee.print('A');
    xbee.print(',');
    xbee.print(accZ);
    xbee.print(',');
    xbee.print('O');
    xbee.print(',');
    xbee.print(throttle);
    //Plot terminator carriage
    xbee.write(13);
  } 
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

void getAccValues()
{
  int accValX = analogRead(accPinX);
  int accValY = analogRead(accPinY);
  int accValZ = analogRead(accPinZ);


//  float aX = (float) (accValX - gX0);
//  float aY = (float) (accValY - gY0);
//  float aZ = (float) (accValZ - gZ0);


 accX = (((accValX*5000.0)/1023.0)-XError)/800;
 accY = (((accValY*5000.0)/1023.0)-YError)/800;
 accZ = (((accValZ*5000.0)/1023.0)-ZError)/800;


//  accX = aX*5/(1024.0*0.800);  
//  accY = aY*5/(1024.0*0.800);  
//  accZ = aZ*5/(1024.0*0.800);  

//  //Compute angles from acc values
//  angleXAcc = (atan2(-accX,accZ)) * RAD_TO_DEG;
//  angleYAcc = (atan2(accY,accZ)) * RAD_TO_DEG;
//  angleZAcc = (atan2(accY,accX)) * RAD_TO_DEG;

  accfilter[0] = accX;
  accfilter[1] = accY;
  accfilter[2] = accZ;
  
  accFilterExpMA(accfilter);
  
    //Compute angles from acc values
  angleXAcc = (atan2(-accfilter[0],accfilter[2])) * RAD_TO_DEG;
  angleYAcc = (atan2(accfilter[1],accfilter[2])) * RAD_TO_DEG;
  //angleZAcc = (atan2(accfilter[0],accfilter[1])) * RAD_TO_DEG;

  if (printRawAcc)
  {
    Serial.print("Acc = [");
    Serial.print(accX);
    Serial.print(",");
    Serial.print(accY);
    Serial.print(",");
    Serial.print(accZ);
    Serial.print("]    ");
  }
  if (printAcc)
  {
    Serial.print("  [axF,ayF,azF] = [");
    Serial.print(accfilter[0]);
    Serial.print(",");
    Serial.print(accfilter[1]);
    Serial.print(",");
    Serial.print(accfilter[2]);
    Serial.print("]    ");
  }
}  

void gpsRoutine()
{
  if (! usingInterrupt) 
  {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    //if (GPSECHO)
    //if (c) Serial.print(c);
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) 
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  
    timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000 /*&& GPS.fix*/) 
  { 
    timer = millis(); // reset the timer

    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); 
    Serial.print(':');
    Serial.print(GPS.minute, DEC); 
    Serial.print(':');
    Serial.print(GPS.seconds, DEC); 
    Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); 
    Serial.print('/');
    Serial.print(GPS.month, DEC); 
    Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); 
    Serial.print((int)GPS.fix);
    Serial.print(" quality: ");  
    Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); 
      Serial.print(GPS.lat);
      Serial.print(", "); 
      xbee.print(" Altitude :");
      xbee.print(GPS.latitude);
      Serial.print(GPS.longitude, 4);
      Serial.println(GPS.lon);

      xbee.print(";  Longitude: ");
      xbee.print(GPS.longitude);
      Serial.print("Speed (knots): "); 
      Serial.println(GPS.speed);
      Serial.print("Angle: "); 
      Serial.println(GPS.angle);
      Serial.print("Altitude: ");
      Serial.println(GPS.altitude);

      xbee.print("; Altitude");
      xbee.print(GPS.altitude);
      Serial.println("Satellites: "); 
      Serial.println((int)GPS.satellites);
    }
  }
}

// Low pass filter
void accFilterExpMA(float val[])
{
  //ArrayList val = new ArrayList();
  xF = (1-alphaA_xy)*xF + alphaA_xy*val[0];
  yF = (1-alphaA_xy)*yF + alphaA_xy*val[1];
  zF = (1-alphaA_z)*zF + alphaA_z*val[2];

  val[0] = xF;
  val[1] = yF;
  val[2] = zF; 
}


void omegaFilterExpMA(float val[])
{
  //ArrayList val = new ArrayList();
  gxF = (1-alphaW)*gxF + alphaW*val[0];
  gyF = (1-alphaW)*gyF + alphaW*val[1];
  gzF = (1-alphaW)*gzF + alphaW*val[2];

  val[0] = gxF - biasWx;
  val[1] = gyF - biasWy;
  val[2] = gzF - biasWz; 
}

void getGyroValues()
{
  wCont++;
  byte xMSB = readRegister(gyroAddress, 0x29);
  byte xLSB = readRegister(gyroAddress, 0x28);
  //wx = ((xMSB << 8) | xLSB);  
  wxCandidate = ((xMSB << 8) | xLSB);  
  if (abs(wxCandidate-wx_past) <= gyrothresholdHigh)
  {
    wx=wxCandidate;
  }
  wx_past  =  wx;
  gyroXAngle += wx * ((double)(micros() - timerMu) / 1000000);
  wxT = wxT + wx;


  byte yMSB = readRegister(gyroAddress, 0x2B);
  byte yLSB = readRegister(gyroAddress, 0x2A);
  wyCandidate = ((yMSB << 8) | yLSB);
  if (abs(wyCandidate-wy_past) <= gyrothresholdHigh)
  {
    wy=wyCandidate;
    wy_past  =  wy;
  }
  wy_past  =  wy;
  gyroYAngle += wy * ((double)(micros() - timerMu) / 1000000);
  wyT = wyT + wy;

  byte zMSB = readRegister(gyroAddress, 0x2D);
  byte zLSB = readRegister(gyroAddress, 0x2C);
  wzCandidate = ((zMSB << 8) | zLSB);
  if (abs(wzCandidate-wz_past) <= gyrothresholdHigh)
  {
    wz=wzCandidate;
  }
  wz_past  =  wz;
  wzT = wzT + wz;
  
  Wfilter[0] = wx;
  Wfilter[1] = wy;
  Wfilter[2] = wz;

  omegaFilterExpMA(Wfilter);
  if (printRawGyro)
  {
    Serial.print(" [wx,wy,wz]");
    Serial.print(" = [ ");
    Serial.print( (float) wx);
  
    Serial.print(" , ");
    Serial.print( (float) wy);
  
    Serial.print(" , ");
    Serial.print( (float) wz);
    Serial.print(" ]");
  }
  if (printGyro)
  {
    Serial.print(" [WxF,WyF,WzF]");
    Serial.print(" = [ ");
    Serial.print( (float) Wfilter[0]);
  
    Serial.print(" , ");
    Serial.print( (float) Wfilter[1]);
  
    Serial.print(" , ");
    Serial.print( (float) Wfilter[2]);
    Serial.print(" ]");
  }
  /*
   * To determine bias, uncomment what follows, leave
   * the board on the support and wait for the limit of 
   * the value wxT/wCont,wyT/wCont,wzT/wCont. Use it as a bias
   */

  //Serial.println();
  //Serial.println(wxT/wCont);
  //Serial.println(wyT/wCont);
  //Serial.println(wzT/wCont);

}

void estimateAngle()
{

  // compAngleX = ((1-complementaryConstant) * (compAngleX + (wx * (double)(micros() - timerMu) / 1000000))) + (complementaryConstant * angleXAcc); 
  // compAngleY = ((1-complementaryConstant) * (compAngleY + (-wy * (double)(micros() - timerMu) / 1000000))) + (complementaryConstant * angleYAcc); 
  // 
  // Serial.print("  C:( ");
  // Serial.print(compAngleX);
  // Serial.print(" ; ");
  // Serial.print(compAngleY);
  // Serial.print(")  "); 


  xAngle = kalmanX.getAngle(angleXAcc,Wfilter[0],(double)(micros() - timerMu)) + kalmanXOffset;
  yAngle = kalmanY.getAngle(angleYAcc,Wfilter[1],(double)(micros() - timerMu)) + kalmanYOffset;
  yAngle = -yAngle;
  if (filterAng == 1)
   {
      zAngle = kalmanZ.getAngle(angPosFilter[2],Wfilter[2],(double)(micros() - timerMu)) + kalmanZOffset;
   }
   else if (filterAng == 0)
   {
      zAngle = kalmanZ.getAngle(bearing1,Wfilter[2],(double)(micros() - timerMu)) + kalmanZOffset;
   }

  angK[0] = xAngle;
  angK[1] = yAngle;
  angK[2] = zAngle;
  angEstKalmanFilter(angK);
  if (printRawKalman)
  {
    Serial.print("K:( ");
    Serial.print((int)yAngle);
    Serial.print(" ; ");
    Serial.print((int)xAngle);
    Serial.print(" ; ");
    Serial.print((int)zAngle);
    Serial.print(")     ");
  }
  if (printKalman)
  {
    Serial.print("KFilt:( ");
    Serial.print((int)  angK[1]);
    Serial.print(" ; ");
    Serial.print((int) angK[0]);
    Serial.print(" ; ");
    Serial.print((int) angK[2]);
    Serial.print(")");
  }
  Serial.println();

  pitchK = angK[0];
  rollK = angK[1];
  yaw = angK[2];

  timerMu = micros();
}

void getBmpValues()
{
  /* Get a new sensor event */
  sensors_event_t event;
  bmp.getEvent(&event);
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    Serial.println("Station data: ");
    /* Display atmospheric pressue in hPa */
    Serial.println("Pressure: ");
    Serial.print(event.pressure);
    Serial.println(" hPa");
    /* Calculating altitude with reasonable accuracy requires pressure *
     * sea level pressure for your position at the moment the data is *
     * converted, as well as the ambient temperature in degress *
     * celcius. If you don't have these values, a 'generic' value of *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA *
     * in sensors.h), but this isn't ideal and will give variable *
     * results from one day to the next. *
     * *
     * You can usually find the current SLP value by looking at weather *
     * websites or from environmental information centers near any major *
     * airport. *
     * *
     * For example, for Paris, France you can check the current mean *
     * pressure and sea level at: http://bit.ly/16Au8ol */
    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    /* Then convert the atmospheric pressure, SLP and temp to altitude */
    /* Update this next line with the current SLP for better results */
    //float seaLevelPressure = 1008.6;
    //float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print("Altitude: ");
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,event.pressure,temperature));
    Serial.println(" m");
    Serial.println("");
  }
  else
  {
    Serial.println("Sensor error");
  }
}

void angEstKalmanFilter(float val[])
{
  //ArrayList val = new ArrayList();
  AKxF = (1-alphaAK)*AKxF + alphaAK*val[0];
  AKyF = (1-alphaAK)*AKyF + alphaAK*val[1];
  //AKzF = (1-alphaAK)*AKzF + alphaAK*val[2];
  
  val[0] = AKxF;
  val[1] = AKyF;
  //val[2] = AKzF; 
}

void angPosFilterExpMA(float val[])
{
  APxF = (1-alphaAP)*APxF + alphaAP*val[0];
  APyF = (1-alphaAP)*APyF + alphaAP*val[1];
  APzF = (1-alphaAP)*APzF + alphaAP*val[2];
  
  val[0] = APxF;
  val[1] = APyF;
  val[2] = APzF; 
}

void displayValues(int b, int p, int r)
{    
  if (printRawCompass)
  {
    Serial.print(" Compass: [R, P, Y] = [");
    Serial.print(r);  
    Serial.print(" , ");
    Serial.print(p);
    Serial.print(" , ");
    Serial.print(b);
    Serial.print(" ]");
  }
}

int setupL3G4200D(int scale){
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
  //writeRegister(gyroAddress, CTRL_REG5, 0b00010000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

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

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
  // writing direct to UDR0 is much much faster than Serial.print 
  // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

/**
 * FreeIMU implementation
 */
 
 void getValues(float val[])
{
  if (filterIMUData)
  {
    val[0] = Wfilter[0];
    val[1] = Wfilter[1];
    val[2] = Wfilter[2];
    val[3] = accfilter[0];
    val[4] = accfilter[1];
    val[5] = accfilter[2];
    val[6] = angPosFilter[0];
    val[7] = angPosFilter[1];
    val[8] = angPosFilter[2];
  } else
  {
    val[0] = wx;
    val[1] = wy;
    val[2] = wz;
    val[3] = accX;
    val[4] = accY;
    val[5] = accZ;
    val[6] = roll1;
    val[7] = pitch1;
    val[8] = bearing1;
  }
}

/**
 * Populates array q with a quaternion representing the IMU orientation with respect to the Earth
 * 
 * @param q the quaternion to populate
*/
void getQ(float * q) {
  float val[9];
  getValues(val);
  
  now = micros();
  sampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);

  lastUpdate = now;
  // gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec

  //AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
AHRSupdate(val[0] * M_PI/180, val[1] * M_PI/180, val[2] * M_PI/180, val[3], val[4], val[5], val[6], val[7], val[8]);

//  AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], -val[6], -val[7], val[8]);
  Serial.println();
  Serial.println();
  Serial.print(q0);
  Serial.print("  ");
  Serial.print(q1);
  Serial.print("  ");
  Serial.print(q2);
  Serial.print("  ");
  Serial.print(q3);
  Serial.println();
  Serial.println();
  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

/**
 * Returns the Euler angles in degrees defined with the Aerospace sequence.
 * See Sebastian O.H. Madwick report "An efficient orientation filter for 
 * inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
 * 
 * @param angles three floats array which will be populated by the Euler angles in degrees
*/
void getEuler(float * angles) {
  getEulerRad(angles);
  arr3_rad_to_deg(angles);
}

/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in degrees between
 * the Earth North and the IMU X axis (yaw), the Earth ground plane and the IMU X axis (pitch)
 * and the Earth ground plane and the IMU Y axis.
 * 
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 * 
 * @param ypr three floats array which will be populated by Yaw, Pitch and Roll angles in degrees
*/
void getYawPitchRoll(float * ypr) {
  getYawPitchRollRad(ypr);
  arr3_rad_to_deg(ypr);
}

/**
 * Returns the Euler angles in radians defined in the Aerospace sequence.
 * See Sebastian O.H. Madwick report "An efficient orientation filter for 
 * inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
 * 
 * @param angles three floats array which will be populated by the Euler angles in radians
*/
void getEulerRad(float * angles) {
  float q[4]; // quaternion
  getQ(q);
  angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

/**
 * Returns the yaw pitch and roll angles, respectively defined as the angles in radians between
 * the Earth North and the IMU X axis (yaw), the Earth ground plane and the IMU X axis (pitch)
 * and the Earth ground plane and the IMU Y axis.
 * 
 * @note This is not an Euler representation: the rotations aren't consecutive rotations but only
 * angles from Earth and the IMU. For Euler representation Yaw, Pitch and Roll see FreeIMU::getEuler
 * 
 * @param ypr three floats array which will be populated by Yaw, Pitch and Roll angles in radians
*/
void getYawPitchRollRad(float * ypr) {
  float q[4]; // quaternion
  float gx, gy, gz; // estimated gravity direction
  getQ(q);
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
  ypr[1] = atan(gx / sqrt(gy*gy + gz*gz));
  ypr[2] = atan(gy / sqrt(gx*gx + gz*gz));
}

/**
 * Fast inverse square root implementation
 * @see http://en.wikipedia.org/wiki/Fast_inverse_square_root
*/
float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}

/**
 * Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
 * compensation algorithms from Sebastian Madgwick's filter which eliminates the need for a reference
 * direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
 * axis only.
 * 
 * @see: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
*/
void  AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
  float qa, qb, qc;

  // Auxiliary variables to avoid repeated arithmetic
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;
  
  // Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
  if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
    float hx, hy, bx, bz;
    float halfwx, halfwy, halfwz;
    
    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);

    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;
    
    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
    
    
    // Estimated direction of magnetic field
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
    
    
    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex = (my * halfwz - mz * halfwy);
    halfey = (mz * halfwx - mx * halfwz);
    halfez = (mx * halfwy - my * halfwx);
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
    float halfvx, halfvy, halfvz;
    
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // Estimated direction of gravity
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
  
    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex += (ay * halfvz - az * halfvy);
    halfey += (az * halfvx - ax * halfvz);
    halfez += (ax * halfvy - ay * halfvx);
  }

  // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
  if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
 
  }
  
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);
  
      
    Serial.println();
    Serial.println();
    Serial.print(qa);
    Serial.print("  ");
    Serial.print(qb);
    Serial.print("   ");
    Serial.print(qc);
    Serial.print("  ");
    Serial.print(q0);
    Serial.println();
    Serial.println(); 
  
  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

/**
 * Converts a 3 elements array arr of angles expressed in radians into degrees
*/
void arr3_rad_to_deg(float * arr) {
  arr[0] *= 180/M_PI;
  arr[1] *= 180/M_PI;
  arr[2] *= 180/M_PI;
}
