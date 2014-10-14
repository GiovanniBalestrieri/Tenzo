/****************************************************************
*              Arduino & L3G4200D gyro & KALMAN                 *
*                  running I2C mode, MMA7260Q Accelerometer     *
*                   Gps & XBEE & MATLAB communication           *
*                        by Balestrieri Giovanni                *
*                          AKA UserK, 2013                      *
*****************************************************************/

/*
 *  Features:
 *  
 *  - Bias substraction
 *  - Std filter for measurements
 *  - XBee connectivity
 *  - Matlab-Ready
 *  - Kalman filter
 *  - Complementary Filter
 *  - Motors control
 */
 
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
#else
  // Older Arduino IDE requires NewSoftSerial, download from:
  // http://arduiniana.org/libraries/newsoftserial/
// #include <NewSoftSerial.h>
 // DO NOT install NewSoftSerial if using Arduino 1.0 or later!
#endif
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

Kalman kalmanX;
Kalman kalmanY;

// angle estimated from gyro
double gyroXAngle = 0;
double gyroYAngle = 0;

// angle estimated from kalman
double xAngle;
double yAngle;

//Offset Kalman
int kalmanXOffset = -10;
int kalmanYOffset = -4;

double compAngleX = 180;
double compAngleY = 180;

// Xbee 
uint8_t pinRx = 2 , pinTx = 4; // the pin on Arduino
long BaudRate = 57600;
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

int rampTill = 40;
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

#define codeVersion 1.45

/*
 * Filter parameters
 * 
 */
//
//// Gyro filter constant 60 HZ
//float alphaW = 0.02; // Exp(-2PI*Fcutoff/SamplingTime) 

// Gyro filter constant 30 HZ
float alphaW = 0.15; // Exp(-2PI*Fcutoff/SamplingTime)
//
//// Gyro filter constant 25 HZ
//float alphaW = 0.20; // Exp(-2PI*Fcutoff/SamplingTime)
//
//// Accelerometer filter constant 60 HZ
//float alphaA_xy = 0.34; // Sampling freq 350HZ
//float alphaA_z  = 0.08; // Sampling freq 150HZ

// Accelerometer filter constant 30  Hz
float alphaA_xy = 0.28;
float alphaA_z  = 0.58;
//
//// Accelerometer filter constant 25 HZ
//float alphaA_xy = 0.35;
//float alphaA_z  = 0.638;

// Magnetometer filter constant
#define alphaAP 0.7

// threshold gyro 
#define gyrothresholdLow 50
#define gyrothresholdMid 70
#define gyrothresholdHigh 100


// Accelerometer parameter definition
#define modeG 0.800 // [mV/G]
#define midV 336*0.0049 // [V]
#define gX0 362
#define gY0 363
#define gZ0 364


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

// W filter count
int wCont;
long wxT;
long wyT;
long wzT; 

// Defining Gyro bias
int biasWx = 10;
int biasWy = -12;
int biasWz = 1;

// Filtering options
int filterAng = 1;

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

float accX=0;
float accY=0;
float accZ=0;

double angleXAcc;
double angleYAcc;
double angleZAcc;

// Pressure Params
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

int pitch1;
int roll1;
float bearing1;

// Motor constant
int thresholdUp=255, thresholdDown=1;
// Motor speed;
int omega=0;

unsigned long timerMu;
  
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
  setupL3G4200D(250); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  
  // Initialize xbee serial communication
  xbee.begin( BaudRate );
  //xbee.println("Setup Completed!");
  
  // Initialise Kalman Values
  kalmanX.setAngle(0);
  kalmanY.setAngle(0);
  
  kalmanX.setQbias(4); // Process noise variance for gyro bias
  kalmanY.setQbias(4); // Process noise variance for gyro bias
  
  kalmanX.setQangle(0.5); // Process noise variance for the accelerometer
  kalmanY.setQangle(0.1); // Process noise variance for the accelerometer
  
  //kalmanX.setRmeasure(0.003); // Measurement noise variance
 
  kalmanX.setRmeasure(4); // Measurement noise variance  
  kalmanY.setRmeasure(4); // Measurement noise variance
  
  
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
  //wait for the sensors to be ready 
  delay(2000); 
}

unsigned long timer = millis();
void loop()
{
  getAccValues();
  getGyroValues();  
  estimateAngle();
  gpsRoutine();
  xbeeRoutine();
  sendDataSensors(matlab);
}

/* 
   *
   *  Test Commands /1/1/0.5 to set 0.5 as Acc filtering coeff 
   *  Test Commands /1/1/0.9 to set 0.9 as Gyro filtering coeff
   *
  
long getSerial()
{
  serialdata = 0;
  while (inbyte != '/')
  {
    inbyte = Serial.read(); 
    if (inbyte > 0 && inbyte != '/')
    {
     
      serialdata = serialdata * 10 + inbyte - '0';
    }
  }
  inbyte = 0;
  return serialdata;
}  
 */
/* 
   *
   *  When Matlab sends P, arduino starts sending sensors data through Xbee
   *  If Matlab sends L, it stops.
   * Change Filtering amplitude:
   * a -> alpfa Acc + 0.5
   * z -> alfa Acc - 0.6
   * o -> alfa Gyro + 0.5
   * l -> alfa Gyro - 0.5
   *
   */
void xbeeRoutine()
{
  /*
  getSerial();
  switch(serialdata)
  {
  case 'f':
    {
      //analog digital write
      getSerial();
      switch (serialdata)
      {
      case 'a':
        {
          //analog write
          getSerial();
          pinNumber = serialdata;
          getSerial();
          analogRate = serialdata;
          pinMode(pinNumber, OUTPUT);
          analogWrite(pinNumber, analogRate);
          pinNumber = 0;
          break;
        }
      case 'w':
        {
          //digital write
          getSerial();
          pinNumber = serialdata;
          getSerial();
          digitalState = serialdata;
          pinMode(pinNumber, OUTPUT);
          if (digitalState == 0)
          {
            digitalWrite(pinNumber, LOW);
          }
          if (digitalState == 1)
          {
            digitalWrite(pinNumber, HIGH);
          }
          pinNumber = 0;
          break;
          
        }
     }
     break; 
   }
  }
  */
  
  if (Serial.available()) 
  {
    GotChar = Serial.read();
    xbee.print(GotChar);
    
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
    } else if(GotChar == 'M')
    {  	
      matlab = true;
    } else if(GotChar == 'N')
    {  	
     matlab = false;
    }
    else if(GotChar == 'r')
    {  	
      resetMotors();
    } 
    else if(GotChar == 'z')
    { 
       if (alphaA_xy>0.05)
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
       if (alphaW>0.05)
       {  	
        alphaW = alphaW - 0.01;
        Serial.println();
        Serial.print(alphaW);
        Serial.println();
       }
    }
    
    if (GotChar == 'w')
    {
     if (omega<thresholdUp)
     {
       omega = omega + 5;
     }
    }
    else if (GotChar == 's')
    {
      if (omega>thresholdDown)
      {
        omega = omega - 5;
      }
    }
    
    if (GotChar == 'e')
    {
     if (omega<thresholdUp)
     {
       omega = omega + 1;
     }
    }
    else if (GotChar == 'd')
    {
      if (omega>thresholdDown)
      {
        omega = omega - 1;
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
     if (omega<thresholdUp)
     {
       omega = omega + 5;
     }
    }
    else if (getData == 's')
    {
      Serial.println('S');
      if (omega>thresholdDown)
      {
        omega = omega - 5;
      }
    }
    else if (getData == 'E')
    {
     if (omega<thresholdUp)
     {
       omega = omega + 1;
     }
    }
    else if (getData == 'D')
    {
      if (omega>thresholdDown)
      {
        omega = omega - 1;
      }
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
    motorSpeed(omega);
    Serial.println(omega);
  }
}

void resetMotors()
{
  omega = 0;
  motorSpeed(0);
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
  omega=rampTill;
  initialized = true;
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
    xbee.print(omega);
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
      servo1.write(80);
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
      servo2.write(80);
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
      servo3.write(80);
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
//  Serial.print("A = [");
//  Serial.print(accValX);
//  Serial.print(",");
//  Serial.print(accValY);
//  Serial.print(",");
//  Serial.print(accValZ);
//  Serial.print("]    ");
//  float aX = (float) accValX * 0.0049;
//  float aY = (float) accValY * 0.0049;
//  float aZ = (float) accValZ * 0.0049;


  float aX = (float) (accValX - gX0);
  float aY = (float) (accValY - gY0);
  float aZ = (float) (accValZ - gZ0);
  
//  float aX = (float) (accValX - g0) * 0.0049;
//  float aY = (float) (accValY - g0) * 0.0049;
//  float aZ = (float) (accValZ - g0) * 0.0049;
//  
//  accX = (aX-midV)/modeG;
//  accY = (aY-midV)/modeG;
//  accZ = (aZ-midV)/modeG;

  accX = aX*5/(1024.0*0.800);  ;
  accY = aY*5/(1024.0*0.800);  ;
  accZ = aZ*5/(1024.0*0.800);  ;

  
  //Compute angles from acc values
  angleXAcc = (atan2(-accX,accZ)) * RAD_TO_DEG;
  angleYAcc = (atan2(-accY,accZ)) * RAD_TO_DEG;
  //angleZAcc = (atan2(-accY,accX)) * RAD_TO_DEG;
  
  float accfilter[] = {accX,accY,accZ};
  accFilterExpMA(accfilter);
  
//  Serial.print("Acc = [");
//  Serial.print(accX);
//  Serial.print(",");
//  Serial.print(accY);
//  Serial.print(",");
//  Serial.print(accZ);
//  Serial.print("]    ");
  
  Serial.print("  [axF,ayF,azF] = [");
  Serial.print(accfilter[0]);
  Serial.print(",");
  Serial.print(accfilter[1]);
  Serial.print(",");
  Serial.print(accfilter[2]);
  Serial.print("]    ");
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
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: ");  Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      xbee.print(" Altitude :");
      xbee.print(GPS.latitude);
      Serial.print(GPS.longitude, 4);
      Serial.println(GPS.lon);
      
      xbee.print(";  Longitude: ");
      xbee.print(GPS.longitude);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: ");
      Serial.println(GPS.altitude);
      
      xbee.print("; Altitude");
      xbee.print(GPS.altitude);
      Serial.println("Satellites: "); Serial.println((int)GPS.satellites);
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
  if (abs(wxCandidate-wx_past) <= gyrothresholdLow)
  {
    wx=wxCandidate;
    wx_past  =  wx;
  }
  gyroXAngle += wx * ((double)(micros() - timerMu) / 1000000);
  wxT = wxT + wx;
  
  
  byte yMSB = readRegister(gyroAddress, 0x2B);
  byte yLSB = readRegister(gyroAddress, 0x2A);
  wyCandidate = ((yMSB << 8) | yLSB);
  if (abs(wyCandidate-wy_past) <= gyrothresholdLow)
  {
    wy=wyCandidate;
    wy_past  =  wy;
  }
  gyroYAngle += wy * ((double)(micros() - timerMu) / 1000000);
  wyT = wyT + wy;
  
  byte zMSB = readRegister(gyroAddress, 0x2D);
  byte zLSB = readRegister(gyroAddress, 0x2C);
  wzCandidate = ((zMSB << 8) | zLSB);
  if (abs(wzCandidate-wz_past) <= gyrothresholdLow)
  {
    wz=wzCandidate;
    wz_past  =  wz;
  }
  wzT = wzT + wz;
  /*
  Serial.print("   Gyroscope [Wx,Wy,Wz]");
  Serial.print(" = [ ");
  Serial.print(wx);

  Serial.print(" , ");
  Serial.print(wy);

  Serial.print(" , ");
  Serial.print(wz);
  Serial.print(" ]");
  */
  
  Wfilter[0] = wx;
  Wfilter[1] = wy;
  Wfilter[2] = wz;
  
  omegaFilterExpMA(Wfilter);
  
  Serial.print(" [WxF,WyF,WzF]");
  Serial.print(" = [ ");
  Serial.print( (float) Wfilter[0]);

  Serial.print(" , ");
  Serial.print( (float) Wfilter[1]);

  Serial.print(" , ");
  Serial.print( (float) Wfilter[2]);
  Serial.print(" ]");
 
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
 /* 
 compAngleX = (0.93 * (compAngleX + (wx * (double)(micros() - timerMu) / 1000000))) + (0.07 * angleXAcc); 
 compAngleY = (0.93 * (compAngleY + (-wy * (double)(micros() - timerMu) / 1000000))) + (0.07 * angleYAcc); 
 
 Serial.println();
 Serial.print("  C:( ");
 Serial.print(compAngleX);
 Serial.print(" ; ");
 Serial.print(compAngleY);
 Serial.print(")  "); 
 */
 
 xAngle = kalmanX.getAngle(angleXAcc,Wfilter[0],(double)(micros() - timerMu))+kalmanXOffset;
 yAngle = kalmanY.getAngle(angleYAcc,Wfilter[1],(double)(micros() - timerMu))+kalmanYOffset;
 yAngle = -yAngle;
 
 Serial.print("K:( ");
 Serial.print((int)yAngle);
 Serial.print(" ; ");
 Serial.print((int)xAngle);
 Serial.print(")");
 Serial.println();
 
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
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
Serial.print("Altitude: ");
Serial.print(bmp.pressureToAltitude(seaLevelPressure,
event.pressure,
temperature));
Serial.println(" m");
Serial.println("");
}
else
{
Serial.println("Sensor error");
}
}

void angPosFilterExpMA(float val[])
{
 //ArrayList val = new ArrayList();
 APxF = (1-alphaAP)*APxF + alphaAP*val[0];
 APyF = (1-alphaAP)*APyF + alphaAP*val[1];
 APzF = (1-alphaAP)*APzF + alphaAP*val[2];
 /*
 val.add((float) gxF);
 val.add((float) gyF);
 val.add((float) gzF);
 */
 val[0] = APxF;
 val[1] = APyF;
 val[2] = APzF; 
}

void displayValues(int b, int p, int r)
{    
   Serial.print(" Compass: [R, P, Y] = [");
   Serial.print(r);  
   Serial.print(" , ");
   Serial.print(p);
   Serial.print(" , ");
   Serial.print(b);
   Serial.print(" ]");
   //Serial.println("");
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
  }else if(scale == 500){
    writeRegister(gyroAddress, CTRL_REG4, 0b00010000);
  }else{
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
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
