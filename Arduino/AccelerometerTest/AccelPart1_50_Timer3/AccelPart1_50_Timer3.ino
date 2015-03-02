//#define DEBUGMODE

#include <Servo.h>
#include <Wire.h>
#include <CMPS10.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include "Kalman.h"
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <PID_v1.h>

int versionCode = 34;

/** 
 * Kalman
 */
 
// Final values
int pitchK;
int rollK;
int yawK;

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

/**
 * Compass
 */

CMPS10 my_compass;
float angPosFilter[3];  

/**
  *  Gyroscope 
  */
int wxCandidate,wyCandidate,wzCandidate;
int gyrothresholdHigh = 1000;
int wx,wy,wz,wx_past,wy_past,wz_past;
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

//0.04
float aButter3[5] = {0,1, -2.7488,2.5282,-0.7776};
float bButter3[5] = {0,0.0002196,0.0006588,0.0006588,0.0002196};

// Acc Timers
unsigned long accTimer;
unsigned long lastAccTimer;
unsigned long timeToRead = 0;
unsigned long lastTimeToRead = 0;

int contSamples = 0;
int samplesNum = 500;

byte mode;
unsigned int sensorValue = 0;
boolean enableFilter = true;
boolean enableKalman = true;

int lenBuff = 1000;
float values[1000];

int rate = 0;
int pastTime = 0;
int timerRate = 0;
int pastTimerTime = 0;

int rampTill = 95;
int motorRampDelayFast = 150;
int throttle = 0;

unsigned long aCont;

// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

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
 Serial.begin(9600); // 9600 bps
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
  
 Serial.print("Tenzo: ");
 Serial.println(versionCode); 
 
 /**
  *  Initialize Accelerometer Timer3 800 Hz
  */
 cli();          // disable global interrupts
 TCCR3A = 0;     // set entire TCCR3A register to 0
 TCCR3B = 0;     // same for TCCR3B
 
 // set compare match register to desired timer count: 800 Hz
 OCR3A = 19; //800Hz 5000; // 3 Hz
 // turn on CTC mode:
 TCCR3B |= (1 << WGM32);
 // Set CS10 and CS12 bits for 1024 prescaler:
 TCCR3B |= (1 << CS30) | (1 << CS32);
 // enable timer compare interrupt:
 TIMSK3 |= (1 << OCIE3B);
 // enable global interrupts:
 sei();
 
 // set up the ADC
 ADCSRA &= ~PS_128;  // remove bits set by Arduino library
 // you can choose a prescaler from above.
 // PS_16, PS_32, PS_64 or PS_128
 ADCSRA |= PS_32;    // set our own prescaler to 64 
 
 Serial.println(" Setting up L3G4200D");
 setupL3G4200D(2000);
 delay(500);
 Serial.println("Setup Completed");
}

void loop()
{
  while(true)
  {
    rate = micros() - pastTime;
    pastTime= micros();
    getGyroValues();
    getCompassValues();
    estimateAngle();
    serialRoutine();
    dispCountersSpeed();
  }
} 
void estimateAngle()
{
  xAngle = kalmanX.getAngle(angleXAcc,wx,(double)(micros() - timerMu)) + kalmanXOffset;
  yAngle = kalmanY.getAngle(angleYAcc,wy,(double)(micros() - timerMu)) + kalmanYOffset;
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
    
  if (printRawKalman)
  {
    Serial.println();
    Serial.print("K:( ");
    Serial.print((int)yAngle);
    Serial.print(" ; ");
    Serial.print((int)xAngle);
    Serial.print(" ; ");
    Serial.print((int)zAngle);
    Serial.print(")     ");
    Serial.println();
  }
  if (printKalman)
  {
    Serial.println();
    Serial.print("KFilt:( ");
    Serial.print((int)  angK[1]);
    Serial.print(" ; ");
    Serial.print((int) angK[0]);
    Serial.print(" ; ");
    Serial.print((int) angK[2]);
    Serial.print(")");
    Serial.println();
  }

  pitchK = angK[0];
  rollK = angK[1];
  yawK = angK[2];

  timerMu = micros();
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
  //  Filter data  
  if (filterAng == 1)
  {    
    angPosFilter[0] = roll1;
    angPosFilter[1] = pitch1;
    angPosFilter[2] = bearing1;
    angPosFilterExpMA(angPosFilter); 
    displayValues(angPosFilter[2],angPosFilter[1],angPosFilter[0]);
  }
  else if (filterAng == 0)
  {
    displayValues(bearing1,pitch1,roll1);
  }
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
   Serial.print("   Ax --->    ");
   Serial.print(axF);
   Serial.print("   Wx --->    ");
   Serial.print(wx);
   contatore = 0;
 }
}

void initialize()
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
  
  accF3[2] = -aButter3[2]*zFM1 -aButter3[3]*zFM2 -aButter3[4]* zFM3 + bButter3[1]*val[2]+bButter3[2]*uZM1+bButter3[3]*uZM2 +bButter3[4]*uZM3;
  
  zFM3 = xFM2;
  zFM2 = xFM1;
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

void land()
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

  byte zMSB = readRegister(gyroAddress, 0x2D);
  byte zLSB = readRegister(gyroAddress, 0x2C);
  wzCandidate = ((zMSB << 8) | zLSB);
  
  //if (abs(wzCandidate-wz_past) <= gyrothresholdHigh)
  //{
  wz=wzCandidate;
  //}

  //wz_past  =  wz;
  
  wx = wx - biasWx;
  wy = wy - biasWy;
  wz = wz - biasWz; 
}

void accRoutine()
{  
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
   if (Serial.available()>1)
   {
     mode = Serial.read();
     
     if (mode == 83)
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
              values[i] = axF;
              //Serial.println(wx);
           }
           else
           {
             values[i] = axF;
           }
         }
         else
         {         
           lastAccTimer = micros(); 
           values[i] = aax;
         }
         
         //delay(3);
         //motorSpeed(throttle);
       }  
       
       for (int i=0;i<=lenBuff;i++)
       {
         Serial.print("X");
         Serial.print(",");
         Serial.println(values[i]);
       }
       
       rate = accTimer;
       Serial.print("U");
       Serial.print(",");
       Serial.println(rate);   
       //land();  
       contSamples = 0;  
     } 
     else if (mode == 84)
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
     else if (mode == 85) // 'U'
     {
       Serial.print(axF);
       Serial.print("  ");
       Serial.print(wx);
       Serial.print("  ");
       Serial.print(pastCount);
       Serial.println();
     }
    if(mode == 16)
    {  	
      Serial.write(17);
    }
    if (mode == 18)
    {
      Serial.write(19);      
      initialize();
    }
   }
}
 
ISR(TIMER3_COMPB_vect)
{
    accRoutine();
}

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
