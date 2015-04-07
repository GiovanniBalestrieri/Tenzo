#include <Wire.h>

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
int L3G4200D_Address = 105; //I2C address of the L3G4200D

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

// delta T control the routine frequency
float deltaT = 5;
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

// Volatile vars
boolean processing = true;
// Timer variables
volatile int cont = 0;
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
volatile boolean initialized = false;


volatile float angleXAcc;
volatile float angleYAcc;

volatile float aax,aay,aaz;
volatile float alphaA = 0.2, alphaW = 0.8;
volatile float estXAngle = 0, estYAngle = 0, estZAngle = 0;
volatile float kG = 0.85, kA = 0.15, kGZ=0.60, kAZ = 0.40;

void setup()
{  
  Wire.begin();
  Serial.begin(115200); // 9600 bps
  pinMode(xaxis,INPUT);
  pinMode(yaxis,INPUT);
  pinMode(zaxis,INPUT);
  
  
  if (!processing)
  {
    Serial.println("starting up L3G4200D");
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
  
  initialized = true;
  k = micros();
}

void loop()
{
 //accRoutine(); 
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
  calcAngle();
  estAngle();
  cont++;
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
      //Serial.print("    calc: ");
      //Serial.println(contCalc);
    }
    cont=0;      
    contSamples=0;      
    contCalc=0;      
  }

  timerRoutine = micros()-kMRoutine;
  
  // The following loop runs every 5ms
  if (timerRoutine >= deltaT*1000) 
  {      
    kMRoutine = micros();    
    count += 1;
    if (count >= 10)
    {
      count = 0;
      printSerialAngle();
      //printAcc();
      //printOmega();
      //printT();
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
  
  if (initialized)
    removeBias();
    
  /*
    Serial.println();
    Serial.print("  ");
    Serial.print(x);
    Serial.print("  ");
    Serial.print(y);
    Serial.print("   ");
    Serial.println(z);
 */
 /*
  if (filterGyro)
  {
    wF[0] = x;
    wF[1] = y;
    wF[2] = z;
    wFilter(wF);
  }
  */

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
  Serial.println(estYAngle);
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

