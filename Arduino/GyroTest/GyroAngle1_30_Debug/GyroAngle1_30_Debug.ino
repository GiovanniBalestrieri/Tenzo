#include <Wire.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

//use address 104 if CS is not connected
int L3G4200D_Address = 105; //I2C address of the L3G4200D

int zOld, xOld, yOld, xCand, yCand, zCand;
int threshold = 200;

float bx= 0,by=0,bz=0;
long bxS,byS,bzS;


// delta T control the routine frequency
float deltaT = 5;
float timerLoop = 0, timerReading = 0, timerSec = 0;
float timerRoutine = 0, count = 0;
float redingTime = 0, samplingTime = 0, calcTime =0, printTime = 0;
float k=0, kM1=0, kMReading = 0, kMRoutine=0, kMLoop=0, secRoutine=0;

//Boolean
boolean processing = false;
// Timer variables
volatile int cont = 0;
volatile int contSamples=0;
volatile int contCalc=0;
volatile float phi=0;
volatile float theta=0;
volatile float psi=0;
volatile int x=0;
volatile int y = 0;
volatile int z= 0;
volatile int dt=0;
//volatile int dtM1=0;
//volatile int dtM2=0;
//volatile int dtM3=0;
//volatile int dtM4=0;
//volatile int dtM5=0;
//volatile int dtM6=0;
//volatile int dtM7=0;
//volatile int dtM8=0;
//volatile int dtM9=0;
 
volatile int dtM1=0;
volatile int dtM2=0;
volatile int dtM3=0;
volatile int dtM4=0;
volatile int dtM5=0;
volatile int yM1=0;
volatile int yM2=0;
volatile int yM3=0;
volatile int yM4=0;
volatile int yM5=0;
volatile float thetaM1=0;
volatile float thetaM2=0;
volatile float thetaM3=0;
volatile float thetaM4=0;
volatile float thetaM5=0;
volatile float thetaOLD = 0;
volatile float thetaOLDM1=0;
volatile float thetaOLDM2=0;
volatile float thetaOLDM3=0;
volatile float thetaOLDM4=0;
volatile float thetaOLDM5=0;

volatile float wF[3];
volatile boolean enFil = true;

// Filter coeff
volatile float alphaW = 0.7;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  if (!processing)
  {
   Serial.println("starting up L3G4200D");
   //Serial.println("Setting up timer3");
  }
   
   
  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  delay(1500); //wait for the sensor to be ready   
  calcBias();
  Serial.print("Readings take: (us)");
  Serial.println(samplingTime);
   
   
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
  
  //Serial.println("Interrupt enabled");
  
  k = micros();
}

void wFilter(volatile float val[])
{
 //ArrayList val = new ArrayList();
 val[0] = (1-alphaW)*x + alphaW*val[0];
 val[1] = (1-alphaW)*y + alphaW*val[1];
 val[2] = (1-alphaW)*z + alphaW*val[2];
 
 /*
 val[0] = wxF;
 val[1] = wyF;
 val[2] = wzF; 
 /*/
 }

void loop()
{
  //SerialRoutine();
  //timerLoop = micros()-kMLoop;
  //kMLoop = micros();
  
  timerSec = micros()-secRoutine;
  
  if (timerSec >= 1000000)
  {
      secRoutine = micros();
      Serial.print(cont);
      Serial.print("    Smpl: ");
      Serial.print(contSamples);
      Serial.print("    calc: ");
      Serial.println(contCalc);
      Serial.print("    wX: ");
      Serial.print(wF[0]);
      Serial.print("    Wy: ");
      Serial.println(wF[1]);
      Serial.print("    Wz: ");
      Serial.println(wF[2]);
      cont=0;      
      contSamples=0;      
      contCalc=0;      
  }
  
  
  timerRoutine = micros()-kMRoutine;
  if (timerRoutine >= deltaT*1000)
  {
    //getGyroValues();      
    kMRoutine = micros();    
    count += 1;
    if (count >= 3)
    {
      //Serial.println(cont);
       
      count = 0;
      printSerialAngle();
      //printOmega();
      //printT();
    }
  }
}


void debug()
{
        Serial.print("thetaOld + y*dt = theta: ");
      Serial.print("  ");
      Serial.print(thetaOLD);
      Serial.print(" + ");
      Serial.print(y);
      Serial.print("  ");
      Serial.print(dt);
      Serial.print(" =  ");
      Serial.println(theta);
      Serial.print("              ");
      Serial.print("  ");
      Serial.print(thetaOLDM1);
      Serial.print(" + ");
      Serial.print(yM1);
      Serial.print("  ");
      Serial.print(dtM1);
      Serial.print(" =  ");
      Serial.println(thetaM1);
      Serial.print("             ");
      Serial.print(thetaOLDM2);
      Serial.print(" + ");
      Serial.print(yM2);
      Serial.print("  ");
      Serial.print(dtM2);
      Serial.print(" =  ");
      Serial.println(thetaM2);
      Serial.print("              ");
      Serial.print(thetaOLDM3);
      Serial.print(" + ");
      Serial.print(yM3);
      Serial.print("  ");
      Serial.print(dtM3);
      Serial.print(" =  ");
      Serial.println(thetaM3);
      Serial.print("              ");
      Serial.print(thetaOLDM4);
      Serial.print(" + ");
      Serial.print(yM4);
      Serial.print("  ");
      Serial.print(dtM4);
      Serial.print(" =  ");
      Serial.println(thetaM4);
      Serial.print("              ");
      Serial.print(thetaOLDM5);
      Serial.print(" + ");
      Serial.print(yM5);
      Serial.print("  ");
      Serial.print(dtM5);
      Serial.print(" =  ");
      Serial.println(thetaM5);
}

ISR(TIMER3_COMPB_vect)
{  
  // Mettilo altrimenti non funziona
  sei();
  // Update x, y, and z with new values 2.5ms     
  getGyroValues(); 
  calcAngle();
  cont++;
}

void SerialRoutine()
{
  if (Serial.available()>0)
  {
    byte in = Serial.read();
    if (in=='A')
    {
     Serial.println("Ciao");
    }
    else if (in == 'B')
    {
      Serial.println("Bella");
    }
  }    
}

void calcAngle()
{
 dt = micros()-k;
 phi=phi+x*(double)dt/1000000.0;
 /*
 Serial.print("  Dt: "); 
 Serial.print(dt);
 Serial.print("  Wy "); 
 Serial.print(y);
 Serial.print("  thetaOLD "); 
 Serial.print(theta);
 */
 thetaOLD = theta;
 theta=theta+y*(float) dt/1000000.0;
 /*
 Serial.print("  thetaNEW "); 
 Serial.println(theta);
 */
 psi=psi+z*(double)dt/1000000.0;
 contCalc++;
 
 
// dtM9=dtM8;
// dtM8=dtM7; 
// dtM7=dtM6;
// dtM6=dtM5; 


 dtM5=dtM4;
 dtM4=dtM3; 
 dtM3=dtM2;
 dtM2=dtM1;
 dtM1=dt;
 
 yM5=yM4;
 yM4=yM3; 
 yM3=yM2;
 yM2=yM1;
 yM1=y;
 
 thetaM5=thetaM4;
 thetaM4=thetaM3; 
 thetaM3=thetaM2;
 thetaM2=thetaM1;
 thetaM1=theta;
 
 
 thetaOLDM5=thetaOLDM4;
 thetaOLDM4=thetaOLDM3; 
 thetaOLDM3=thetaOLDM2;
 thetaOLDM2=thetaOLDM1;
 thetaOLDM1=thetaOLD;
 
 k=micros();  
}

void printT()
{
  Serial.print("  timer loop: "); 
  Serial.print(timerLoop);
  Serial.print("  timer reading: "); 
  Serial.print(timerReading);
  Serial.print("  timer routine: "); 
  Serial.print(timerRoutine);
  //Serial.print("  Calc time: "); 
  //Serial.print(calcTime);
  //Serial.print("  Sampling time: "); 
  //Serial.print(samplingTime);
  Serial.print("  dt: "); 
  Serial.println(dt);
}

void getGyroValues()
{  
  
  //starting samplingTimer
  //samplingTime = micros();
  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);
  
  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  y = ((yMSB << 8) | yLSB);
  
  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
    
  removeBias();
  
  if (enFil)
  {
    wF[0] = x;
    wF[1] = y;
    wF[2] = z;
    wFilter(wF);
  }

  //samplingTime = micros()- samplingTime;
  contSamples++;
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

void printOmega()
{
  Serial.print("       Wx:");
  Serial.print(x);

  Serial.print("       Wy:");
  Serial.print(y);

  Serial.print("       Wz:");
  Serial.println(z);
}

void printAngle()
{
  Serial.print("Phi:");
  Serial.print(phi);

  Serial.print("       Theta:");
  Serial.print(theta);

  Serial.print("       Psi:");
  Serial.println(psi);
}

void printSerialAngle()
{
  Serial.print(phi);
  Serial.print("                ,");
  Serial.print(theta);
  Serial.print("                ,");
  Serial.println(psi);
}

void calcBias()
{  
  if (!processing)
    Serial.println("Bias");
  int c = 2000;
  for (int i = 0; i<c; i++)
  {
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

void removeBias()
{
  x = x - bx;
  y = y - by;
  z = z - bz; 
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
