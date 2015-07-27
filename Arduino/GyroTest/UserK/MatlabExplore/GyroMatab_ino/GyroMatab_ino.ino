//#define DEBUG
#include <Wire.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

// Declare L3G4200D address
// use address 104 if CS is not connected
int L3G4200D_Address = 105; 

int x;
int y;
int z;

int deltaT = 2;
float timerLoop = 0, timerReading = 0, count = 0;
long k=0, kM1=0;
double dt=0;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("starting up L3G4200D");
  // Configure L3G4200  - 250, 500 or 2000 deg/sec
  setupL3G4200D(2000); 
  delay(1500); //wait for the sensor to be ready   
  k = millis();
}

void loop()
{
  // This will update x, y, and z with new values
  getGyroValues(); 
  serialRoutine();
}

void serialRoutine()
{
  if(Serial.available()>0)
  {
    char t = Serial.read();
    
    if (t =='T')
    {
      Serial.println("Ok");
    }  
    else if (t== 'M')
    {  
      #ifdef DEBUG
        printTime();
      #endif
      printCSVomega();
    }
  }
}

void printTime()
{
  Serial.print("  timer loop (ms): "); 
  Serial.print(timerLoop);
  Serial.print("  timer reading (micro): "); 
  Serial.println(timerReading);
}

void getGyroValues()
{
  timerReading = micros() - dt;
  
  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
  
  dt = micros();
}

int setupL3G4200D(int scale)
{
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b01001111);
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00001001);
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

void writeRegister(int deviceAddress, byte address, byte val) 
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();    
}

int readRegister(int deviceAddress, byte address)
{
    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read from
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) 
    {
        // waiting
        Serial.println("No Data");
    }

    v = Wire.read();
    return v;
}

void printCSVomega()
{
  Serial.print("A,");
  Serial.print(x);

  Serial.print(",");
  Serial.print(y);

  Serial.print(",");
  Serial.print(z);
  Serial.println(",Z");
}

