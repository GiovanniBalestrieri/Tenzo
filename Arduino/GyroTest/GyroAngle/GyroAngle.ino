//Arduino 1.0+ only

#include <Wire.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

//use address 104 if CS is not connected
int L3G4200D_Address = 105; //I2C address of the L3G4200D

int x;
int y;
int z;

int bx,by,bz;
long bxS,byS,bzS;

float phi=0;
float theta=0;
float psi=0;

float deltaT = 2.5, count = 0;
long k=0, kM1=0, kM2=0;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("starting up L3G4200D");
  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  delay(1500); //wait for the sensor to be ready   
  calcBias();
  k = millis();
}

void loop()
{
  getGyroValues();  // This will update x, y, and z with new values
  if (millis()-kM1>=deltaT)
  {
    kM1 = millis();
    count += 1;
    calcAngle();
    if (count >= 200)
    {
      count = 0;
      printAngle();
      printOmega();
    }
  }
  
  //printOmega();
}

void calcBias()
{  
  int c = 1000;
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
  
  Serial.print(" Bias: [ ");
  Serial.print(bx);
  Serial.print("  ;  ");
  Serial.print(by);
  Serial.print("  :  ");
  Serial.println(bz);
}

void calcAngle()
{
 int dt = millis()-k;
 phi=phi+x*(double)dt/1000.0;
 theta=theta+y*(double)dt/1000.0;
 psi=psi+z*(double)dt/1000.0;
// Serial.print("SSSSS  ");
// Serial.print(dt);
// Serial.println("  SSSS  ");
 k=millis();  
}

void getGyroValues()
{
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
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b01011111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000100);

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
        Serial.println("No Data");
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
