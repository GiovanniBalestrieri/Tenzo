#include <math.h>
#include <float.h>
#include <limits.h>

int accXIn=0,accYIn=0,accZIn=0;
int pinAccX=0,pinAccY=1,pinAccZ=2;
float accX,accY,accZ,Vcc=3.3;
float midV=Vcc/2, modeG=0.800;
float minAccX=0,maxAccX=0,minAccY=0,maxAccY=0,minAccZ=0,maxAccZ=0;
long time1 = 0;
long time2 = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("Accelerometer test Activity.\r\n");
  Serial.println("Constants:\n");
  Serial.print("Zero acceleration voltage: ");
  Serial.print(midV);
  Serial.print("V, G-selector: ");
  Serial.print(modeG);
  Serial.print("V/g ");
  pinMode(pinAccX,INPUT);
  pinMode(pinAccY,INPUT);
  pinMode(pinAccZ,INPUT);
  time1=millis();
}

void loop()
{
  accXIn=readAcc(pinAccX);
  accYIn=readAcc(pinAccY);
  accZIn=readAcc(pinAccZ);
  accX= mapIn(accXIn);
  accY= mapIn(accYIn); 
  accZ= mapIn(accZIn);
  if (accX < minAccX)
    minAccX=accX;
  if (accY < minAccY)
    minAccY=accY;
  if (accZ < minAccZ)
    minAccZ=accZ;
  if (accX > maxAccX)
    maxAccX=accX;
  if (accY > maxAccY)
    maxAccY=accY;
  if (accZ > maxAccZ)
    maxAccZ=accZ;
  printMax();
  //printValues();
  if(Serial.available()>0)
  {
    char inChar=Serial.read();
    if (inChar=='R')
      resetVal();
  }
}

float mapIn(float val)
{
  float aV=val*0.005;
  return -(aV-midV)/modeG;
}

float readAcc(int axis)
{
  float val = analogRead(axis); 
  return val;
}
void printMax()
{
  Serial.println();
  Serial.print(" AccX [");
  Serial.print(minAccX);
  Serial.print(",");
  Serial.print(maxAccX);
  Serial.print("],      AccY [ ");;
  Serial.print(minAccY);
  Serial.print(",");
  Serial.print(maxAccY);
  Serial.print("],      AccZ [ ");
  Serial.print(minAccZ);
  Serial.print(",");
  Serial.print(maxAccZ);
  Serial.print("]");
  Serial.println();
}

void printValues()
{
  Serial.println();
  Serial.print(accXIn);
  Serial.print(", ");
  Serial.print(accX);
  Serial.print(" g,      ");
  Serial.print(accYIn);
  Serial.print(", ");
  Serial.print(accY);
  Serial.print(" g,      ");
  Serial.print(accZIn);
  Serial.print(", ");
  Serial.print(accZ);
  Serial.print(" g"); 
  Serial.println();
}
void resetVal()
{
  minAccX=0;
  minAccY=0;
  minAccZ=0;
  maxAccX=0;
  maxAccY=0;
  maxAccZ=0;
}



