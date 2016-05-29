#include <Wire.h>
#include "FreeSixIMU.h"
#include "FIMU_ADXL345.h"
#include "FIMU_ITG3200.h"
 
float angles[3];
float heading;
long Temperature = 0, Pressure = 0, Altitude = 0;
 
// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();
 
// Record any errors that may occur in the compass.
int error = 0;
 
void setup(){

  Serial2.begin(9600);
  Serial.begin(115200);
  Wire.begin();
 
  delay(5);
  sixDOF.init(); //init the Acc and Gyro 
}
 
void loop(){   
  sixDOF.getEuler(angles);
  SerialRoutine();
  PrintData();
  delay(400);
}

void SerialRoutine()
{
  if (Serial2.available())
  {
    char t = Serial2.read();
    if (t=='a')
    {      
      Serial2.print("a,");
      Serial2.print(angles[0]);
      Serial2.print(",");
      Serial2.print(angles[1]);
      Serial2.print(",");
      Serial2.print(angles[2]);
      Serial2.println(",z");
    }
  }
  
}
 
void PrintData()
{
  Serial.print("a,");
  Serial.print(angles[0]);
  Serial.print(",");
  Serial.print(angles[1]);
  Serial.print(",");
  Serial.print(angles[2]);
  Serial.println(",z");   
}
