//#ifndef 
#include <Arduino.h>
#include "Sonar.h"
#include <Wire.h>

Sonar::Sonar()
{
  Wire.begin();
  Wire.beginTransmission(112); 

  Wire.write(byte(0x00));     
  Wire.write(byte(0x51));     
  Wire.endTransmission();    
  delay(80);
  distance = 0;
}

float Sonar::getDistance()
{
  // request 2 bytes from slave device #112
  Wire.requestFrom(112, 2);    

  // step 5: receive reading from sensor
  if (2 <= Wire.available())  
  { 
    // if two bytes were received
    // receive high byte (overwrites previous reading)
    distance = Wire.read();  
    // shift high byte to be high 8 bits
    distance = distance << 8;     
    // receive low byte as lower 8 bits
    distance |= Wire.read();
  }
  return distance;
}

void Sonar::printAltitude()
{
  Serial.print("\nZ = ");
  Serial.print(this->distance);
}
  


