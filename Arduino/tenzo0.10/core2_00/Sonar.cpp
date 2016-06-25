#include <Arduino.h>
#include "Sonar.h"
#include <Wire.h>

Sonar::Sonar()
{
  distance = 0;
}

void Sonar::init()
{
  Wire.beginTransmission(112); 
  Wire.write(byte(0x00));     
  Wire.write(byte(0x51));     
  Wire.endTransmission();   
}

int Sonar::getDistance()
{
  Wire.beginTransmission(112); // transmit to device #112
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting
  
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
  Serial.print("\n\t\t\t\tRelative Altitude = ");
  Serial.println(this->distance);
}  


