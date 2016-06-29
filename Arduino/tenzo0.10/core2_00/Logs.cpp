#include <Arduino.h>
#include "Logs.h"
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>

Logs::Logs()
{
  distance = 0;
}

void Logs::init()
{
  
}

int Logs::getDistance()
{
  return distance;
}

void Logs::printAltitude()
{
  Serial.print("\n\t\t\t\tRelative Altitude = ");
  Serial.println(this->distance);
}  


