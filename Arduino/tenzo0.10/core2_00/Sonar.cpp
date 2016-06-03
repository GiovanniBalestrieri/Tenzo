//#ifndef 
#include <Arduino.h>
#include "Sonar.h"

Sonar::Sonar(int trigger, int echo)
{
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  Serial.println("[ OK ] Sonar");
  
  _echo = echo;
  _trigger = trigger;
  _k1 = 0.034;
  _threshold = 38000;
  duration = 0;  
  _distance = 0;
}

float Sonar::getDistance()
{

  sonarTimer = micros();
  
  // [max] 1300 us [avg] 1230 us     
  _distance = _k1 * this->getDuration() / 2;
  
  
  contSonarRoutine++;  
  sonarTimer = micros() - sonarTimer;
  if (maxsonarTimer <= sonarTimer)
    maxsonarTimer = sonarTimer;
  sonarTimeTot = sonarTimeTot + sonarTimer;
  
  return _distance;
}

void Sonar::printAltitude()
{
  Serial.print("\nZ = ");
  Serial.print(this->_distance);
}


unsigned long Sonar::getDuration()
{
  digitalWrite(_trigger, LOW);  // Added this line
  delayMicroseconds(19); // Added this line
  digitalWrite(_trigger, HIGH);
  delay(1); // Added this line
  digitalWrite(_trigger, LOW);
  duration = pulseIn(_echo, HIGH);
  return duration;
}

  


