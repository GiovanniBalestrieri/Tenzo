//Arduino with Tiny RTC I2C http://zygzax.com/?lang=en
#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 RTC;

void setup () {
  //Initialize the serial port, wire library and RTC module
  Serial.begin(115200);
  Wire.begin();
  RTC.begin();
  //If we remove the comment from the following line, we will set up the module time and date with the computer one
  RTC.adjust(DateTime(__DATE__, __TIME__));
}
unsigned long sonarTimer;
void loop () {
  
  sonarTimer = micros();
  DateTime now = RTC.now();

  sonarTimer = micros() - sonarTimer;
  
  //We print the day
  Serial.print(now.day());
  Serial.print('/');
  //We print the month
  Serial.print(now.month());
  Serial.print('/');
  //We print the year
  Serial.print(now.year());
  Serial.print(' ');
  //We print the hour
  Serial.print(now.hour());
  Serial.print(':');
  //We print the minutes
  Serial.print(now.minute());
  Serial.print(':');
  //We print the seconds
  Serial.print(now.second());
  Serial.print("\t\t\t\tTimer:");
  Serial.print(sonarTimer);
  Serial.println();
  //We check the time and sent through the serial port every 3s
  delay(3000);
}
