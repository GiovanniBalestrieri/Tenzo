/*
  Propulsion.cpp - Library for Tenzo Propulsion code.
  Created by Giovanni Balestrieri - UserK, June 2, 2015
  Released into the public domain.
*/

#include <Arduino.h>
#include "Logs.h"
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>

Logs::Logs()
{
  extern RTC_DS1307 RTC;
  
  File stateFile = SD.open(statePath, FILE_WRITE);
  File inertialFile = SD.open(inertialPath, FILE_WRITE);
  File warningFile = SD.open(warningPath, FILE_WRITE);
  File errorFile = SD.open(errorPath, FILE_WRITE);
  File wcetFile = SD.open(wcetPath, FILE_WRITE);
}

void Logs::init()
{
  Serial.print("\nInitializing SD card...");
  pinMode(53, OUTPUT);
   
  if (!SD.begin(_chipSelect)) 
  {
    Serial.println("SdCard failed. Continuing without log:");
  } 
  else 
  {
    Serial.println("Wiring is correct and a card is present."); 
    //folder = SD.open("/arduino");
    //printDirectory(folder,0);
    //folder.close();
  }  
}

boolean Logs::checkFileState()
{
  if (stateFile)
  {
    stateFileCheck = true;
    stateFile.close();
  }  
  else 
  {
    stateFileCheck = false;    
  }
  return stateFileCheck;
}

boolean Logs::checkFiles()
{
  
  return false;
}

/*
void Logs::printAltitude()
{
  Serial.print("\n\t\t\t\tRelative Altitude = ");
  Serial.println(this->distance);
}  
*/


