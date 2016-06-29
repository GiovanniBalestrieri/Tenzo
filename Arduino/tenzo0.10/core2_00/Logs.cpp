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

boolean Logs::init()
{
  Serial.print("\nInitializing SD card...");
  pinMode(53, OUTPUT);
   
  if (!SD.begin(_chipSelect)) 
  {
    Serial.println("SdCard failed. Continuing without log:");
    _cardOk = false;
  } 
  else 
  {
    Serial.println("Wiring is correct and a card is present.");
    _cardOk = true;
  }  
  return _cardOk;
}

/*
 * Checks File existence
 */
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


/*
 * Checks File existence
 */
boolean Logs::checkFileInertial()
{
  if (inertialFile)
  {
    inertialFileCheck = true;
    inertialFile.close();
  }  
  else 
  {
    inertialFileCheck = false;    
  }
  return inertialFileCheck;
}


/*
 * Checks File existence
 */
boolean Logs::checkFileError()
{
  if (errorFile)
  {
    errorFileCheck = true;
    errorFile.close();
  }  
  else 
  {
    errorFileCheck = false;    
  }
  return errorFileCheck;
}


/*
 * Checks File existence
 */
boolean Logs::checkFileWarning()
{
  if (warningFile)
  {
    warningFileCheck = true;
    warningFile.close();
  }  
  else 
  {
    warningFileCheck = false;    
  }
  return warningFileCheck;
}

/*
 * Checks File existence
 */
boolean Logs::checkFileWcet()
{
  if (wcetFile)
  {
    wcetFileCheck = true;
    wcetFile.close();
  }  
  else 
  {
    wcetFileCheck = false;    
  }
  return wcetFileCheck;
}


/*
 * Checks All Files
 */
boolean Logs::checkFiles()
{
  if (this->checkFileWcet() && this->checkFileWarning() && this->checkFileError() && this->checkFileState() && this->checkFileInertial())
  {
    checkFilesExistence = true;
  }
  else
  {
    checkFilesExistence = false;
  }
  return checkFilesExistence;
}



/*
void Logs::printAltitude()
{
  Serial.print("\n\t\t\t\tRelative Altitude = ");
  Serial.println(this->distance);
}  
*/


