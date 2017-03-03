/*
  Propulsion.cpp - Library for Sd Card Logging code.
  Created by Giovanni Balestrieri - UserK, June 29, 2015
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
}

boolean Logs::init()
{
  //Serial.print("\nInitializing SD card...");
  pinMode(53, OUTPUT);
   
  if (!SD.begin(_chipSelect)) 
  {
    cardOK = false;
  } 
  else 
  {
    cardOK = true;
    File logFile = SD.open(logPath, FILE_WRITE);
    logFile.close();
    File warningFile = SD.open(warningPath, FILE_WRITE);
    warningFile.close();
    File errorFile = SD.open(errorPath, FILE_WRITE);
    errorFile.close();
    File wcetFile = SD.open(wcetPath, FILE_WRITE);
    wcetFile.close();

    sessionStart();
  }  
  return cardOK;
}

/*
 * Log the start of the new session with date from RTC
 */
void Logs::sessionStart()
{  
    this->logSessionFile(logPath);
    this->logSessionFile(errorPath);
    this->logSessionFile(warningPath);
    this->logSessionFile(wcetPath);
}


/*
 * Log Session Data
 */
void Logs::logSession()
{
  if (this->cardOK && this->checkFileLog())
  {
    now_instant = RTC.now();

    this->openLogFile();
    
    logFile.print(" Session ");
    logFile.print(now_instant.day());
    logFile.print('/');
    //We print the month
    logFile.print(now_instant.month());
    logFile.print('/');
    //We print the year
    logFile.print(now_instant.year());
    logFile.print(' ');
    //We print the hour
    logFile.print(now_instant.hour());
    logFile.print(':');
    //We print the minutes
    logFile.print(now_instant.minute());
    logFile.print(':');
    //We print the seconds
    logFile.print(now_instant.second());
    logFile.println();
    
    this->closeLogFile();  
  }  
}

/*
 * Log Session Data
 */
void Logs::logSessionFile(String path)
{
  if (this->cardOK && this->checkFileLog())
  {
    now_instant = RTC.now();

    // this method opens a file and saves the object in a the public File file variable
    File file = this->openFile(path);
    
    file.print(" Session ");
    file.print(now_instant.day());
    file.print('/');
    //We print the month
    file.print(now_instant.month());
    file.print('/');
    //We print the year
    file.print(now_instant.year());
    file.print(' ');
    //We print the hour
    file.print(now_instant.hour());
    file.print(':');
    //We print the minutes
    file.print(now_instant.minute());
    file.print(':');
    //We print the seconds
    file.print(now_instant.second());
    file.println();

    // Needs to be closed to free the resource
    closeFile(file);  
  }  
}

/*
 * Log states
 */
void Logs::logStates(int initialized, int hovering, int landed)
{
  if (this->cardOK && this->checkFileLog())
  {
    this->openLogFile();
    logFile.print("S,");
    logFile.print(initialized);
    logFile.print(',');
    logFile.print(hovering);
    logFile.print(',');
    logFile.print(landed);
    logFile.println();
    this->closeLogFile(); 
  }
}

/*
 * Log Accelerometer's value
 */
void Logs::logAcc(float x,float y,float z)
{
  if (this->cardOK && this->checkFileLog())
  {
    this->openLogFile();
    logFile.print("Acc,");
    logFile.print(x);
    logFile.print(',');
    logFile.print(y);
    logFile.print(',');
    logFile.print(z);
    logFile.print(",z");
    logFile.println();
    this->closeLogFile(); 
  }
}

/*
 * Log Gyro's value
 */
void Logs::logGyro(float x,float y,float z)
{
  if (this->cardOK && this->checkFileLog())
  {
    this->openLogFile();
    logFile.print("Gyro,");
    logFile.print(x);
    logFile.print(',');
    logFile.print(y);
    logFile.print(',');
    logFile.print(z);
    logFile.print(",z");
    logFile.println();
    this->closeLogFile(); 
  }
}

/*
 * Log Orientation's value
 */
void Logs::logOrientation(float x,float y,float z)
{
  if (this->cardOK && this->checkFileLog())
  {
    this->openLogFile();
    logFile.print("EstAngle,");
    logFile.print(x);
    logFile.print(',');
    logFile.print(y);
    logFile.print(',');
    logFile.print(z);
    logFile.print(",z");
    logFile.println();
    this->closeLogFile(); 
  }
}

/*
 * Log Altitude's value
 */
void Logs::logAltitude(float a)
{
  if (this->cardOK && this->checkFileLog())
  {
    this->openLogFile();
    logFile.print("altitude,");
    logFile.print(a);
    logFile.print(",z");
    logFile.println();
    this->closeLogFile(); 
  }
}


/*
 * Log Gps's value
 */
void Logs::logGps(String lat, String lon)
{
  if (this->cardOK && this->checkFileLog())
  {
    this->openLogFile();
    logFile.print("gps,");
    logFile.print(lat);
    logFile.print(",");
    logFile.print(lon);
    logFile.print(",z");
    logFile.println();
    this->closeLogFile(); 
  }
}

/*
 * Log Setpoint value
 */
void Logs::logSetpoint(float altSet)
{
  if (this->cardOK && this->checkFileLog())
  {
    this->openLogFile();
    logFile.print("AltSetpt,");
    logFile.print(altSet);
    logFile.print(",z");
    logFile.println();
    this->closeLogFile(); 
  }
}

/*
 * Log 'Worst Case Execution Time' value
 */
void Logs::logWcet(float val,int taskID,String label)
{
  if (this->cardOK && this->checkFileWcet())
  {
    this->openWcetFile();
    wcetFile.print("Task:");
    wcetFile.print(taskID);
    wcetFile.print("\tlabel:");
    wcetFile.print(label);
    wcetFile.print("\twcet:");
    wcetFile.print(val);
    wcetFile.println();
    this->closeWcetFile(); 
  }
}
      


/*
 * Checks File existence
 */
boolean Logs::checkFileLog()
{
  if (SD.exists(logPath))
  {
    logFileCheck = true;
    logFile.close();
  }  
  else 
  {
    logFileCheck = false;    
  }
  return logFileCheck;
}

/*
 * Checks File existence
 */
boolean Logs::checkFileError()
{
  if (SD.exists(errorPath))
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
  if (SD.exists(warningPath))
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
  if (SD.exists(wcetPath))
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
  if (this->checkFileWcet() && this->checkFileWarning() && this->checkFileError() && this->checkFileLog())
  {
    checkFilesExistence = true;
  }
  else
  {
    checkFilesExistence = false;
  }
  
  return checkFilesExistence;
}

void Logs::openLogFile()
{
  logFile = SD.open(logPath,FILE_WRITE);
}

void Logs::closeLogFile()
{
  logFile.close();
}

File Logs::openFile(String a)
{
  return SD.open(a,FILE_WRITE);
}

void Logs::closeFile(File a)
{
  a.close();
}

void Logs::openErrorFile()
{
  errorFile = SD.open(errorPath,FILE_WRITE);
}

void Logs::closeErrorFile()
{
  errorFile.close();
}

void Logs::openWarningFile()
{
  warningFile = SD.open(warningPath,FILE_WRITE);
}

void Logs::closeWarningFile()
{
  warningFile.close();
}

void Logs::openWcetFile()
{
  wcetFile = SD.open(wcetPath,FILE_WRITE);
}

void Logs::closeWcetFile()
{
  wcetFile.close();
}

