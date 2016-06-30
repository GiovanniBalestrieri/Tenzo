/*
  Propulsion.cpp - Library for Sd Card Logging code.
  Created by Giovanni Balestrieri - UserK, June 29, 2015
  Released into the public domain.
*/
#ifndef LOGS_H
#define LOGS_H
  
#include <SD.h>
#include <RTClib.h>

const char logPath[] = "arduino/log.txt";
const char errorPath[] = "arduino/error.txt";
const char warningPath[] = "arduino/warning.txt";
const char wcetPath[] = "arduino/wcet.txt";

class Logs
{
  public:
    Logs();

    boolean init();
    
    boolean initSession();
    void logStates(int,int,int);    
    void logInertial();
      void logAcc(float,float,float);
      void logGyro(float,float,float);
      void logOrientation(float,float,float);
      void logAltitude(float);
      void logGps(String,String);
      
    void logSetpoint(float);
    void logWcet(float,int,String);
    void logSession();

    /*
     * Check and Creates Files
     */
    boolean checkFiles();
      boolean checkFileLog();
      boolean checkFileWarning();
      boolean checkFileError();
      boolean checkFileWcet();

    // basic config files
    uint8_t sd_answer;

    // flags to check file presence
    boolean logFileCheck = false;
    boolean errorFileCheck = false;
    boolean warningFileCheck = false;
    boolean wcetFileCheck = false;
    boolean checkFilesExistence = false;

    /*
     * Open Close Methods
     */
     void closeLogFile();
     void closeErrorFile();
     void closeWarningFile();
     void closeWcetFile();
     
     void openLogFile();
     void openErrorFile();
     void openWarningFile();
     void openWcetFile();
    
    RTC_DS1307 RTC;
  
    File logFile;
    File warningFile;
    File errorFile;
    File wcetFile;

    DateTime now_instant;
    
    boolean cardOK = false;

 private: 
    const int _chipSelect = 4; 
};
#endif
