#ifndef LOGS_H
#define LOGS_H

  
#include <SD.h>
#include <RTClib.h>

extern char statePath[],wcetPath[],inertialPath[],warningPath[],errorPath[];

class Logs
{
  public:
    Logs();
    void init();
    boolean initSession();
    boolean logState();
    boolean logAcc();
    boolean logGyro();
    boolean logOrientation();
    boolean logAltitude();
    boolean logGps();
    boolean logInertial();
    boolean logSetpoint();
    boolean logWcet();

    // basic config files
    uint8_t sd_answer;

    // flags to check file presence
    boolean stateFileCheck = false;
    boolean inertialFileCheck = false;
    boolean errorFileCheck = false;
    boolean warningFileCheck = false;
    boolean wcetFileCheck = false;
    boolean filesCheck = false;
    
    RTC_DS1307 RTC;
  
    File stateFile;
    File inertialFile;
    File warningFile;
    File errorFile;
    File wcetFile;

    
     boolean checkCard();
     boolean checkFileState();
     boolean checkFileInertial();
     boolean checkFileWarning();
     boolean checkFileError();
     boolean checkFileWcet();
     boolean checkFiles();
     boolean createFileState();
     boolean createFileInertial();
     boolean createFileWarning();
     boolean createFileError();
     boolean createFileWcet();
     boolean createFiles();

 private:
 
    const int _chipSelect = 4; 
};
#endif
