#ifndef LOGS_H
#define LOGS_H

  
#include <SD.h>
#include <RTClib.h>

extern char statePath[],wcetPath[],inertialPath[],warningPath[],errorPath[];

class Logs
{
  public:
    Logs();

    boolean init();
    
    boolean initSession();
    boolean logState();
    
    boolean logInertial();
      boolean logAcc();
      boolean logGyro();
      boolean logOrientation();
      boolean logAltitude();
      boolean logGps();
      
    boolean logSetpoint();
    boolean logWcet();

    /*
     * Check and Creates Files
     */
    boolean checkFiles();
      boolean checkFileState();
      boolean checkFileInertial();
      boolean checkFileWarning();
      boolean checkFileError();
      boolean checkFileWcet();

    // basic config files
    uint8_t sd_answer;

    // flags to check file presence
    boolean stateFileCheck = false;
    boolean inertialFileCheck = false;
    boolean errorFileCheck = false;
    boolean warningFileCheck = false;
    boolean wcetFileCheck = false;
    boolean checkFilesExistence = false;
    
    RTC_DS1307 RTC;
  
    File stateFile;
    File inertialFile;
    File warningFile;
    File errorFile;
    File wcetFile;
    

 private: 
    const int _chipSelect = 4; 
    boolean _cardOk = false;
};
#endif
