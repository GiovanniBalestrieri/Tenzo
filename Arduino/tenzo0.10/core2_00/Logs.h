#ifndef LOGS_H
#define LOGS_H

extern char state[],wcet[],inertial[],warning[],error[];

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
    

 private:
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
};
#endif
