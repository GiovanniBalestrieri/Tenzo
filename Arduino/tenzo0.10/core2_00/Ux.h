/*
  Ux.h - Library to handle user interaction.
  Created by Giovanni Balestrieri - UserK, August 26, 2015.
  www.userk.co.uk
*/
#ifndef UX_H
#define UX_H

#include "Arduino.h"

// Constants
const int VERSION = 0.13;  
const String welcomeMSG = "Welcome to Tenzo";
const String MIN_ = "700";
const String IDLE_ = "790";

const int MOTOR_1 = 3;
const int MOTOR_2 = 5;
const int MOTOR_3 = 22;
const int MOTOR_4 = 9;

extern boolean printSerial;
extern boolean processing;
extern boolean printBlue;
extern boolean printMotorsVals;
extern boolean printMotorsValsUs;
extern boolean printPIDVals;
extern boolean printSerialInfo;
extern boolean printTimers;
extern boolean printAccs;
extern boolean printMotorsPid;
extern boolean printOmegas;
extern boolean sendBlueAngle;
extern boolean serialByteProtocol;
extern boolean printVerboseSerial;
extern boolean removeSpikesNumerically;
extern volatile boolean filterGyro;
extern volatile boolean filterAcc;
extern const double baudRate;
extern const int VERSION;
extern const int MOTOR_1;
extern const int MOTOR_2;
extern const int MOTOR_3;
extern const int MOTOR_4;

class Ux
{
  public:
        Ux();
        void welcome();
        void warning();
        void feedback();
        
        int getM(int);
        boolean getPrintMotorVals();
        boolean getPrintMotorValsUs();
        boolean getVersion();
        boolean getPrintMotorPid();
        boolean getProcessing();
        boolean getPrintBlue();
        boolean getPrintSerialInfo();
        boolean getPrintPIDVals();
        boolean getPrintSerial();
        boolean getPrintTimers();
        boolean getPrintOmegas();
        boolean getPrintAccs();
        boolean getSendBlueAngle();
        boolean getSerialByteProtocol();
        boolean getPrintVerboseSerial();
        boolean getGyroFilterFlag();
        boolean getAccFilterFlag();
        double getBaudRate();
        
        boolean printMotorsVals;
        boolean printMotorsValsUs;
        double baudrate;
        double motor_1;
        double motor_2;
        double motor_3;
        double motor_4;
        
        void setPrintMotorVals(boolean);
        void setPrintMotorValsUs(boolean);
        void setPrintMotorPid(boolean);
        void setPrintPIDVals(boolean);
        void setProcessing(boolean);
        void setPrintBlue(boolean);
        void setPrintAccs(boolean);
        void setPrintSerialInfo(boolean);
        void setPrintSerial(boolean);
        void setPrintTimers(boolean);
        void setPrintOmegas(boolean);
        void setSendBlueAngle(boolean);
        void setSerialByteProtocol(boolean);
        void setPrintVerboseSerial(boolean);
        void setGyroFilterFlag(volatile boolean);
        void setAccFilterFlag(volatile boolean);
        
        
        
 private:
       boolean _printMotorsPid;
       boolean _processing;
       boolean _printPIDVals;
       boolean _printSerialInfo;
       boolean _printSerial;
       boolean _printTimers;
       boolean _printAccs;
       boolean _printBlue;
       boolean _printOmegas;
       boolean _sendBlueAngle;
       boolean _serialByteProtocol;
       boolean _printVerboseSerial;
       volatile boolean _filterAcc;
       volatile boolean _filterGyro;
};
#endif
