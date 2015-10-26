/*
  propulsion.h - Library to handle user interaction.
  Created by Giovanni Balestrieri - UserK, August 26, 2015.
*/
#ifndef Ux_h
#define Ux_h

#include "Arduino.h"

// Constants
const String welcomeMSG = "Welcome to Tenzo";
const String MIN_ = "700";
const String IDLE_ = "790";

class Ux
{
  public:
        Ux();
        void welcome();
        void warning();
        void feedback();
        
        int getM(int);
        boolean getPrintMotorVals();
        boolean getVersion();
        boolean getPrintMotorPid();
        boolean getProcessing();
        boolean getPrintBlue();
        boolean getPrintSerialInfo();
        boolean getPrintSerial();
        boolean getPrintTimers();
        boolean getPrintOmegas();
        boolean getPrintAccs();
        boolean getSendBlueAngle();
        boolean getSerialByteProtocol();
        boolean getPrintVerboseSerial();
        
        void setPrintMotorVals(boolean);
        void setPrintMotorPid(boolean);
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
        
 private:
       boolean _printMotorsVals;
       boolean _printMotorsPid;
       boolean _processing;
       boolean _printSerialInfo;
       boolean _printSerial;
       boolean _printTimers;
       boolean _printAccs;
       boolean _printBlue;
       boolean _printOmegas;
       boolean _sendBlueAngle;
       boolean _serialByteProtocol;
       boolean _printVerboseSerial;
};
#endif
