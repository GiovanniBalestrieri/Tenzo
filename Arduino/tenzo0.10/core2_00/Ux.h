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


byte mode;
char readAnswer, readChar, readCh;

byte modeS; 

/**
 ** Serial 
 **/
 
// Gps
int BaudRateGps = 4800;
byte loBytew1, hiBytew1,loBytew2, hiBytew2;
int loWord,hiWord;

int printBlueAngleCounter = 0;
int printBlueAngleSkip = 5;

String inString = "";
String inComingString = "";

// Serial remote gains PID change

char kReadChar;
char k1ReadChar;
int k3ReadInt;
float readPropVal,readIntVal,readDerVal,readSetVal;
int readChar2;

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
        int getBaudRate();
        
        boolean printMotorsVals;
        boolean printMotorsValsUs;
        
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
       double _baudrate;
};
#endif
