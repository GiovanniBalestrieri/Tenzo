/*
  propulsion.h - Library to handle user interaction.
  Created by Giovanni Balestrieri - UserK, August 26, 2015. 
*/

#include "Ux.h"


Ux::Ux()
{
   /*
   baudrate = baudRate;
   motor_4 = MOTOR_4;
   motor_3 = MOTOR_3;
   motor_2 = MOTOR_2;
   motor_1 = MOTOR_1;
   */
   printMotorsVals = printMotorsVals;
   printMotorsValsUs = printMotorsValsUs;
   _printMotorsPid = _printMotorsPid;
   _processing = processing;
   _printPIDVals = printPIDVals;
   _printSerialInfo = printSerialInfo;
   _printSerial = printSerial;
   _printTimers = printTimers;
   _printAccs = printAccs;
   _printBlue = printBlue;
   _printOmegas = printOmegas;
   _sendBlueAngle = sendBlueAngle;
   _serialByteProtocol = serialByteProtocol;
   _printVerboseSerial = printVerboseSerial;
   _filterGyro = filterGyro;
   _filterAcc = filterAcc;
}

void Ux::welcome()
{  
  if (!processing)
  {
    Serial.print("Welcome To Tenzo ");
    Serial.println(VERSION);
  }
}

void Ux::warning()
{
  // Constructor
}

void Ux::feedback()
{
  // Constructor
}


boolean Ux::getVersion()
{
  return VERSION;
}


int Ux::getM(int i)
{
  int a[4]={MOTOR_1,MOTOR_2,MOTOR_3,MOTOR_4};
  return a[i-1];
}


// Get Methods
boolean Ux::getPrintMotorVals()
{
  return printMotorsVals;
}

boolean Ux::getPrintMotorValsUs()
{
  return printMotorsValsUs;
}

boolean  Ux::getPrintMotorPid()
{
  return _printMotorsPid;
}

boolean  Ux::getProcessing()
{
  return _processing;
}

boolean Ux::getPrintSerial()
{
  return _printSerial;
}

boolean Ux::getPrintSerialInfo()
{
  return _printSerialInfo;
}

boolean  Ux::getPrintTimers()
{
  return _printTimers;
}

boolean Ux::getPrintAccs()
{
  return _printAccs;
}

boolean Ux::getPrintOmegas()
{
  return _printOmegas;
}

boolean Ux::getPrintBlue()
{
  return _printBlue;
}

boolean  Ux::getSendBlueAngle()
{
  return _sendBlueAngle;
}

boolean  Ux::getSerialByteProtocol()
{
  return _serialByteProtocol;
}

boolean  Ux::getPrintVerboseSerial()
{
  return _printVerboseSerial;
}

boolean  Ux::getPrintPIDVals()
{
  return _printPIDVals;
}



/**
 **  SET Methods
 **/

void Ux::setPrintMotorVals(boolean state)
{
  printMotorsVals = state;
}

void Ux::setPrintMotorValsUs(boolean state)
{
  printMotorsValsUs = state;
}

void  Ux::setPrintMotorPid(boolean state)
{
  _printMotorsPid = state;
}

void  Ux::setPrintBlue(boolean state)
{
  _printBlue = state;
}

void Ux::setPrintSerial(boolean state)
{
  _printSerial = state;
}

void Ux::setPrintSerialInfo(boolean state)
{
  _printSerialInfo = state;
}

void  Ux::setPrintTimers(boolean state)
{
  _printTimers = state;
}

void Ux::setPrintAccs(boolean state)
{
  _printAccs = state;
}

void Ux::setPrintOmegas(boolean state)
{
  _printOmegas = state;
}

void  Ux::setSendBlueAngle(boolean state)
{
  _sendBlueAngle = state;
}

void  Ux::setSerialByteProtocol(boolean state)
{
  _serialByteProtocol = state;
}

void  Ux::setPrintVerboseSerial(boolean state)
{
  _printVerboseSerial = state;
}

void  Ux::setPrintPIDVals(boolean state)
{
  _printPIDVals = state;
}


void Ux::setGyroFilterFlag(volatile boolean filterGyro)
{
 _filterGyro = filterGyro;
}

void Ux::setAccFilterFlag(volatile boolean filterAcc)
{
 _filterAcc = filterAcc; 
}

double Ux::getBaudRate()
{
 return baudrate; 
}

boolean Ux::getGyroFilterFlag()
{
 return _filterGyro; 
}

boolean Ux::getAccFilterFlag()
{
 return _filterAcc; 
}
