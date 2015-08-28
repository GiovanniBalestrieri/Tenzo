/*
  propulsion.h - Library to handle user interaction.
  Created by Giovanni Balestrieri - UserK, August 26, 2015.
*/

#include "Arduino.h"
#include "Ux.h"
#include "Ctx.h"

Ux::Ux()
{
  // Constructor
  Serial.print("Welcome!! XD ");
}

void Ux::welcome()
{  
  Serial.print("Welcome To Tenzo ");
  Serial.println(VERSION);
  Serial.println("Want to perform a test,start? [t/s]");
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
  int a[4]={MOTOR_1,MOTOR_2,MOTOR_3,MOTOR_4 };
  return a[i];
}


// GEt Methods
boolean Ux::getPrintMotorVals()
{
  return _printMotorsVals;
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

// SET Methods

void Ux::setPrintMotorVals(boolean state)
{
  _printMotorsVals = state;
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
