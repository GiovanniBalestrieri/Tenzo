/*
  Ctx - Context for Tenzo code.
  Created by Giovanni Balestrieri - UserK, August 26, 2015
*/


#ifndef Ctx_h
  #define Ctx_h
  
  #define VERSION 0.02
  
  #define MOTOR_1 3
  #define MOTOR_2 5
  #define MOTOR_3 22
  #define MOTOR_4 9
  
  boolean processing = false;
  boolean printBlue = false;
  boolean printMotorsVals = true;
  boolean printMotorsValsUs = false;
  boolean printPIDVals = false;
  boolean printSerialInfo = true;
  boolean printSerial = true;
  boolean printTimers = false; // true
  boolean printAccs = false;
  boolean printMotorsPid = true;
  boolean printOmegas = false;
  boolean sendBlueAngle = false;
  boolean serialByteProtocol = false;
  boolean printVerboseSerial = true;
  
#endif
