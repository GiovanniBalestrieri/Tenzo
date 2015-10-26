/*
  Ctx - Context for Tenzo code.
  Created by Giovanni Balestrieri - UserK, August 26, 2015
*/


#ifndef Ctx_h
  #define Ctx_h
  
  #define VERSION 0.11  
  #define MOTOR_1 3
  #define MOTOR_2 5
  #define MOTOR_3 22
  #define MOTOR_4 9
  
  boolean processing = true;
  boolean printBlue = false;
  boolean printMotorsVals = false;
  boolean printMotorsValsUs = false;
  boolean printPIDVals = false;
  boolean printSerialInfo = false;
  boolean printSerial = true;
  boolean printTimers = false; 
  boolean printAccs = true;
  boolean printMotorsPid = false;
  boolean printOmegas = false;
  boolean sendBlueAngle = false;
  boolean serialByteProtocol = false;
  boolean printVerboseSerial = true;
  
  // Filters
  boolean removeSpikesNumerically = false;
  
  //const double baudRate = 57600;
  const double baudRate = 115200;
  
#endif
