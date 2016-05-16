/*
  Ctx - Context for Tenzo code.
  Created by Giovanni Balestrieri -
  UserK, August 26, 2015
*/


#ifndef Ctx_h
  #define Ctx_h
  
  #define VERSION 0.13  
  #define MOTOR_1 3
  #define MOTOR_2 5
  #define MOTOR_3 22
  #define MOTOR_4 9
  
  boolean printSerial = true;
  boolean processing = false;
  boolean printBlue = false;
  boolean printMotorsVals = false;
  boolean printMotorsValsUs = false;
  boolean printPIDVals = false;
  boolean printSerialInfo = false;
  boolean printTimers = true; 
  boolean printAccs = false;
  boolean printMotorsPid = false;
  boolean printOmegas = false;
  boolean sendBlueAngle = false;
  boolean serialByteProtocol = false;
  boolean printVerboseSerial = true;
  
  
  // Filters
  boolean removeSpikesNumerically = false;
  // ISR
  volatile boolean filterGyro = true;
  volatile boolean filterAcc = true;
  
  //const double baudRate = 57600;
  const double baudRate = 115200;
  
#endif
