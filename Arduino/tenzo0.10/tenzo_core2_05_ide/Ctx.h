/*
  Ctx - Context for Tenzo code.
  Created by Giovanni Balestrieri -
  UserK, August 26, 2015
*/


#ifndef Ctx_h
  #define Ctx_h

  int ACC_X_AXIS_PIN = 0;
  int ACC_Y_AXIS_PIN = 1;
  int ACC_Z_AXIS_PIN = 2;
  int gyroBiasSamples = 500;
  int gyroSensibility = 2000; // 500 // 250 [deg/s]

  float alphaA= 0.97, alphaW = 0.8;  

  const float pi = 3.1415;


  unsigned long timerInertial1= 0;
  unsigned long timerInertial2= 0;
  
  boolean printSerial = true;
  boolean processing = false;
  boolean printBlue = false;
  boolean printMotorsVals = false;
  boolean printMotorsValsUs = false;
  boolean printPIDVals = false;
  boolean printSerialInfo = false;
  boolean printTimers = false; 
  boolean printAccs = false;
  boolean printMotorsPid = false;
  boolean printOmegas = false;
  boolean sendBlueAngle = false;
  boolean serialByteProtocol = false;
  boolean printVerboseSerial = true;
  
  
  // Filters
  boolean removeSpikesNumerically = false;
  // ISR
  boolean filterGyro = false;
  boolean filterAcc = true;
  
  
  
#endif
