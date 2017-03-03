/*
  Ctx - Context for Tenzo code.
  Created by Giovanni Balestrieri -
  UserK, August 26, 2015
*/


#ifndef Ctx_h
  #define Ctx_h
  
  
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
  volatile boolean filterGyro = true;
  volatile boolean filterAcc = true;
  
  
  
#endif
