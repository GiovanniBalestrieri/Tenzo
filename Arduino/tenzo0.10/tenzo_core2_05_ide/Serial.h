/*
  Serial - Serial context for Tenzo.
  Created by Giovanni Balestrieri
  UserK, June 1, 2016
  www.userk.co.uk
*/

#ifndef SERIAL_H
  #define SERIAL_H


//const double baudRate = 57600;
const double baudRate = 115200;

byte mode;
char readAnswer, readChar, readCh;

byte modeS; 
 
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

#endif
