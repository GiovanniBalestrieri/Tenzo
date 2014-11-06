/**
 *  Wiring on Arduino UNO
 *  Connect Dout to Pin 2 (Arduino pin(2) Rx , Dout)
 *  Connect Din to Pin 4 (Arduino pin(4) Tx , Din)
 *  Wiring on Arduino Mega 
 *  Connect Dout to Pin 12 (Arduino pin(12) Rx , Dout)
 *  Connect Din to Pin 13 (Arduino pin(13) Tx , Din)
 **/

#include <SoftwareSerial.h>

uint8_t pinRx = 12 , pinTx = 13; // the pin on Arduino
long BaudRate = 9600;
char GotChar, getData;

// Compare function variables
char inData[20]; // Allocate some space for the string
char inChar=-1; // Where to store the character read
byte index = 0; // Index into array; where to store the character

// Xbee SoftwareSerial initialization
SoftwareSerial xbee(pinRx, pinTx); // RX, TX

void setup() {
  Serial.begin(BaudRate);
  Serial.println( "Welcome to the XBee Communication Test" );
  Serial.print("BaudRate:");
  Serial.println(BaudRate);
  Serial.print(" Rx Pin#");
  Serial.println(pinRx,DEC);
  Serial.print(" Tx Pin#");
  Serial.println(pinTx,DEC);
  // set the data rate for the SoftwareSerial port
  xbee.begin( BaudRate );
  xbee.println("Setup Completed!");
}

void loop() 
{
  // Reads from serial monitor 
  if (Serial.available()) 
  {
    GotChar = Serial.read();
    // sends char to xbee
    xbee.print(GotChar);
  }
  
  while(xbee.available())
  {  
    getData = xbee.read();  
    
    // send it back to the Xbee
    xbee.println(getData);
    
    // print to serial monitor for visual feedback
    Serial.print(getData);
  }
}
