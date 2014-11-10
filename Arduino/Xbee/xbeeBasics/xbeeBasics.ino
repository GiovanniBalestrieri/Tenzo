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
long BaudRateSerial = 9600;
long BaudRateXbee = 57600;
char GotChar;
byte getData;

// Compare function variables
char inData[20]; // Allocate some space for the string
char inChar=-1; // Where to store the character read
byte index = 0; // Index into array; where to store the character

// Xbee SoftwareSerial initialization
SoftwareSerial xbee(pinRx, pinTx); // RX, TX

void setup() {
  Serial.begin(BaudRateSerial);
  Serial.println( "Welcome to the XBee Communication Test" );
  Serial.print("BaudRate:");
  Serial.println(BaudRateXbee);
  Serial.print(" Rx Pin#");
  Serial.println(pinRx,DEC);
  Serial.print(" Tx Pin#");
  Serial.println(pinTx,DEC);
  // set the data rate for the SoftwareSerial port
  xbee.begin( BaudRateXbee );
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
    
    Serial.println();
    Serial.println(getData);
    // send it back to the Xbee
    //xbee.println(getData);
    if(getData == 16)
    {  	 
      int intValue = 17;
      //xbee.print(intValue, HEX);   
      //xbee.print(xbee, BIN);
      xbee.write(0x11);
      Serial.println(" Sending to Matlab:  0x11");
      //Serial.write(0x34);
    } 
    else if (getData == 18)
    {
      Serial.println(" Matlab communication established. ");
    }
    else if(getData == 19)
    {  	 
      Serial.println(" [Warning] Matlab communication problem. ");
    }
    
    /*if(getData == 'T')
    {  	 
      xbee.write(0x34);
      Serial.println(" Sending to Matlab:  0x34");
      //Serial.write(0x34);
    } 
    else if(getData == 'Y')
    {  	 
      Serial.println(" Matlab communication established. ");
    } 
    else if(getData == 'N')
    {  	 
      Serial.println(" [Warning] Matlab communication problem. ");
    } */
    // print to serial monitor for visual feedback
  }
}
