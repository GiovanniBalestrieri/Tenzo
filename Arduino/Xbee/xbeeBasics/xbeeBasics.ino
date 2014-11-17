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
long BaudRateXbee = 19200;
char GotChar;
byte getData;

// Compare function variables
char inData[20]; // Allocate some space for the string
char inChar=-1; // Where to store the character read
byte index = 0; // Index into array; where to store the character

// Xbee SoftwareSerial initialization
SoftwareSerial xbee(pinRx, pinTx); // RX, TX

typedef struct mcTag {
       unsigned char srcAddr;
       unsigned char dstAddr;
       unsigned long versionX;
       unsigned char numCmds;
       unsigned char hdrLength;
       unsigned char cmdLength;
       unsigned short totalLen;
       unsigned short crc; } MyControlHdr;

    typedef struct ctrTag {
      unsigned char cmd;
      long param1;
      long param2;
      long param3;
      long param4;   } MyCommand;

    unsigned char buffer[255];  // or worst case message size

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
    Serial.print("Received: ");
    Serial.print(getData);
    Serial.println();
    // send it back to the Xbee
    //xbee.println(getData);
    if(getData == 16)
    {  	 
      byte message[] = {0x11, 0x11};

      xbee.write(message, sizeof(message));
      uint8_t s = 17;
      Serial.println();
      Serial.print(" Sending to Matlab:  ");
      Serial.println();
      Serial.print(message[1]);
      //xbee.write(0x11);
    } 
    else if (getData == 18)
    {
      Serial.println(" Matlab communication established. ");
    }
    else if(getData == 19)
    {  	 
      Serial.println(" [Warning] Matlab communication problem. ");
    }
  }
}
