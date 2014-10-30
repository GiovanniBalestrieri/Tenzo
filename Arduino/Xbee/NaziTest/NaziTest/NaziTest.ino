// Wiring
// Connect Dout to the Pin2 (Arduino pin(2) Rx , Dout)
// Connect Din to the Pin4 (Arduino pin(4) Tx , Din)

#include <SoftwareSerial.h>

uint8_t pinRx = 4 , pinTx = 2; // the pin on Arduino
long BaudRate = 9600 , sysTick = 0;
char GotChar, getData;
unsigned long timer = millis();
// Xbee SoftwareSerial initialization
SoftwareSerial xbee(pinRx, pinTx); // RX, TX

void setup() 
{
  Serial.begin(9600);
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
  //pinMode(13, OUTPUT);   	//set pin 13 as output
}

void loop() 
{
  sysTick++ ; // a system timer
  if (Serial.available()) 
  {
    GotChar = Serial.read();
    xbee.print(GotChar);
    Serial.print(GotChar);
  }
  if (xbee.available()>0)
  {  //is there anything to read
    //Serial.println("Ohohoh");
    getData = xbee.read();  //if yes, read it  
    byte dat = xbee.read();
    Serial.print(getData);
    xbee.println(getData);
    xbee.println(dat);
    Serial.println();
    if(getData == 'a')
    {  	 
      Serial.println(" Ambé ");
    }  
    else if(getData == 'b')
    {
      Serial.println(" Bene ");
    }
  }
  
}
