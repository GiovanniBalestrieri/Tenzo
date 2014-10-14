// Wiring
// Connect Dout to the Pin2 (Arduino pin(2) Rx , Dout)
// Connect Din to the Pin4 (Arduino pin(4) Tx , Din)

#include <SoftwareSerial.h>

uint8_t pinRx = 15 , pinTx = 14; // the pin on Arduino
long BaudRate = 57600 , sysTick = 0;
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
  while (xbee.available()>0)
  {  //is there anything to read
    Serial.println("Ohohoh");
    getData = xbee.read();  //if yes, read it  
    
    Serial.print(getData);
    //xbee.println(getData);
    Serial.print(getData);
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
