#include <Servo.h>

// Wiring
// Connect Dout to the Pin2 (Arduino pin(2) Rx , Dout)
// Connect Din to the Pin4 (Arduino pin(4) Tx , Din)
// Esc 1 pin 3, Esc2 - pin5, Esc 3 - pin 6, Esc 4 - pin 9

#include <SoftwareSerial.h>
#define numberOfDigits 3
char theNumberString[numberOfDigits + 1];
int theNumber,omega=0, thresholdUp=255, thresholdDown=1;

uint8_t pinRx = 2 , pinTx = 4; // the pin on Arduino
long BaudRate = 57600 , sysTick = 0;
char GotChar, getData;
boolean stringComplete=false;
String inputString = "";

// Compare function variables
char inData[20]; // Allocate some space for the string
char inChar=-1; // Where to store the character read
byte index = 0; // Index into array; where to store the character

// Xbee SoftwareSerial initialization
SoftwareSerial xbee(pinRx, pinTx); // RX, TX

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
int val=-1;
boolean cond=false, manual=false, test=false, pid=false;
String incomingString = "-1";

void setup() 
{ 
  // Xbee communication
  
  Serial.begin(BaudRate);
  Serial.println( "Welcome on board." );
  Serial.print("BaudRate:");
  Serial.println(BaudRate);
  Serial.print(" Rx Pin#");
  Serial.println(pinRx,DEC);
  Serial.print(" Tx Pin#");
  Serial.println(pinTx,DEC);
  // set the data rate for the SoftwareSerial port
  xbee.begin( BaudRate );
  xbee.println("Setup Completed. Welcome on board.");
  
  // Motor initialization
  servo1.attach(3);  // attaches the servo on pin 3
  servo2.attach(5);  // attaches the servo on pin 5  
  servo3.attach(6);  // attaches the servo on pin 6 
  servo4.attach(9);
  
  servo1.write(0);  
  servo2.write(0); 
  servo3.write(0); 
  servo4.write(0);
  
  delay(2000);
}

void loop() 
{ 
  while(xbee.available())
  {
    cond=true;
    // there is something to read
    getData = xbee.read();
    if (getData == 'p')
    {
      pid = true;
    }
    if (getData == 'w')
    {
     if (omega<thresholdUp)
       {
         omega++;
       }
    }
    else if (getData == 's')
    {
      if (omega>thresholdDown)
      {
        omega--;
      }
    }
    else if(getData == 'r')
    {
      omega=1;
    }
    else if (getData== '1')
    {
      testMotor(1);
    }
    else if (getData== '2')
    {
      testMotor(2);
    }
    else if (getData== '3')
    {
      testMotor(3);
    }
    else if (getData== '4')
    {
      testMotor(4);
    }
    motorSpeed(omega);
  }
  /*
  if (cond)
  Serial.println("COND");
  int val = incomingString.toInt();
  Serial.println("");
  
  Serial.print("     ");
  Serial.print(val);
  
  Serial.println("");
  if (val>-1 && val<=255)
  {
    
    Serial.println("AAA");
    
    Serial.println(val);
//    println("Motor reference value= ");   
//    print(val); 
//    servo1.write(val);      
//    servo2.write(val); 
//    servo3.write(val); 
//    servo4.write(val); 
  }
  incomingString="-1";
  cond=false;
  */
}

void motorSpeed(int x)
{   
    xbee.println(x); 
    servo1.write(x);      
    servo2.write(x); 
    servo3.write(x); 
    servo4.write(x); 
}

void testMotor(int x)
{   
    xbee.print(" Testing motor: ");
    xbee.print(x);
    if (x==1)
    {
      xbee.print(" Increasing angular velocity");
      servo1.write(40);
      delay(500);
      servo1.write(60);
      delay(500);
      servo1.write(80);
      xbee.print(" Decreasing angular velocity");
      delay(500);
      servo1.write(40);
      delay(500);
      servo1.write(0);
    }
    else if (x==2)
    {
      xbee.print(" Increasing angular velocity");
      servo2.write(40);
      delay(500);
      servo2.write(60);
      delay(500);
      servo2.write(80);
      xbee.print(" Decreasing angular velocity");
      delay(500);
      servo2.write(40);
      delay(500);
      servo2.write(0);
    }
    else if (x==3)
    {
      xbee.print(" Increasing angular velocity");
      servo3.write(40);
      delay(500);
      servo3.write(60);
      delay(500);
      servo3.write(80);
      xbee.print(" Decreasing angular velocity");
      delay(500);
      servo3.write(40);
      delay(500);
      servo3.write(0);
    }
    else if (x==4)
    {
      xbee.print(" Increasing angular velocity");
      servo4.write(40);
      delay(500);
      servo4.write(100);
      delay(500);
      servo4.write(150);
      xbee.print(" Decreasing angular velocity");
      delay(500);
      servo4.write(40);
      delay(500);
      servo4.write(0);
    }
}
