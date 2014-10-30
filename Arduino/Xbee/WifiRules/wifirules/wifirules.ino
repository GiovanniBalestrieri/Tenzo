// Wiring
// Connect Dout to the Pin2 (Arduino pin(2) Rx , Dout)
// Connect Din to the Pin4 (Arduino pin(4) Tx , Din)

#include <SoftwareSerial.h>
#define numberOfDigits 3
char theNumberString[numberOfDigits + 1];
int theNumber;

uint8_t pinRx = 12 , pinTx = 13; // the pin on Arduino
long BaudRate = 9600 , sysTick = 0;
char GotChar, getData;
boolean stringComplete=false;
String inputString = "";

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
  /*if (Comp("motor 1")==0)
  {
    xbee.write("\n Motor 1 selected, insert speed value after the 'R' character:");
    if (stringComplete)
    {
      if(xbee.read() == 'R')
      {
        for (int i = 0; i < numberOfDigits; theNumberString[i++] = Serial.read());
        theNumberString[numberOfDigits] = 0x00;
        theNumber = atoi(theNumberString);
      }
      Serial.println(theNumber);
    }
  }
  
  if (Comp("a1")==0)
  {
    xbee.write("\n Motor 1 selected, insert speed value after the 'R' character:");
    if (stringComplete)
    {
      if(xbee.read() == 'R')
      {
        for (int i = 0; i < numberOfDigits; theNumberString[i++] = Serial.read());
        theNumberString[numberOfDigits] = 0x00;
        theNumber = atoi(theNumberString);
      }
      Serial.println(theNumber);
    }
  }
  */
  if (Serial.available()) 
  {
    GotChar = Serial.read();
    xbee.print(GotChar);
  }
  /*
  if (stringComplete)
  {
    if (inputString == "low")
    Serial.println("LOW");
    else if (inputString == "high")
    Serial.println("HIGH");   
    stringComplete = false;
    /*
    if (inputString[0]=='A'&&inputString[4]=='A')
    {
      int val=atoi(inputString[1]+inputString[2]+inputString[3])
      Serial.println("value = ");
      Serial.print(val);
    }
    if (inputString[0]=='A'&&inputString[3]=='A')
    {
      int val=atoi(inputString[1]+inputString[2])
      Serial.println("value = ");
      Serial.print(val);
    }
    */
  //}
  
  while(xbee.available())
  {  //is there anything to read
    getData = xbee.read();  //if yes, read it  
    //xbee.write( (char) getData);
    xbee.println(getData);
    //xbee.print("another?\n");
    Serial.print(getData);
    //Serial.println();
    if(getData == 'a')
    {  	 
      digitalWrite(13, HIGH);
      
    }  
    else if(getData == 'b')
    {
      digitalWrite(13, LOW);
    }
  }
}

void serialEvent() 
{
  while (xbee.available()) 
  {
    // get the new byte:
    char inChar = (char) xbee.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') 
    {
      stringComplete = true;
      //Serial.println("done");
    } 
  }
}

char Comp(char* This) {
    while (xbee.available() > 0) 
    {
      if(index < 19) // One less than the size of the array
        {
            inChar = xbee.read(); // Read a character
            inData[index] = inChar; // Store it
            index++; // Increment where to write next
            inData[index] = '\0'; // Null terminate the string
        }
    }

    if (strcmp(inData,This)  == 0) {
        for (int i=0;i<19;i++) {
            inData[i]=0;
        }
        index=0;
        return(0);
    }
    else {
        return(1);
    }
}
