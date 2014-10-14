#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
int val=-1;
boolean cond=false;
String incomingString = "-1";

void setup() 
{ 
  servo1.attach(3);  // attaches the servo on pin 3
  servo2.attach(5);  // attaches the servo on pin 5  
  servo3.attach(6);  // attaches the servo on pin 6 
  servo4.attach(9);
  
  servo1.write(0);  
  servo2.write(0); 
  servo3.write(0); 
  servo4.write(0);
  
  delay(2000);
   // Required for I/O from Serial monitor
  Serial.begin(9600);
  // Print a startup message
  Serial.println("initializing"); 
  delay(350);
  Serial.println("Ready to go bro!");
}

void loop() 
{ 
  if(Serial.available() > 0)
  {
    cond=true;
    // read the value
    char ch = Serial.read();

    if (ch != 's')
    {
      // Print out the value received
      // so that we can see what is
      // happening
      Serial.print("                           I have received: ");
      Serial.print(ch);
      Serial.print('\n');
    
      // Add the character to
      // the incomingString
      incomingString += ch;
      
      //Serial.println(incomingString);
      // Convert the string to an integer
    }
    else
    {
      // print the incoming string
      Serial.println("I am printing the entire string");
      Serial.println(incomingString);
    
      // Convert the string to an integer
      int val = incomingString.toInt();
    
      // print the integer
      Serial.println("Printing the value: ");
      Serial.println(val);
    
      /*
      *  We only want to write an integer between
      *  0 and 180 to the motor. 
      */
      if (val > -1 && val < 181)
     {
       // Print confirmation that the
       // value is between 0 and 180
       Serial.println("Value is between 0 and 180");
       // Write to Servo
       servo1.write(val);
     }
     // The value is not between 0 and 180.
     // We do not want write this value to
     // the motor.
     else
     {
       Serial.println("Value is NOT between 0 and 180");
      
       // IT'S a TRAP!
       Serial.println("Error with the input");
     }
    
      // Reset the value of the incomingString
      incomingString = "";
    }
  }
  
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
}
