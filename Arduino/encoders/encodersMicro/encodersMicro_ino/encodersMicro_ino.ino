#include<SoftwareSerial.h>

#define RESOLUTION 128

int encoderPin1 = 0;
int encoderPin2 = 1;

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

volatile  int MSB = 0;
volatile int LSB = 0;
volatile double angle;

boolean matlab = false;
boolean requestedSerial = false;
boolean requestedBlu = false;
boolean identifying = false;
unsigned long timer = 0, start;

void setup() {
  Serial.begin (115200);
  //blu.begin(9600);

  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(encoderPin1, updateEncoder, CHANGE); 
  attachInterrupt(encoderPin2, updateEncoder, CHANGE);

  //blu.println("Setup Completed");
  Serial.println("Setup Completed");
}

void loop()
{ 
  //convertTicksToAngle();
  serialRoutine();
  //bluRoutine();
  
  if (identifying){
    printAngles();
    delay(9);
  }  
}

/*
// To communicate with the module you need to send a 
// connection request using 'c'. It will answer with K
void bluRoutine()
{
  if (blu.available()>0)
  {
    Serial.print("Received command via BT:\t");
    char t =  blu.read();
    Serial.println(t);
    if (t=='a')
    {
      requestedBlu = true;
      if (matlab)
        printBluAngles();
    }
    else if (t=='c')
    {
      // Connection request from CPanel
      Serial.println("CPanel connection...");
      blu.print("K");
      Serial.println("ACK sent: 'K'");
      matlab = true;
      Serial.println("Matlba = True");
    }
    else if (t=='X')
    {
      // Connection request from CPanel
      Serial.println("CPanel Disconnect...");
      blu.print("X");
      Serial.println("ACK sent: 'X'");
      matlab = false;
      Serial.println("Matlba = False");
    }
    else if (t=='r')
      encoderValue = 0;
  }
}

*/

void convertTicksToAngle()
{
   angle = (float) encoderValue*360/(RESOLUTION*4);  
}

void serialRoutine()
{
 if (Serial.available()>0)
 {
    char t = Serial.read(); 
    if (t=='a')
    {
      requestedSerial = true;
      //if (matlab)
        printAngles();
    }
    else if (t=='c')
    {
      // Connection request from CPanel
      Serial.println("K");
      matlab = true;
      encoderValue = 0;
    }
    
    else if (t=='1')
    {
      // Starting identification data acquisition     
      identifying = true;
      start = millis();
    }
    else if (t=='2')
    {
      // Starting identification data acquisition     
      identifying = false;
    }
    else if (t=='X')
    {
      // Disconnection request from CPanel
      Serial.println("X");
      matlab = false;
    }
    else if (t=='r')
      encoderValue = 0;
 }
}

void updateEncoder()
{
  MSB = digitalRead(2); //MSB = most significant bit
  LSB = digitalRead(3); //LSB = least significant bit
  
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
    encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) 
    encoderValue --;

  lastEncoded = encoded; //store this value for next time

  convertTicksToAngle();
}

/*
void printBluAngles()
{
  blu.print("e,");
  blu.print(angle);
  blu.print(",");
  blu.print(encoderValue);
  blu.println(",z");
  requestedBlu = false;
}
*/

void printAngles()
{
  timer = millis() - start;
  Serial.print("e,");
  Serial.print(angle);
  
  /*
  Serial.print(",");
  Serial.print(encoderValue);
  */
  Serial.print(",");
  Serial.print(timer);
  Serial.println(",z");
  requestedSerial = false;
}
