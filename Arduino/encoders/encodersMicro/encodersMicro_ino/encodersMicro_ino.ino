#include<SoftwareSerial.h>

#define RESOLUTION 128

int encoderPin1 = 2;
int encoderPin2 = 3;

SoftwareSerial blu(10, 11); // RX, TX 


volatile int lastEncoded = 0;
volatile long encoderValue = 0;

long lastencoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;
double angle;

boolean matlab = false;
boolean requested = false;

void setup() {
  Serial.begin (9600);
  blu.begin(9600);

  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(2, updateEncoder, CHANGE); 
  attachInterrupt(3, updateEncoder, CHANGE);


  
  blu.println("Setup Completed");
  Serial.println("Setup Completed");

}

void loop()
{ 
  convertTicksToAngle();
  serialRoutine();
  bluRoutine();
  delay(1000);
}

void bluRoutine()
{
  if (blu.available()>0)
  {
    Serial.print("Received command via BT:\t");
    char t =  blu.read();
    Serial.println(t);
    if (t=='a')
    {
      requested = true;
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

void convertTicksToAngle()
{
   angle = (float) encoderValue*360/(RESOLUTION*4);  
}

void serialRoutine()
{
 if (Serial.available()>0)
 {
    char t = Serial.read(); 
    if (t=='r')
      encoderValue = 0;
 }
 Serial.print(angle);
 Serial.print("\tValue: encoded: ");
 Serial.println(encoderValue);
 
 
 //if (matlab && requested)
 //{
  //printBluAngles();
 //}
}

void updateEncoder()
{
  int MSB = digitalRead(2); //MSB = most significant bit
  int LSB = digitalRead(3); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
    encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) 
    encoderValue --;

  lastEncoded = encoded; //store this value for next time
}

void printBluAngles()
{
  blu.print("e,");
  blu.print(angle);
  blu.print(",");
  blu.print(encoderValue);
  blu.println(",z");
  requested = false;
}
