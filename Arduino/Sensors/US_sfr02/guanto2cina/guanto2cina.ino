#include <Wire.h>
#include <Servo.h>
#include "handy.h"

#define trigPin 7 
#define echoPin 8

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  Serial.println("START");
  pinMode( trigPin, OUTPUT );
  pinMode( echoPin, INPUT );
  pinMode(pushPin, INPUT_PULLUP);

}

void loop()
{
  readVelostat();
 
  if(statoPush==1){
    stato= !stato;
    statoPush=0;
  }
  stato = 1;
  if(stato==1){
    Serial.println("----------ON");
    usRoutineCina();
    viberRoutine();
    SerialRoutine();
  }else if(stato==0){
    Serial.println("----------OFF");
    analogWrite(9, 0);
  }
  delay(100);
}



void SerialRoutine()
{
  Serial.print(reading);
  Serial.print("\t\t");
  Serial.println(us);

  if (coro)
  {
    Serial.print("us: ");
    Serial.println(us);
    coro = false;
  }
}
