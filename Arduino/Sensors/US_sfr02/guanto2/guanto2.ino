#include <Wire.h>
#include <Servo.h>

int reading = 0, readingM1 = 0;

const int FILTERING = 1;

int pushPin = A0;
int pushValue;
int pushTreshold = 30;
bool statoPush, stato;

float reads[10];
double sonarTimer = 0;
// #TUNE
int thresholdMax = 110;
int thresholdMin = 5;

long int contSonarRoutine = 0;
long int maxsonarTimer = 0;
double sonarTimeTot = 0;
boolean modeD = false, coro = false, cora = false;

//Filter #TUNE
float alpha = 0.5;

byte lowB, highB;
int us;
//Servo viber;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("START");
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
    usRoutine();
    viberRoutine();
    SerialRoutine();
  }else if(stato==0){
    Serial.println("----------OFF");
    analogWrite(9, 0);
  }
  delay(100);
}


void readVelostat() {

  pushValue = analogRead(pushPin);
  //Serial.println(pushValue);

  if (pushValue < pushTreshold) {
    statoPush = 1;
    delay(50);
  }
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

void viberRoutine()
{
  //int us = analogRead(0);
  //us = K*1/reading;
  if (modeD)
  {
    if (reading < thresholdMin)
    {
      analogWrite(9, 255);
      coro = true;
    }
    else
      analogWrite(9, 0);
  }
  else
  {
    if (reading < thresholdMin) {
      analogWrite(9, 0);
    } else if (reading > thresholdMax) {
      analogWrite(9, 0);
    } else {
      us = map(reading, thresholdMax, thresholdMin, 100, 255);
      analogWrite(9, us);
    }
  }
  readingM1 = reading;
}

void usRoutine()
{
  Wire.beginTransmission(112);
  Wire.write(byte(0x00));
  Wire.write(byte(0x51));

  Wire.endTransmission();
  delay(15);

  Wire.beginTransmission(112);
  Wire.write(byte(0x02));
  Wire.endTransmission();
  sonarTimer = micros();
  Wire.requestFrom(112, 2);

  if (2 <= Wire.available()) {
    reading = Wire.read();
    reading = reading << 8;
    reading |= Wire.read();
  }

  if (FILTERING)
    filter();

  contSonarRoutine++;
  sonarTimer = micros() - sonarTimer;
  if (maxsonarTimer <= sonarTimer)
    maxsonarTimer = sonarTimer;
  sonarTimeTot = sonarTimeTot + sonarTimer;
  //Serial.print(sonarTimer);   // print the reading
}

void filter()
{
  reading = (1 - alpha) * readingM1 + alpha * reading;
}
