#include <Servo.h>
/* Pin: 3,5,22,9*/
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700
#define MOTOR_1 3
#define MOTOR_2 5
#define MOTOR_3 22
#define MOTOR_4 9

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

char readAnswer, readChar, readCh;

void setup() {
  Serial.begin(115200);
  Serial.println("Welcome!");
  calibrate();
  Serial.println("Do you want to calibrate or test? [c/t]");
}

void loop() {  
  SerialRoutine();
}

void SerialRoutine()
{
  if (Serial.available())
  {
      char t = Serial.read();
      if (t == 't')
        test();
      else if (t == 'c')
        calibrate();
  }
}

void calibrate()
{  
  Serial.println("This program will calibrate the ESC.");

  Serial.println("Now writing maximum output.");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  
  // Wait for input
  while (!Serial.available());
  Serial.read();
  motor1.attach(MOTOR_1);
  motor2.attach(MOTOR_2);
  motor3.attach(MOTOR_3);
  motor4.attach(MOTOR_4);
  delay(100);
  
  motor1.writeMicroseconds(MAX_SIGNAL);
  motor2.writeMicroseconds(MAX_SIGNAL);
  motor3.writeMicroseconds(MAX_SIGNAL);
  motor4.writeMicroseconds(MAX_SIGNAL);

  // Send min output
  Serial.println("Sending minimum output. Press key");
  // Wait for input
  while (!Serial.available());
  Serial.read();

  motor1.writeMicroseconds(MIN_SIGNAL);
  motor2.writeMicroseconds(MIN_SIGNAL);
  motor3.writeMicroseconds(MIN_SIGNAL);
  motor4.writeMicroseconds(MIN_SIGNAL);
  Serial.println("Done!");
}

void question()
{
  Serial.println("Want to test it? [y/n]");
  while (!Serial.available());
  readChar = Serial.read();
  // Wait for input
  test();
  Serial.println("Again? [y/n]");
  while (!Serial.available());
  readCh = Serial.read();
  if (readCh == 'y')
    test();
}

void test()
{
  if (readChar == 'y' || readAnswer == 't')
  {  
    Serial.println("Caution! Testing motor!");
    Serial.println("Increasing speed...");
    for (int i = 1000 ;i<1500;i++)
    {
      if (i==700 || i == 1000)
        Serial.println("tick");
      delay(2);
      motor1.writeMicroseconds(i);
      motor2.writeMicroseconds(i);
      motor3.writeMicroseconds(i);
      motor4.writeMicroseconds(i);
    }
    Serial.println("1500 us");
    Serial.println("Decreasing speed ...");
    for (int i = 1500;i<=1000;i--)
    {
      delay(2);
      motor1.writeMicroseconds(i);
      motor2.writeMicroseconds(i);
      motor3.writeMicroseconds(i);
      motor4.writeMicroseconds(i);
    }
    Serial.println("1000 us");
    delay(2000);
    Serial.println("shutting down");
    for (int i = 1000;i<700;i--)
    {
      delay(2);
      motor1.writeMicroseconds(i);
      motor2.writeMicroseconds(i);
      motor3.writeMicroseconds(i);
      motor4.writeMicroseconds(i);
    }
//    for (int i = 600 ;i<1000;i++)
//    {
//      if (i==700 || i == 1000)
//        Serial.println("tick");
//      delay(2);
//      motor.writeMicroseconds(i);
//    }
//    Serial.println("800. ");
//    for (int i = 1000;i<=0;i--)
//    {
//      delay(2);
//      motor.writeMicroseconds(i);
//    }
    Serial.println("Stop.");
  }
  else if (readChar == 'n')
  {  
    Serial.println("Have a good one!");
  }
}
