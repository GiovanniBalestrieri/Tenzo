#include <Servo.h>
#include "Propulsion.h"

/* Pin: 3,5,22,9*/
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700
#define IDLE_THRESHOLD 790

#define MOTOR_1 3
#define MOTOR_2 5
#define MOTOR_3 22
#define MOTOR_4 9

//Servo motor1;
//Servo motor2;
//Servo motor3;
//Servo motor4;

int throttle = 700;

char readAnswer, readChar, readCh;
Propulsion tenzoProp(MOTOR_1,MOTOR_2,MOTOR_3,MOTOR_4);

void setup() {
  Serial.begin(115200);
  tenzoProp.init();
  Serial.println("Welcome!");
  Serial.println("This tool is required to calibrate your servo before flight.");
  Serial.println("You can communicate with PWM signal with your ESCs, arm them,");
  Serial.println("calibrate them sending a sequence of Max : 2.000 - Min : 700.");
  Serial.println("After the calibration you can perform a test pressing 't' or 'y' when asked.");
  Serial.println("New feature: Test your motors without calibration.");
  tenzoProp.calibrateOnce();
  Serial.println("Do you want to calibrate or test? [c/t]");
}

void loop() {  
  SerialRoutine();
  motors(throttle);
}

void SerialRoutine()
{
  if (Serial.available())
  {
      char t = Serial.read();
      if (t == 't')
        tenzoProp.test();
      else if (t == 'c')
        tenzoProp.calibrateOnce();
      else if (t == 's')
        tenzoProp.stopAll();
      else if (t == 'r')
        tenzoProp.resetMotors();
      else if (t == 'q')
      {
        throttle = throttle + 10;
        Serial.println(throttle);
      }
      else if (t == 'a')
      {
        throttle = throttle - 10;
        Serial.println(throttle);
      }
      else if (t == 'v')
      {
        Serial.println(throttle);
      }
  }
}

void calibrate()
{  
  Serial.println("This program will Arm and Calibrate the ESC.");

  Serial.println("Now writing maximum output.");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  
  // Wait for input
  //while (!Serial.available());
  
  /*
  
  //Serial.read();
  motor1.attach(MOTOR_1);
  motor2.attach(MOTOR_2);
  motor3.attach(MOTOR_3);
  motor4.attach(MOTOR_4);
  delay(500);
  motor1.writeMicroseconds(MAX_SIGNAL);
  motor2.writeMicroseconds(MAX_SIGNAL);
  motor3.writeMicroseconds(MAX_SIGNAL);
  motor4.writeMicroseconds(MAX_SIGNAL);

  // Send min output
  Serial.println("Sending minimum output.");
//  // Wait for input
//  while (!Serial.available());
//  Serial.read();
  
  delay(2000);
  motor1.writeMicroseconds(MIN_SIGNAL);
  motor2.writeMicroseconds(MIN_SIGNAL);
  motor3.writeMicroseconds(MIN_SIGNAL);
  motor4.writeMicroseconds(MIN_SIGNAL);
  
  Serial.println("Done!");
  throttle = IDLE_THRESHOLD;
  
  */
}

void resetMotors()
{
  Serial.println("This program will calibrate the ESC.");

  Serial.println("Now writing maximum output.");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  
  // Wait for input
  //while (!Serial.available());
  /*
  //Serial.read();
  motor1.attach(MOTOR_1);
  motor2.attach(MOTOR_2);
  motor3.attach(MOTOR_3);
  motor4.attach(MOTOR_4);
  
  motor1.writeMicroseconds(MIN_SIGNAL);
  motor2.writeMicroseconds(MIN_SIGNAL);
  motor3.writeMicroseconds(MIN_SIGNAL);
  motor4.writeMicroseconds(MIN_SIGNAL);
  */
  Serial.println("Armed!");
}

void test()
{
  /*
    Serial.println("Testing motor!");
    Serial.println("They should start spinning.");
    for (int i = 700; i < 1500; i++)
    {
      if (i==700 || i == 1000)
        Serial.println("tick");
      delay(2);
      motor1.writeMicroseconds(i);
      motor2.writeMicroseconds(i);
      motor3.writeMicroseconds(i);
      motor4.writeMicroseconds(i);
    }
    delay(1000);
    Serial.println("Now, decreasing");
    for (int i = 1500;i<=1000;i--)
    {
      delay(2);
      motor1.writeMicroseconds(i);
      motor2.writeMicroseconds(i);
      motor3.writeMicroseconds(i);
      motor4.writeMicroseconds(i);
    }
    Serial.println("Like that!");
    delay(2000);
    Serial.println("And stop.");
    for (int i = 1000;i<=700;i--)
    {
      delay(2);
      motor1.writeMicroseconds(i);
      motor2.writeMicroseconds(i);
      motor3.writeMicroseconds(i);
      motor4.writeMicroseconds(i);
    }
    
    motor1.writeMicroseconds(700);
    motor2.writeMicroseconds(700);
    motor3.writeMicroseconds(700);
    motor4.writeMicroseconds(700);
    */
}
void motors(int thr)
{
  /*
  motor1.writeMicroseconds(thr);
  motor2.writeMicroseconds(thr);
  motor3.writeMicroseconds(thr);
  motor4.writeMicroseconds(thr);
  */
}
void stopAll()
{
  /*
  Serial.println("Arresto forzato.");
  motor1.writeMicroseconds(700);
  motor2.writeMicroseconds(700);
  motor3.writeMicroseconds(700);
  motor4.writeMicroseconds(700);
  */
}

