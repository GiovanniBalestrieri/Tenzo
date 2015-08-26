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

int throttle = 700;

char readAnswer, readChar, readCh;
Propulsion tenzoProp(MOTOR_1,MOTOR_2,MOTOR_3,MOTOR_4);

void setup() {
  Serial.begin(115200);
  tenzoProp.calibrateOnce();
  tenzoProp.init();
  throttle = tenzoProp.getThrottle();
}

void loop() {  
  SerialRoutine();
  tenzoProp.setSpeedWUs(tenzoProp.getThrottle());
}

void welcome()
{  
  Serial.println("Welcome!");
  Serial.println("This tool is required to calibrate your servo before flight.");
  Serial.println("You can communicate with PWM signal with your ESCs, arm them,");
  Serial.println("calibrate them sending a sequence of Max : 2.000 - Min : 700.");
  Serial.println("After the calibration you can perform a test pressing 't' or 'y' when asked.");
  Serial.println("New feature: Test your motors without calibration.");
  Serial.println("Do you want to calibrate or test? [c/t]");
}

void SerialRoutine()
{
  if (Serial.available())
  {
      char t = Serial.read();
      if (t == 't')
        tenzoProp.test();
      else if (t == 'c')
        tenzoProp.calibrateAgain();
      else if (t == 's')
        tenzoProp.stopAll();
      else if (t == 'r')
        tenzoProp.resetMotors();
      else if (t == 'q')
      {
        tenzoProp.setThrottle(tenzoProp.getThrottle() + 10);
        Serial.println(tenzoProp.getThrottle());
      }
      else if (t == 'a')
      {
        tenzoProp.setThrottle(tenzoProp.getThrottle() - 10);
        Serial.println(tenzoProp.getThrottle());
      }
      else if (t == 'v')
      {
        Serial.println(tenzoProp.getThrottle());
      }      
      else if (t == 'x')
      {
        tenzoProp.detachAll();
        Serial.println("Warning Servo detached");
      }
  }
}

