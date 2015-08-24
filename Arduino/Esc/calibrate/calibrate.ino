#include <Servo.h>
/* Pin: 3,5,22,9*/
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 700
#define MOTOR_PIN 5

Servo motor;
char readChar, readCh;

void setup() {
  Serial.begin(9600);
  Serial.println("Program begin...");
  Serial.println("This program will calibrate the ESC.");

  Serial.println("Now writing maximum output.");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  
  // Wait for input
  while (!Serial.available());
  Serial.read();

  motor.attach(MOTOR_PIN);
  delay(100);
  motor.writeMicroseconds(MAX_SIGNAL);

  // Send min output
  Serial.println("Sending minimum output. Press key");
  // Wait for input
  while (!Serial.available());
  Serial.read();
;
  motor.writeMicroseconds(MIN_SIGNAL);
  Serial.println("Done!");
  
  Serial.println("Want to test it? [y/n]");
  while (!Serial.available());
  readChar = Serial.read();
  // Wait for input
  encore();
  Serial.println("Again? [y/n]");
  while (!Serial.available());
  readCh = Serial.read();
  if (readCh == 'y')
    encore();
}

void loop() {  

}

void encore()
{
  if (readChar == 'y')
  {  
    Serial.println("Caution! Testing motor!");
    Serial.println("Increasing speed...");
    for (int i = 0;i<1500;i++)
    {
      if (i==700 || i == 1000)
        Serial.println("tick");
      delay(2);
      motor.writeMicroseconds(i);
    }
    Serial.println("Decreasing speed ...");
    for (int i = 1200;i<700;i--)
    {
      delay(2);
      motor.writeMicroseconds(i);
    }
    Serial.println("Stop.");
    motor.writeMicroseconds(0);
  }
  else if (readChar == 'n')
  {  
    Serial.println("Have a good one!");
  }
}
