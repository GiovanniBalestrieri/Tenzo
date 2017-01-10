#include "header.h"
#include <Servo.h> 

Servo myservo;

void setup() {
  Serial.begin(9600);

  myservo.attach(servoPin); 
  myservo.writeMicroseconds(MAX_SIGNAL);
  delay(MAX_SIGNAL);
  myservo.writeMicroseconds(MIN_SIGNAL);
  
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count, FALLING);
  pinMode(13, OUTPUT);
  
  timeTracker = micros();
  Serial.println("Welcome");
}

void loop() {
  serialRoutine();
  computeRevolutions();
  servoRoutine();
}

void servoRoutine() {
  myservo.writeMicroseconds(currentUs);
  if (initialize) {
    for (int i = MIN_SIGNAL; i<=REF_SIGNAL; i++) {
      myservo.writeMicroseconds(i);
      delay(10);
    }
    currentUs = REF_SIGNAL;
    initialized = true;
    initialize = false;
    Serial.println("Initialized");
  }
  
  if (land) {
    for (int i = currentUs; i>MIN_SIGNAL; i--) {
      myservo.writeMicroseconds(i);
      delay(10);
    }
    currentUs = MIN_SIGNAL;
    landed = true;
    land = false;
    Serial.println("Landed");
  }
}


