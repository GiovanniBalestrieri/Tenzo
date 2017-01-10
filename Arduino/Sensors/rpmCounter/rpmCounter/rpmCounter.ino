#include "header.h"

void setup() {
  Serial.begin(9600);
  Serial.println("Yo");
  
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count, FALLING);
  pinMode(13, OUTPUT);
  
  timeTracker = micros();
}

void loop() {
  serialRoutine();
  computeRevolutions();
}


