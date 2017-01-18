void calibrate() {
  detachMotors();  
  delay(500);
  Serial.println("Calibrating");
  myservo.attach(servoPin);
  myservo.writeMicroseconds(MAX_SIGNAL);
  delay(2000);
  Serial.println("...");
  myservo.writeMicroseconds(MIN_SIGNAL);
  currentUs = MIN_SIGNAL;
  Serial.println("safe");
  delay(1000);
}

void detachMotors() {
  myservo.detach();
}

