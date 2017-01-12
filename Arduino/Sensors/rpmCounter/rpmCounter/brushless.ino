void calibrate() {
  myservo.detach();
  
  myservo.attach(servoPin); 
  myservo.writeMicroseconds(MAX_SIGNAL);
  delay(3000);
  Serial.println("...");
  myservo.writeMicroseconds(MIN_SIGNAL);
  currentUs = MIN_SIGNAL;
  delay(3000);
}
void servoRoutine() {
  if (setupOk) {
    myservo.writeMicroseconds(currentUs);
  }
  
  if (initialize) {
    initializing = true;
    for (int i = MIN_SIGNAL; i<=REF_SIGNAL; i++) {
      myservo.writeMicroseconds(i);
      delay(10);
    }
    currentUs = REF_SIGNAL;
    initialized = true;
    initialize = false;
    initializing = false;
    Serial.println("Initialized");
  }
  
  if (land) {
    landing = true;
    for (int i = currentUs; i>MIN_SIGNAL; i--) {
      myservo.writeMicroseconds(i);
      delay(10);
    }
    currentUs = MIN_SIGNAL;
    landed = true;
    land = false;
    landing = false;
    Serial.println("Landed");
  }

  
  if (test && initialized) {
    
    Serial.println("Phase 1");
    for (float i = 0; i<100; i=i+0.1) {
      float val = ((MAX_SIGNAL - MIN_SIGNAL)/2) * sin(i) + MIN_SIGNAL;
      myservo.writeMicroseconds((int) val);
      delay(10);
    }
    
    Serial.println("Phase 2");
      
    for (int i = 0; i<2000; i++) {
      int val = MIN_SIGNAL;
      if (i<500) {
        val = MIN_SIGNAL;
      }
      else if (i<1000) {
        val = REF_SIGNAL;
      }
      else if (i<1500) {
        val = MAX_SIGNAL*0.7;
      }
      else {
        val = MIN_SIGNAL;
      }
      myservo.writeMicroseconds((int) val);
      delay(10);
    }
    currentUs = MIN_SIGNAL;
    test = false;
    Serial.println("Tested");
  }
}

void detachMotors() {
  myservo.detach();
}

