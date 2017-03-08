


void computeSignal() {

  signalComputeTimer = micros();
  
  if (test) {

    
    if (!initialized){
      initialize();
    }
    testing = true;    
    if (firstTest) {
      Serial.println("start");
      // TODO remove
      delay(2000);
      start = millis();
      firstTest = false;
    }    
    signalTimer = millis() - start;

/*
    if (signalTimer>= 500 && signalTimer < 5000) {
       if (signalTimer<600) {
        currentUs = MIN_SIGNAL_DELTA;
      } else if (signalTimer<2000) {
        currentUs = MIN_SIGNAL_DELTA*1.2;
      } else if (signalTimer<2500) {
        currentUs = MIN_SIGNAL_DELTA*1.3;
      } else if (signalTimer<3500) {
        currentUs = MIN_SIGNAL_DELTA*1.4;
      } else {
        currentUs = MIN_SIGNAL*1.2;
      }
    } else */
    float boom = 0.35;
    if (signalTimer>= 3000 && signalTimer <= 10000) {
      if (signalTimer<4000) {
        // [ 3s , 4s]        
        currentDelta = (MAX_SIGNAL_DELTA -MIN_SIGNAL_DELTA)*boom;
      } else if (signalTimer<5000 && signalTimer >= 4000) {
        currentDelta = MIN_SIGNAL_DELTA;
      } if (signalTimer<6000 && signalTimer >= 5500)  {        
        currentDelta = -(MAX_SIGNAL_DELTA -MIN_SIGNAL_DELTA)*boom/3;
      } else if (signalTimer<10000 && signalTimer >= 6000) {
        currentDelta = MIN_SIGNAL_DELTA;
      } 
    } else if (signalTimer<30000 && signalTimer >= 10000) {
        currentDelta = (MAX_SIGNAL_DELTA - MIN_SIGNAL_DELTA)*0.35/2 * sin(0.3*signalTimer*3.1415/180)  + (MAX_SIGNAL_DELTA - MIN_SIGNAL_DELTA)*0.35/2;
    } else if (signalTimer < 2000) {
        currentDelta = MIN_SIGNAL_DELTA;
    }
      
    if (currentDelta > MAX_SIGNAL_DELTA) {
        currentDelta = MAX_SIGNAL_DELTA;
    }
    
    if (currentDelta < -MAX_SIGNAL_DELTA) {
        currentDelta = -MAX_SIGNAL_DELTA;
    }

    if (signalTimer >= 30000) {
      currentDelta = 0;
      //Serial.println("Tested");
      test = false;
      firstTest = true;
      
      Serial.print("y,");
      Serial.print(signalTimer);
      Serial.print(",");
      Serial.print(tenzoProp.getThrottle());
      Serial.print(",");
      Serial.print(currentDelta);  
      Serial.print(",");
      Serial.print(estXAngle);  
      Serial.println(",S,z");
      
      signalTimer = 0;
      //land();
    }

    if (test) {
      /*
      Serial.print("y,");
      Serial.print(signalTimer);
      Serial.print(",");
      Serial.print(tenzoProp.getThrottle());
      Serial.print(",");
      Serial.print(currentDelta);  
      Serial.print(",");
      Serial.print(estXAngle);  
      // Added status pre temrinator
      Serial.println(",N,z");
      */
    
      currentDelta = (int) currentDelta;  
      tenzoProp.setSpeeds(tenzoProp.getThrottle(), 0, currentDelta, 0, 0);
    
    }  

    
    
  }
  contSignalRoutine++;
  signalComputeTimer = micros() - signalComputeTimer;
  
    if (maxsignalTimer <= signalComputeTimer)
      maxsignalTimer = signalComputeTimer;
  
  signalTimeTot = signalTimeTot + signalComputeTimer;
  
}

