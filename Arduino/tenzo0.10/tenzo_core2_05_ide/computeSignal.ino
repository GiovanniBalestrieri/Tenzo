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
      //delay(2000);
      start = millis();
      firstTest = false;
    }    
    signalTimer = millis() - start;

    float boom = 0.25;
    float boom2 = 0.65;
    if (mode3) {
        if (signalTimer>= 3000 && signalTimer <= 20000) {
          if (signalTimer<4000) {
            // [ 3s , 4s]        
            currentDelta = (MAX_SIGNAL_DELTA -MIN_SIGNAL_DELTA)*boom/2;
          } else if (signalTimer<6000 && signalTimer >= 4000) {
            // [ 4s , 6s]        PAUSA 
            currentDelta = MIN_SIGNAL_DELTA;
          } else if (signalTimer<6500 && signalTimer >= 6000) {
            // [ 6 , 6.5]        
            currentDelta = (MAX_SIGNAL_DELTA -MIN_SIGNAL_DELTA)*boom/2;
          } if (signalTimer<8000 && signalTimer >= 6500)  {        
            // [ 6.5 , 8]  PAUSA 
            currentDelta = MIN_SIGNAL_DELTA;
          }  else if (signalTimer<9000 && signalTimer >= 8000) {
            // [8 , 9]        
            currentDelta = -(MAX_SIGNAL_DELTA -MIN_SIGNAL_DELTA)*boom/2;
          } if (signalTimer<11000 && signalTimer >= 9000)  {        
            // [9 , 11]      PAUSA 
            currentDelta = MIN_SIGNAL_DELTA;
          }  else if (signalTimer<11500 && signalTimer >= 11000) {
            currentDelta = -(MAX_SIGNAL_DELTA -MIN_SIGNAL_DELTA)*boom/2;
          } else if (signalTimer<14000 && signalTimer >= 11500)  {        
            // [9 , 11]      PAUSA 
            currentDelta = MIN_SIGNAL_DELTA;
          } else if (signalTimer<15000 && signalTimer >= 14000) {
            currentDelta = -(MAX_SIGNAL_DELTA -MIN_SIGNAL_DELTA)*boom/2;
          } else if (signalTimer<20000 && signalTimer >= 15000)  {        
            // [9 , 11]      PAUSA 
            currentDelta = MIN_SIGNAL_DELTA;
          } 
        }/* else if (signalTimer<30000 && signalTimer >= 20001) {
            currentDelta = (MAX_SIGNAL_DELTA - MIN_SIGNAL_DELTA)*boom2 * sin(0.3*signalTimer*3.1415/180)  + (MAX_SIGNAL_DELTA - MIN_SIGNAL_DELTA)*boom/2;
        } */else if (signalTimer < 2000) {
            currentDelta = MIN_SIGNAL_DELTA;
        }

    } else if (mode1) {
        if (signalTimer>= 3000 && signalTimer <= 20000) {
          if (signalTimer<8000 && signalTimer >= 7000) {
            // [ 7s , 8s]        
            currentDelta = (MAX_SIGNAL_DELTA -MIN_SIGNAL_DELTA)*boom/2;
          } else {
            // PAUSA 
            currentDelta = MIN_SIGNAL_DELTA;
          }      
      }
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
      Serial.print(currentDelta);  
      Serial.print(",");
      Serial.print(estXAngle);  
      Serial.print(",");
      Serial.print(angleEnc);
      Serial.println(",S,z");
      
      signalTimer = 0;
      land();
    }

    if (signalTimer>= 6900 && signalTimer <= 9900) {
      // Save samples 
      timeSamples[counterSamples] = signalTimer;
      deltaSamples[counterSamples] = currentDelta  ;
      //estAngleSamples[counterSamples] = angles[0];
      angleSamples[counterSamples] = (int) angleEnc*100;
      if (counterSamples <= maxSamples)
        counterSamples++;
    }

    currentDelta = (int) currentDelta;  
    tenzoProp.setSpeeds(tenzoProp.getThrottle(), 0, currentDelta, 0, 0);
    
  }
  
  contSignalRoutine++;
  signalComputeTimer = micros() - signalComputeTimer;
  
  if (maxsignalTimer <= signalComputeTimer)
    maxsignalTimer = signalComputeTimer;
  
  signalTimeTot = signalTimeTot + signalComputeTimer; 
}

void printSamples(){
  for (int i = 0; i < maxSamples; i++){
     Serial.print("y,");
      Serial.print(timeSamples[i]);
      Serial.print(",");
      Serial.print(deltaSamples[i]);  
      Serial.print(",");
      //Serial.print(estAngleSamples[i]);  
      Serial.print(1);  
      Serial.print(",");
      float a = (float) angleSamples[i];
      Serial.print(a/100);
      if (i==maxSamples-1)
        Serial.println(",P,z");
      else 
        Serial.println(",N,z");
      delay(20);
  }
   
}

void sendSamples(){
  
  
}

