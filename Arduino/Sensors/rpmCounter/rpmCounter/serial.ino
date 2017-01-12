
void serialRoutine() {
  
  serialTimer = micros();

  // #LOOP
  if (timerSec >= MAIN_LOOP_DISP_PERIOD)
  {
    if (printScheduler)
    {
      for(int i = 1; i<=scheduler.num_tasks ;i++) {
        if (scheduler.isTaskAlive(i)) {
          Serial.print("T"); Serial.print(i);
          Serial.print("\t(");
          Serial.print(scheduler.getTaskPeriod(i));
          Serial.print(",\te,");
          Serial.print(scheduler.getTaskPriority(i));
          Serial.print(")\tJob queue:  ");
          Serial.print(scheduler.getJobReleased(i));
          //Serial.print("\tValid:  ");
          //Serial.print(scheduler.isTaskValid(i));
          //Serial.print("\tActive:  ");
          //Serial.print(scheduler.isTaskActive(i));
          Serial.print("\t");
          Serial.println(scheduler.getTaskLabel(i));
        }
      }
    }
    
    computeAverageExecTime();
    
    UXRoutine();
    
    resetCounters();
    
    secRoutine = micros();
  }
  
  if (printRev) {
    if (printRevSec) {
      Serial.print("r/s:\t");
      Serial.println(rev_sec);
    }else if (printRevMin) {
      Serial.print("rev/min:\t");
      Serial.println(rev_min);
    }else if (printRadSec) {
      Serial.print("rad/sec:\t");
      Serial.println(rad_sec);
    }
  } 
  if (printServoSignal) {
    Serial.println(currentUs);
  }
  
  if (Serial.available()>0){
    char t = Serial.read();

    if (t == 'a' && !working){
      working = true;
      currentUs += 5;
      working = false; 
    }
    else if (t == 'm' && !working){
      working = true;
      Serial.println("Switching to rev/min");
      printRevSec = false;
      printRevMin = true;
      printRadSec = false;
      working = false; 
    }
    else if (t == 'r' && !working){
      working = true;
      Serial.println("Switching to rad/sec");
      printRevSec = false;
      printRevMin = false;
      printRadSec = true;
      working = false; 
    }
    else if (t == 's' && !working){
      working = true;
      Serial.println("Switching to rev/sec");
      printRevSec = true;
      printRevMin = false;
      printRadSec = false;
      working = false; 
    }
    else if (t == 'i' && !working){
      working = true;
      Serial.println("initializing");
      initialize = true;
      working = false; 
    }
    else if (t == 'l' && !working){
      working = true;
      Serial.println("Landing");
      land = true;
      working = false; 
    } else if (t == 'u' && !working){
      working = true;
      
      printServoSignal =  !printServoSignal;
      working = false; 
    } else if (t == 'p' && !working){
      working = true;
      
      printRev =  !printRev;
      working = false; 
    } else if (t == 't' && !working){
      working = true;
      
      test = !test;
      working = false; 
    } else if (t == 'z' && !working){
      working = true;
      currentUs -= 5;
      working = false; 
    } else if (t == 'c' && !working){
      working = true;
      calibrate();
      working = false; 
    } else if (t == 's' && !working){
      working = true;
      printScheduler != printScheduler;
      working = false; 
    
    } else if (t == 'd' && !working){
      working = true;
      detachMotors();
      working = false; 
    }
  }      


  
  contSerialRoutine++;
  serialTimer = micros() - serialTimer;
  
    if (maxserialTimer <= serialTimer && !initializing && !landing)
      maxserialTimer = serialTimer;
  
  serialTimeTot = serialTimeTot + serialTimer;
}

