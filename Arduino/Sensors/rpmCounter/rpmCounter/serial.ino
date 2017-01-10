
void serialRoutine() {
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
      // FREE
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
    }
  }           
}

