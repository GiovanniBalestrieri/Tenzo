
void serialRoutine() {
  if (printRevSec) {
    Serial.print("\trev/sec:\t");
    Serial.println(rev_sec);
  }else if (printRevMin) {
    Serial.print("\trev/min:\t");
    Serial.println(rev_min);
  }else if (printRadSec) {
    Serial.print("\trad/sec:\t");
    Serial.println(rad_sec);
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
  }           
}

