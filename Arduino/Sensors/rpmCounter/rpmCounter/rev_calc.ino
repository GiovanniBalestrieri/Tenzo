
void computeRevolutions() {
  
  revTimer = micros();    
  
  //if (micros() - timeTracker >= SECOND_US) {
    float dt = micros() - timeTracker;
    rev_sec = (counter/NUM_BLADES)*SECOND_US/dt;
    rev_min = rev_sec*SEC_IN_MIN;
    rad_sec = rev_sec*3.1415/180;
    counter = 0;  
    timeTracker = micros();

    contRevRoutine++;  
    revTimer = micros() - revTimer;
    
    if (maxRevTimer <= revTimer)
    maxRevTimer = revTimer;
    revTimeTot = revTimeTot + revTimer;
    
    contRev = 0;
  
    
  //}  
}

void count() { // ISR
  counter++;
}
