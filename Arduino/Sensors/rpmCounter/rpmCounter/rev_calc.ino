
void computeRevolutions() {
  if (micros() - timeTracker >= SECOND_US) {
    rev_sec = counter/NUM_BLADES;
    rev_min = rev_sec*SEC_IN_MIN;
    rad_sec = rev_sec*3.1415/180;
    counter = 0;  
    timeTracker = micros();
  }  
}

void count() { // ISR
  counter++;
}
