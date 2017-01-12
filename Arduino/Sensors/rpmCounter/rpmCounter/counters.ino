
void computeAverageExecTime()
{
    // Compute average servo readings time
    if (countServoAction >  0)
      servoTimeTot = servoTimeTot/countServoAction;
    else 
      servoTimeTot = -999;
        
        
    
    // Compute average isr readings time
    if (countISR >  0)
      isrTimeTot = isrTimeTot/countISR;
    else 
      isrTimeTot = -999;
        
    
    // Compute average SerialRoutine time
    if (contSerialRoutine >  0)
      serialTimeTot = serialTimeTot/contSerialRoutine;
    else 
      serialTimeTot = -999;  
}



void UXRoutine()
{
    printTimersSched();
}

void printTimersSched() {
    if (printScheduler) {
      
      // Print Samples rate: [sample/sec] \t execTime \t wcet
      //ISR
      Serial.print("t,ISR: ");
      Serial.print(countISR);
      Serial.print("\t");
      Serial.print(isrTimeTot);
      Serial.print("\tMax ");
      Serial.print(maxisrTimer);
      
      
      Serial.print(",\nServo: ");
      Serial.print(countServoAction);
      Serial.print("\t");
      Serial.print(servoTimeTot);
      Serial.print("\tMax ");
      Serial.print(maxservoTimer);

      
      Serial.print(",\nSerial: ");
      Serial.print(contSerialRoutine);
      Serial.print("\t");
      Serial.print(serialTimeTot);
      Serial.print("\tMax ");
      Serial.println(maxserialTimer);
      
      Serial.println();
    }
}
