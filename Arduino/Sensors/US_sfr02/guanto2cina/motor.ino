
void viberRoutine()
{
  //int us = analogRead(0);
  //us = K*1/reading;
  if (modeD)
  {
    if (reading < thresholdMin)
    {
      analogWrite(9, 255);
      coro = true;
    }
    else
      analogWrite(9, 0);
  }
  else
  {
    if (reading < thresholdMin) {
      analogWrite(9, 0);
    } else if (reading > thresholdMax) {
      analogWrite(9, 0);
    } else {
      us = map(reading, thresholdMax, thresholdMin, 100, 255);
      analogWrite(9, us);
    }
  }
  readingM1 = reading;
}
