void setup()
{
  Serial.begin(115200);
}

void loop()
{
  SerialRoutine();
}

void SerialRoutine()
{
 //Read from usb serial to bluetooth
  if(Serial.available())
  {
    char modeS = Serial.read(); 
    
    if (modeS == 'a')
    {
      Serial.println(" TakeOff ");
    }
    if (modeS == 'L')
    {
      Serial.println(" Land ");
    }
    if (modeS == 'p')
    {
      Serial.println(" Pid ");
    }
    if (modeS == 't')
    {
      Serial.println("o,12,1,-4,");
    }
    if (modeS == 's')
    {
      Serial.println("c,p,");
    }
  } 
}
