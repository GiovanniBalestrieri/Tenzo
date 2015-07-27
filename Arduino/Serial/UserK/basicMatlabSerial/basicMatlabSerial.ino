int a=0;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  serialRoutine();
}

void serialRoutine()
{
  if(Serial.available()>0)
  {
    char t = Serial.read();
    
    if (t =='T')
    {
      Serial.println("Ok");
    }  
    else if (t== 'M')
    {  
      a++;
      Serial.println(a);
    }
  }
}
