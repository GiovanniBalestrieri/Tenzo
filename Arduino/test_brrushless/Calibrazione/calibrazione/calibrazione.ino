
#include <Servo.h>


Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
int throttle = 0;

// Motor constant
int thresholdUp=2000, thresholdDown=0;
int dance = 1000;

void setup()
{
  Serial.begin(115200);
   servo1.attach(3); 
   servo2.attach(5);     
   servo3.attach(22);    
   servo4.attach(9);  
  
   servo1.write(0); 
   servo2.write(0); 
   servo3.write(0); 
   servo4.write(0); 
   dance = throttle*2 +0;
}

void loop()
{ 
 serialRoutine();
 delay(20);
 Serial.println(dance); 
  servo1.writeMicroseconds(dance);  
  servo3.writeMicroseconds(dance);  
  servo2.writeMicroseconds(dance);  
  servo4.writeMicroseconds(dance);  
}

void serialRoutine()
{
  if (Serial.available())
   {
     byte modeS = Serial.read();
     if (modeS == 'a')
      { 
        Serial.println("+5");   
        dance = dance +5;
        if (dance>=thresholdUp)
            dance = thresholdUp;
        if (dance<=thresholdDown)
            dance = thresholdDown;
        Serial.println(dance);   
      }
      
     if (modeS == 'z')
      {
        Serial.println("            -5");  
        dance = dance -5;
        if (dance>=thresholdUp)
            dance = thresholdUp;
        if (dance<=thresholdDown)
            dance = thresholdDown;
      }
      
      if (modeS == 'd')
      { 
        Serial.println("Descrescendo");   
        dance = 2000;
        for (int i = 2000; i>=1000; i--)    
        {         
          dance =  i;     
          Serial.println(dance);
          if (dance>=thresholdUp)
              dance = thresholdUp;
          if (dance<=thresholdDown)
              dance = thresholdDown;              
          Serial.println(dance);
          servo1.writeMicroseconds(dance);  
          servo3.writeMicroseconds(dance);
          servo2.writeMicroseconds(dance);  
          servo4.writeMicroseconds(dance);
          delay(1); 
        }
        throttle = dance; 
      }
      
      if (modeS == 'c')
      {
        Serial.println("            Crescendo");  
        dance = 0;
        for (int i = 700; i<2000; i++)    
        {         
          dance = i;
          if (dance>=thresholdUp)         
              dance = thresholdUp;
          if (dance<=thresholdDown)
              dance = thresholdDown;
          Serial.println(dance);
          servo1.writeMicroseconds(dance);  
          servo3.writeMicroseconds(dance);
          servo2.writeMicroseconds(dance);  
          servo4.writeMicroseconds(dance);
          delay(1);
        }
        throttle = dance;                
      }   
      if (modeS == 'o') // MAX
      {
        Serial.print("MAX:2000...");  
        dance = 2000;
          servo1.writeMicroseconds(dance);  
          servo3.writeMicroseconds(dance);
          servo2.writeMicroseconds(dance);  
          servo4.writeMicroseconds(dance);
        delay(100);
        Serial.println(" Ok.");               
      }      
   }  
   
}
