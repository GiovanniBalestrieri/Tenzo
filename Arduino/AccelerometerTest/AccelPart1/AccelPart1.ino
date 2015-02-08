const float RESOLUTION=800; // Resolution 1.5g -> 800mV/g
const float VOLTAGE=3.3;  
const float ZOUT_1G = 850;   // mv Voltage @ 1G
const int N  = 50;        // Número de lecturas para calcular el error

// Connect the X,Y and Z pin to A0,A1 and A2 respectively
const int xaxis = 0;
const int yaxis = 1;
const int zaxis = 2;

float zeroX,zeroY,zeroZ;
int x,y,z;
float aax,aay,aaz;

// Acc Timers
unsigned long accTimer;
unsigned long lastAccTimer;

byte mode;

#define DEBUG

float zeroPoint(int axis)
{
 float acc = 0;
 for (int j=0;j<N;j++)
 {
   
   acc = acc + (((float) analogRead(axis)*5000)/1023.0);
   delay(20); 
 }
 return acc/N;
}

void setup()
{  
 Serial.begin(9600); 
 pinMode(xaxis,INPUT);
 pinMode(yaxis,INPUT);
 pinMode(zaxis,INPUT);

 zeroX =  zeroPoint(xaxis);
 zeroY =  zeroPoint(yaxis);
 zeroZ =  zeroPoint(zaxis);
 zeroZ = zeroZ - ZOUT_1G;
}

void loop()
{
 accRoutine(); 
 serialRoutine();
} 

void serialRoutine()
{
 if (Serial.available()>1)
 {
  mode = Serial.read();
  if (mode == 82)
  {
     #ifdef DEBUG
     Serial.print("Acc[ ");
     Serial.print(aax);
     Serial.print(" ; ");
     Serial.print(aay);
     Serial.print(" ; ");          
     Serial.print(aaz);
     Serial.print(" ]"); 
     Serial.print(" @ ");        
     Serial.print(accTimer);  
     Serial.print(" ms/sample");   
     Serial.println();
    #endif
  } 
  if(mode == 16)
  {  	
    Serial.write(17);
  }
  if (mode == 18)
  {
    Serial.write(19);
  }
 }
}

void accRoutine()
{
   x=analogRead(xaxis);
   y=analogRead(yaxis);
   z=analogRead(zaxis);
  
   aax = (((x*5000.0)/1023.0)-zeroX)/RESOLUTION;
   aay = (((y*5000.0)/1023.0)-zeroY)/RESOLUTION;
   aaz = (((z*5000.0)/1023.0)-zeroZ)/RESOLUTION;
  
   // computes sample time
   accTimer = millis() - lastAccTimer;
   // updates last reading timer
   lastAccTimer = millis(); 
   
    #ifdef DEBUG
     Serial.print("Acc[ ");
     Serial.print(aax);
     Serial.print(" ; ");
     Serial.print(aay);
     Serial.print(" ; ");          
     Serial.print(aaz);
     Serial.print(" ]"); 
     Serial.print(" @ ");        
     Serial.print(accTimer);  
     Serial.print(" ms/sample");   
     Serial.println();
    #endif
}
