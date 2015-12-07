//#define DEBUGMODE

const float RESOLUTION=800;
const float VOLTAGE=3.3;

const float ZOUT_1G = 850;

const int N  = 50;   

// Connect the X,Y and Z pin to A0,A1 and A2 respectively
const int xaxis = 0;
const int yaxis = 1;
const int zaxis = 2;

float zeroX,zeroY,zeroZ;
int x,y,z;
float aax,aay,aaz;

// Array to store data
int lenBuff = 100;
float values[100];
float time[100];
int contSamples = 0;

// Serial byte received
byte mode;

// Acc Timers
unsigned long accTimer;
unsigned long lastAccTimer;
unsigned long startRead;
unsigned long stopRead;
int rate;

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
 Serial.begin(9600); // 9600 bps
 pinMode(xaxis,INPUT);
 pinMode(yaxis,INPUT);
 pinMode(zaxis,INPUT);

 zeroX =  zeroPoint(xaxis);
 zeroY =  zeroPoint(yaxis);
 zeroZ =  zeroPoint(zaxis);
 zeroZ = zeroZ - ZOUT_1G;
 
 Serial.println("Ready");
}

void loop()
{
  serialRoutine();
 
} 

void serialRoutine()
{
  if (Serial.available())
  {
     char t = Serial.read();
     if ( t == 'T')
     {
       Serial.println("Ok");  
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
   
     Serial.print("A,");
     Serial.print(aax);
     Serial.print(" ; ");
     Serial.print(aay);
     Serial.print(" ; ");          
     Serial.print(aaz);
     Serial.print(","); 
     Serial.print("Z");        
     //Serial.print(accTimer);  
     //Serial.print(" ms/sample");   
     Serial.println();
}
