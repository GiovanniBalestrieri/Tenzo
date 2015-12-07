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
float alphaA = 0.98;
volatile float axm1,aym1,azm1;





// Acc Timers
unsigned long accTimer;
unsigned long lastAccTimer;
unsigned long startRead;
unsigned long stopRead;
unsigned long timer;
int rate;
boolean filterAcc = true;
float aF[3];

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
     else if (t == 'M')
     {
        accRoutine(); 
     }
     
  }
}

void aFilter(volatile float val[])
{
  val[0] = (1-alphaA)*val[0] + alphaA*axm1;
  val[1] = (1-alphaA)*val[1] + alphaA*aym1;
  val[2] = (1-alphaA)*val[2] + alphaA*azm1;
  
  axm1 = val[0];
  aym1 = val[1];
  azm1 = val[2];
}

void accRoutine()
{
   x=analogRead(xaxis);
   y=analogRead(yaxis);
   z=analogRead(zaxis);
  
   aax = (((x*5000.0)/1023.0)-zeroX)/RESOLUTION;
   aay = (((y*5000.0)/1023.0)-zeroY)/RESOLUTION;
   aaz = (((z*5000.0)/1023.0)-zeroZ)/RESOLUTION;
   
   if (filterAcc)
   {
      aF[0] = aax;
      aF[1] = aay;
      aF[2] = aaz;
      aFilter(aF);
   }
  
   // computes sample time
   accTimer = millis() - lastAccTimer;
   // updates last reading timer
   lastAccTimer = millis(); 
   
     Serial.print("A,");
     Serial.print(aF[0]);
     Serial.print(",");
     Serial.print(aF[1]);
     Serial.print(",");          
     Serial.print(aF[2]); 
     Serial.print(","); 
     timer = millis();     
     Serial.print(timer);  
     Serial.print(","); 
     Serial.print("Z"); 
     Serial.println();
}
