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
   if (Serial.available()>1)
   {
     mode = Serial.read();
     if (mode == 83)
     {
       startRead = micros();
       Serial.println("Acquiring 100 samples.");
       for (int i=0; i <= lenBuff ; i++)
       {
         lastAccTimer = micros();
         x=analogRead(xaxis);        
         aax = (((x*5000.0)/1023.0)-zeroX)/RESOLUTION; 
         values[i] = aax;
         time[i] = accTimer;
         accTimer = micros() - lastAccTimer;
       }       
       stopRead = micros() - startRead;       
       Serial.println("Printing acceleration values");
       for (int i=0;i<=lenBuff;i++)
       {
         Serial.print(values[i]);
         Serial.print("@");
         Serial.println(time[i]);
       }
       
       int rate1 = stopRead/lenBuff;
       rate = accTimer;
       Serial.print("Sampling rate: ");
       Serial.print(rate);
       Serial.println(" us And ::");
       Serial.print(rate1);
       Serial.println(" us");
       
       
       Serial.print("Total time ");
       Serial.print(stopRead);
       Serial.println(" us");
       contSamples = 0;  
     } 
   }
}
