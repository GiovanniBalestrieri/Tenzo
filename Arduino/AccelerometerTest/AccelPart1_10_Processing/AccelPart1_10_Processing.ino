//#define DEBUGMODE

const float RESOLUTION=800; //0.8 v/g -> resolucion de 1.5g -> 800mV/g
const float VOLTAGE=3.3;  //voltage al que está conectado el acelerómetro

const float ZOUT_1G = 850;   // mv Voltage en Zout a 1G

const int NADJ  = 50;        // Número de lecturas para calcular el error

// Entradas analógicas donde van los sensores
const int xaxis = 0;
const int yaxis = 1;
const int zaxis = 2;

float XError,YError,ZError;
float xd,yd,zd,z,x,y;
float aax,aay,aaz;


// Acc Timers
unsigned long accTimer;
unsigned long lastAccTimer;
unsigned long timeToRead = 0;
unsigned long lastTimeToRead = 0;

int contSamples = 0;
int lastTimeToRead = 0;//500;
int samplesNum = 0;

byte mode;
unsigned int sensorValue = 0;
boolean connectEd = false;

// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);


float AccelAdjust(int axis)
{
 float acc = 0;
 for (int j=0;j<NADJ;j++)
 {
   float lectura=analogRead(axis);
   acc = acc + ((lectura*5000)/1023.0);
   delay(11); //número primo para evitar ciclos de lectura proporcionales
 }
 return acc/NADJ;
}



void setup()
{  
 Serial.begin(115200); // 9600 bps
 pinMode(xaxis,INPUT);
 pinMode(yaxis,INPUT);
 pinMode(zaxis,INPUT);

 XError =  AccelAdjust(xaxis);
 YError =  AccelAdjust(yaxis);
 ZError =  AccelAdjust(zaxis);
 ZError = ZError - ZOUT_1G;
 
 
 // set up the ADC
 ADCSRA &= ~PS_128;  // remove bits set by Arduino library
 // you can choose a prescaler from above.
 // PS_16, PS_32, PS_64 or PS_128
 ADCSRA |= PS_32;    // set our own prescaler to 64 
}

void loop()
{
 accRoutine(); 
 serialRoutine();
 delay(20);
} 

void serialRoutine()
{
   lastTimeToRead = micros();
   x=analogRead(xaxis);
   y=analogRead(yaxis);
   z=analogRead(zaxis);
   timeToRead = micros() - lastTimeToRead;
   
   aax = (((x*5000.0)/1023.0)-XError)/RESOLUTION;
   aay = (((y*5000.0)/1023.0)-YError)/RESOLUTION;
   aaz = (((z*5000.0)/1023.0)-ZError)/RESOLUTION;
  
   // gets the value sample time
   accTimer = micros() - lastAccTimer;
   // updates last reading timer
   lastAccTimer = micros(); 
   
   Serial.print("S");
   Serial.print(",");
   Serial.print(aax);
   Serial.print(",");
   Serial.print(aay);
   Serial.print(",");
   Serial.print(aaz);
   Serial.print(",");
   Serial.print(accTimer);
   Serial.print(",");
   Serial.print(timeToRead);
   Serial.print(",");
   Serial.println("E");
  /*
   if (Serial.available()>1)
   {
     mode = Serial.read();
     if (mode == 82)
     {
       lastTimeToRead = micros();
       Serial.print("T");
       Serial.print(",");
       Serial.println("A");
       delay(100);
       while (contSamples <= samplesNum)
       {
         timeToRead = micros() - lastTimeToRead;
         x=analogRead(xaxis);
         y=analogRead(yaxis);
         z=analogRead(zaxis);
        
         aax = (((x*5000.0)/1023.0)-XError)/RESOLUTION;
         aay = (((y*5000.0)/1023.0)-YError)/RESOLUTION;
         aaz = (((z*5000.0)/1023.0)-ZError)/RESOLUTION;
        
         // gets the value sample time
         accTimer = micros() - lastAccTimer;
         // updates last reading timer
         lastAccTimer = micros(); 
         
         Serial.print("S");
         Serial.print(",");
         Serial.print(aax);
         Serial.print(",");
         Serial.print(aay);
         Serial.print(",");
         Serial.print(aaz);
         Serial.print(",");
         Serial.println(lastAccTimer);
         //Plot terminator carriage
         //Serial.write(13);
         contSamples++;
       }
       
       Serial.print("T");
       Serial.print(",");
       Serial.println("B");     
       contSamples = 0;  
     } 
     else if (mode == 84)
     {
       Serial.print("R");
       Serial.print(",");
       Serial.print(aax);
       Serial.print(",");
       Serial.print(aay);
       Serial.print(",");
       Serial.print(aaz);
       Serial.print(",");
       Serial.println(lastAccTimer);
     }
    if(mode == 16)
    {  	
      Serial.write(17);
    }
    if (mode == 18)
    {
      connectEd = true;
      Serial.write(19);
    }
   }
   */
}

void accRoutine()
{
  
   x=analogRead(xaxis);
   y=analogRead(yaxis);
   z=analogRead(zaxis);
  
   aax = (((x*5000.0)/1023.0)-XError)/RESOLUTION;
   aay = (((y*5000.0)/1023.0)-YError)/RESOLUTION;
   aaz = (((z*5000.0)/1023.0)-ZError)/RESOLUTION;
  
   // gets the value sample time
   accTimer = micros() - lastAccTimer;
   // updates last reading timer
   lastAccTimer = micros(); 
  
  #ifdef DEBUGMODE
   Serial.print(aax);
   Serial.print(" ");
   Serial.print(aay);
   Serial.print(" ");          
   Serial.print(aaz);
   Serial.println();
  #endif
}
