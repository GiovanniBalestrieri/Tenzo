
//Analog read pins
const int xPin = 0;
const int yPin = 1;
const int zPin = 2;

//The minimum and maximum values that came from
//the accelerometer while standing still
//You very well may need to change these
int minVal = 265;
int maxVal = 402;

int xRead,yRead,zRead;

//to hold the caculated values
double x;
double y;
double z;
int nMax=100;

float xRaw,yRaw,zRaw;  
float xAccel,yAccel,zAccel;

float zeroX,zeroY,zeroZ;
float aax,aay,aaz;

// Acc Timers
unsigned long accTimer;
unsigned long lastAccTimer;

const float RESOLUTION=300; // Resolution 1.5g -> 800mV/g
const float VOLTAGE=3.3;  
const float ZOUT_1G = 406; 
const int N  = 100;

void setup()
{
  Serial.begin(9600);
  Serial.println("Occhio");
  
   pinMode(xPin,INPUT);
   pinMode(yPin,INPUT);
   pinMode(zPin,INPUT);
  
   zeroX =  zeroPoint(xPin);
   zeroY =  zeroPoint(yPin);
   zeroZ =  zeroPoint(zPin);
   zeroZ = zeroZ - ZOUT_1G;
}

void loop()
{
  //read the analog values from the accelerometer
  readAcc();
  delay(40);
}


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

void readAcc()
{
  xRead = analogRead(xPin);
  yRead = analogRead(yPin);
  zRead = analogRead(zPin); 
  
  aax = (((xRead*5000.0)/1023.0)-zeroX)/RESOLUTION;
  aay = (((yRead*5000.0)/1023.0)-zeroY)/RESOLUTION;
  aaz = (((zRead*5000.0)/1023.0)-zeroZ)/RESOLUTION;
  
  // computes sample time
  accTimer = millis() - lastAccTimer;
  // updates last reading timer
  lastAccTimer = millis(); 
   
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
}

