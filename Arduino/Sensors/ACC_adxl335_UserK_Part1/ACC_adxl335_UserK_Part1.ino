
//Analog read pins
const int xPin = 0;
const int yPin = 1;
const int zPin = 2;

// Save values 
int xRead,yRead,zRead;

// to hold the caculated values
double x;
double y;
double z;

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
  Serial.println("Adxl355");
  
  pinMode(xPin,INPUT);
  pinMode(yPin,INPUT);
  pinMode(zPin,INPUT);  
}

void loop()
{
  readAcc();
  delay(40);
}

void readAcc()
{
  xRead = analogRead(xPin);
  yRead = analogRead(yPin);
  zRead = analogRead(zPin); 
  
  // Convert from raw to G
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

