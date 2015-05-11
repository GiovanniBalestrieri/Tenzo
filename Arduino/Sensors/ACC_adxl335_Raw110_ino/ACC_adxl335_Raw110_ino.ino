
//Analog read pins
const int xPin = 0;
const int yPin = 2;
const int zPin = 1;

int xRead,yRead,zRead;

//to hold the caculated values
double x;
double y;
double z;

float xRaw,yRaw,zRaw;  
float xAccel,yAccel,zAccel;

int xRawMin = 416;
int xRawMax = 250;
 
int yRawMin = 400;
int yRawMax = 268;
 
int zRawMin = 401;
int zRawMax = 269;


// angle estimated from kalman
float angleXAcc;
float angleYAcc;

boolean processing = false;
boolean printAngles = false;

void setup()
{
  Serial.begin(9600);
  Serial.println("Occhio");
  pinMode(zPin,INPUT); 
  printCalVals();
}

void loop()
{
  //read the analog values from the accelerometer
  readAcc();
  serialRoutine();
  
  scaleAccs();
  calcAngle();
  
  printVals();

  delay(500);
}

void scaleAccs()
{
  float xScaled = map(xRead, xRawMin, xRawMax, -1000, 1000);
  float yScaled = map(yRead, yRawMin, yRawMax, -1000, 1000);
  float zScaled = map(zRead, zRawMin, zRawMax, -1000, 1000);
  
  //float zScaled = zm;
  
  // re-scale to fractional Gs
  xAccel = xScaled / 1000.0;
  yAccel = yScaled / 1000.0;
  zAccel = zScaled / 1000.0; 
}

void calcAngle()
{
  angleXAcc = (atan2(xAccel,zAccel)) * RAD_TO_DEG;
  angleYAcc = (atan2(yAccel,zAccel)) * RAD_TO_DEG;  
}

void readAcc()
{
  xRead = analogRead(xPin);
  yRead = analogRead(yPin);
  zRead = analogRead(zPin); 
}

void printVals()
{
  if (!processing)
  {
    Serial.print("xG: ");
    Serial.print(xAccel);
    Serial.print(" | yG: ");
    Serial.print(yAccel);
    Serial.print(" | zG: ");
    Serial.print(zAccel);
    Serial.print("      X: ");
    Serial.print(xRead);
    Serial.print(" | Y: ");
    Serial.print(yRead);
    Serial.print(" | Z: ");
    Serial.print(zRead);
    if (printAngles)
    {
      Serial.print("  Angles: ");
      Serial.print(angleXAcc);
      Serial.print("  :  ");
      Serial.print(angleYAcc);
    } 
  }
  else
  {
   Serial.print(",");
   Serial.print(xAccel);
   Serial.print(",");
   Serial.print(yAccel);
   Serial.print(",");
  }
  Serial.println();
}

void printCalVals()
{
  Serial.print("X _ min max: ");
  Serial.print(xRawMin);
  Serial.print(" : ");
  Serial.println(xRawMax);
  Serial.print("Y _ min max: ");
  Serial.print(yRawMin);
  Serial.print(" : ");
  Serial.println(yRawMax);
  Serial.print("Z _ min max: ");
  Serial.print(zRawMin);
  Serial.print(" : ");
  Serial.println(zRawMax);  
}


void serialRoutine()
{
 if (Serial.available())
 {
   char t = Serial.read();
   Serial.println(t);
   if (t == 'x')
   {   
     char sec = Serial.read();
     if (sec == 'm')
     {
      xRawMin = xRead;
       Serial.print("Min X: ");
       Serial.println(xRawMin);
     }
     else if (sec == 'M')
     {
       xRawMax = xRead;
       Serial.print("Max X: ");
       Serial.println(xRawMax);
     }
   }
   else if (t == 'y')
   {   
     char sec = Serial.read();
     if (sec == 'm')
     {
      yRawMin = yRead;
       Serial.print("Min Y: ");
       Serial.println(yRawMin);
     }
     else if (sec == 'M')
     {
       yRawMax = yRead;
       Serial.print("Max Y: ");
       Serial.println(yRawMax);
     }
   }
   else if (t == 'z')
   {   
     char sec = Serial.read();
     if (sec == 'm')
     {
      zRawMin = zRead;
       Serial.print("Min Z: ");
       Serial.println(zRawMin);
     }
     else if (sec == 'M')
     {
       zRawMax = zRead;
       Serial.print("Max Z: ");
       Serial.println(zRawMax);
     }
   }
   else if (t=='v')
   {
     Serial.print("Min x: ");
     Serial.println(xRawMin);
     Serial.print("Max x: ");
     Serial.println(xRawMax);
     Serial.print("Min Y: ");
     Serial.println(yRawMin);
     Serial.print("Max Y: ");
     Serial.println(yRawMax);
     Serial.print("Min Z: ");
     Serial.println(zRawMin);
     Serial.print("Max Z: ");
     Serial.println(zRawMax);
   }
   else if (t=='p')
   {
     processing != processing;
   }
   else if (t=='a')
   {
     printAngles = !printAngles; 
   }
 } 
}
