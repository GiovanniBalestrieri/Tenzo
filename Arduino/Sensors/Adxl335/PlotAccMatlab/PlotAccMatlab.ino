//#define 3d3volts
//#define 5VOLTS

//Analog read pins
const int xPin = 0;
const int yPin = 1;
const int zPin = 2;

int xRead,yRead,zRead;

//to hold the caculated values
double x;
double y;
double z;

float xRaw,yRaw,zRaw;  
float xAccel,yAccel,zAccel;

// 3.3 V a vuoto
//int xRawMin = 406;
//int xRawMax = 271;
//// 
//int yRawMin = 401;
//int yRawMax = 266;
//// 
//int zRawMin = 418;
//int zRawMax = 284;

// 5V

//  int xRawMin = 628;
//  int xRawMax = 404;
//   
//  int yRawMin = 619;
//  int yRawMax = 398;
//   
//  int zRawMin = 626;
//  int zRawMax = 426;


// 3.3 V con carico Tenzo V2.1
//int xRawMin = 425;
//int xRawMax = 280;
//// 
//int yRawMin = 410;
//int yRawMax = 275;
//// 
//int zRawMin = 428;
//int zRawMax = 289;

// 3.3 V con  carico Tenzo V2.2
int xRawMin = 402;
int xRawMax = 277;
// 
int yRawMin = 402;
int yRawMax = 269;
// 
int zRawMin = 416;
int zRawMax = 283;

// angle estimated from aCc
float angleXAcc;
float angleYAcc;

float angleXAccF;
float angleYAccF;

volatile boolean filterAcc = true;
float alphaA = 0.995;
volatile float aF[3];

boolean processing = false;
boolean printAngles = false;

// Filter
float axm1,aym1,azm1;

void setup()
{
  Serial.begin(115200);
  //printCalVals();
}

void loop()
{
  //read the analog values from the accelerometer
  readAcc();
  serialRoutine();
  
  scaleAccs();
  calcAngle();
  
  //printVals();

  //delay(500);
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
  
  if (filterAcc)
   {      
      aF[0] = xAccel;
      aF[1] = yAccel;
      aF[2] = zAccel;
      aFilter(aF);
   }
}

void calcAngle()
{
  if (filterAcc)
  {
    angleXAccF = -(atan2(-aF[1],-aF[2])) * RAD_TO_DEG;
    angleYAccF = (atan2(-aF[0],-aF[2])) * RAD_TO_DEG;
  }
  
    // From Acc
    angleXAcc = -((atan2(-yAccel,-zAccel)))* RAD_TO_DEG;
    angleYAcc = ((atan2(-xAccel,-zAccel)))* RAD_TO_DEG;
  
  
  
//  angleXAcc = ((atan2(-yAccel,-zAccel)))* RAD_TO_DEG;
//  angleYAcc = ((atan2(xAccel,-zAccel)))* RAD_TO_DEG; 
  
//  angleXAcc = (atan2(xAccel,zAccel)) * RAD_TO_DEG;
//  angleYAcc = (atan2(yAccel,zAccel)) * RAD_TO_DEG;  
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
    
    Serial.print("A,");
    Serial.print(angleXAcc);
    Serial.print(",");
    Serial.print(angleYAcc);
    Serial.print(",");
    Serial.print(angleXAccF);
    Serial.print(",");
    Serial.print(angleYAccF);
    Serial.println(",Z");
    
    /*
    Serial.print("A,");
    Serial.print(xRead);
    Serial.print(",");
    Serial.print(yRead);
    Serial.print(",");
    Serial.print(zRead);
    Serial.println(",Z");
    */
    if (printAngles)
    {
      Serial.print("  Angles: ");
      Serial.print(angleXAcc);
      Serial.print("  :  ");
      Serial.print(angleYAcc);
    Serial.println(",Z");
    } 
  }
  else
  {
    /*
   Serial.print(",");
   Serial.print(xAccel);
   Serial.print(",");
   Serial.print(yAccel);
   Serial.print(",");
   */
  }
  //Serial.println();
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
//   Serial.println(t);
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
   }if (t =='T')
    {
      Serial.println("Ok");
    }  
    else if (t== 'M')
    {  
      printVals();
    }
   else if (t=='a')
   {
     printAngles = !printAngles; 
   }
 } 
}
