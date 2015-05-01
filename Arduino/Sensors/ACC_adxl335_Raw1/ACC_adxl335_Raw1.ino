
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
int nMax=200;

float xRaw,yRaw,zRaw;  
float xAccel,yAccel,zAccel;

boolean stopCal = false;

int xRawMin = 250;
int xRawMax = 410;
 
int yRawMin = 250;
int yRawMax = 415;
 
int zRawMin = 250;
int zRawMax = 420;

void setup()
{
  Serial.begin(9600);
  Serial.println("Occhio");
  calibrate();
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

void loop()
{
  //read the analog values from the accelerometer
  readAcc();
  
  float xScaled = map(xRead, xRawMin, xRawMax, -1000, 1000);
  float yScaled = map(yRead, yRawMin, yRawMax, -1000, 1000);
  float zScaled = map(zRead, zRawMin, zRawMax, -1000, 1000);
  
  // re-scale to fractional Gs
  float xAccel = xScaled / 1000.0;
  float yAccel = yScaled / 1000.0;
  float zAccel = zScaled / 1000.0;
  

  //Output the caculations
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
  Serial.println(zRead);

  delay(100);
}

void readAcc()
{
  xRead = analogRead(xPin);
  yRead = analogRead(yPin);
  zRead = analogRead(zPin); 
}

void calibrate()
{
  Serial.println("Calibrate");
  
  for(int i=0;i<nMax;i++)
  {
    Serial.println(i);
    delay(100);
    readAcc();  
    if (xRead < xRawMin)
    {
      xRawMin = xRead;
    }
    if (xRead > xRawMax)
    {
      xRawMax = xRead;
    }
    
    if (yRead < yRawMin)
    {
      yRawMin = yRead;
    }
    if (yRead > yRawMax)
    {
      yRawMax = yRead;
    }
   
    if (zRead < zRawMin)
    {
      zRawMin = zRead;
    }
    if (zRead > zRawMax)
    {
      zRawMax = zRead;
    }
  }
}
