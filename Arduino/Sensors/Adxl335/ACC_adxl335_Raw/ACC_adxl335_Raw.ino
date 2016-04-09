
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

boolean stopCal = false;

int xRawMin = 512;
int xRawMax = 200;
 
int yRawMin = 512;
int yRawMax = 200;
 
int zRawMin = 512;
int zRawMax = 200;

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
  
  //convert read values to degrees -90 to 90 - Needed for atan2
  int xAng = map(xRead, minVal, maxVal, -90, 90);
  int yAng = map(yRead, minVal, maxVal, -90, 90);
  int zAng = map(zRead, minVal, maxVal, -90, 90);

  //Output the caculations
  Serial.print("x: ");
  Serial.print(xAng);
  Serial.print(" | y: ");
  Serial.print(yAng);
  Serial.print(" | z: ");
  Serial.print(zAng);
  Serial.print("X: ");
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
