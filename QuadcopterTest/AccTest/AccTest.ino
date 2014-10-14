#include <math.h>
#include <float.h>
#include <limits.h>
 
 
int valx = 0;
int valy = 0;
int valz = 0;
 
int xaxis = 0;
int yaxis = 1;   
int zaxis = 2;   
 
float xa = 0;
float ya = 0;
float za = 0;
 
 
 
 
int minx = INT_MAX;
int maxx = 0;
int miny = INT_MAX;
int maxy = 0;
int minz = INT_MAX;
int maxz = 0;
 
int g0x = 0;
int g0y = 0;
int g0z = 0;
 
long time1 = 0;
long time2 = 0;
 
float rho = 0;
float phi = 0;
float theta = 0;
 
 
void setup()
{
  Serial.begin(9600);
  Serial.println("RST\r\n");
  pinMode(xaxis,INPUT);
  pinMode(yaxis,INPUT);
  pinMode(zaxis,INPUT);
  time1=millis();
  time2=millis();
}
 
void loop()
{
 
  valx = analogRead(xaxis);    // read the value from the sensor
  valy = analogRead(yaxis);    // read the value from the sensor
  valz = analogRead(zaxis);    // read the value from the sensor
 
  int pfx = valx;
  int pfy = valy;
  int pfz = valz;
 
  autoZeroCalibration(pfx,pfy,pfz);
 
  int fx = (pfx - g0x);
  int fy = (pfy - g0y);
  int fz = (pfz - g0z);
 
  float ax = fx*(3.3/(1024.0*0.800));
  float ay = fy*(3.3/(1024.0*0.800));
  float az = fz*(3.3/(1024.0*0.800));
 
  //
  /*
  rho =   atan(ax/sqrt(pow(ay,2)+pow(az,2)))*(360/(2*3.141592));  
  phi =   atan(ay/sqrt(pow(ax,2)+pow(az,2)))*(360/(2*3.141592));  
  theta = atan(sqrt(pow(ay,2)+pow(ax,2))/az)*(360/(2*3.141592));  
 
  printAngles();
 */
}
 
void autoZeroCalibration(int pfx, int pfy, int pfz)
{
    if ((pfx < minx)||(pfy < miny)||(pfz < minz)||(pfx > maxx)||(pfy > maxy)||(pfz > maxz)) {
     // autozero calibration
     if (pfx < minx) minx = pfx;
     if (pfy < miny) miny = pfy;
     if (pfz < minz) minz = pfz;
 
     if (pfx > maxx) maxx = pfx;
     if (pfy > maxy) maxy = pfy;
     if (pfz > maxz) maxz = pfz;
 
     g0x = ((maxx - minx)/2)+minx;
     g0y = ((maxy - miny)/2)+miny;
     g0z = ((maxz - minz)/2)+minz;
 
     printValues();
  }
}
void printFloat(float value, int places) {
   // this is used to cast digits
   int digit;
   float tens = 0.1;
   int tenscount = 0;
   int i;
   float tempfloat = value;
 
   // if value is negative, set tempfloat to the abs value
   // make sure we round properly. this could use pow from  
   //<math .h>, but doesn't seem worth the import
   // if this rounding step isn't here, the value  54.321 prints as  
   //54.3209
 
   // calculate rounding term d:   0.5/pow(10,places)
   float d = 0.5;
   if (value < 0)
     d *= -1.0;
   // divide by ten for each decimal place
   for (i = 0; i < places; i++)
     d/= 10.0;
   // this small addition, combined with truncation will round our  
   // values properly
   tempfloat +=  d;
 
   // first get value tens to be the large power of ten less than value
   // tenscount isn't necessary but it would be useful if you wanted  
   // to know after this how many chars the number will take
 
   if (value < 0)
     tempfloat *= -1.0;
   while ((tens * 10.0) <= tempfloat) {
     tens *= 10.0;
     tenscount += 1;
   }
 
 
   // write out the negative if needed
   if (value < 0)
     Serial.print('-');
 
   if (tenscount == 0)
     Serial.print(0, DEC);
 
   for (i=0; i< tenscount; i++) {
     digit = (int) (tempfloat/tens);
     Serial.print(digit, DEC);
     tempfloat = tempfloat - ((float)digit * tens);
     tens /= 10.0;
   }
 
   // if no places after decimal, stop now and return
   if (places <= 0)
     return;
 
   // otherwise, write the point and continue on
   Serial.print('.');
 
   // now write out each decimal place by shifting digits one by one  
   // into the ones place and writing the truncated value
   for (i = 0; i < places; i++) {
     tempfloat *= 10.0;
     digit = (int) tempfloat;
     Serial.print(digit,DEC);
     // once written, subtract off that digit
     tempfloat = tempfloat - (float) digit;
   }
}
 
void printAngles()
{
   if (millis()-time1 > 1000) {
    time1 = millis();
    Serial.print("rho: ");
    printFloat(rho,3);
    Serial.print(" phi: ");
    printFloat(phi,3);
    Serial.print(" theta: ");
    printFloat(theta,3);
    Serial.print("\r\n");
  }
}
 
void printValues()
{
  if ((millis() - time2) > 800) {
    time2=millis();
    Serial.print("minx: ");
    Serial.print((int)minx,DEC);
    Serial.print(".");
    Serial.print((minx-(int)minx)*1000,DEC);
    Serial.print(" miny: ");
    Serial.print((int)miny,DEC);
    Serial.print(".");
    Serial.print((miny-(int)miny)*1000,DEC);
    Serial.print(" minz: ");
    Serial.print((int)minz,DEC);
    Serial.print(".");
    Serial.print((minz-(int)minz)*1000,DEC);
    Serial.print("\r\n");
 
    Serial.print("maxx: ");
    Serial.print((int)maxx,DEC);
    Serial.print(".");
    Serial.print((maxx-(int)maxx)*1000,DEC);
    Serial.print(" maxy: ");
    Serial.print((int)maxy,DEC);
    Serial.print(".");
    Serial.print((maxy-(int)maxy)*1000,DEC);
    Serial.print(" maxz: ");
    Serial.print((int)maxz,DEC);
    Serial.print(".");
    Serial.print((maxz-(int)maxz)*1000,DEC);
    Serial.print("\r\n");
  }
}
