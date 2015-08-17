#include <Wire.h>

// Defines address of CMPS10
#define ADDRESS 0x60      

byte highByte, lowByte, fine;    
char pitch, roll;                
int bearing;   

void setup()
{
  Wire.begin();
  Serial.begin(9600);
}

void loop()
{                            
  requestData();
  getData();
  displayData(bearing, fine, pitch, roll);  // Display data to the LCD03
   
  delay(10);
}

void requestData()
{
   //starts communication with CMPS10
   Wire.beginTransmission(ADDRESS); 
   //Sends the register we wish to start reading from   
   Wire.write(2);                 
   Wire.endTransmission();             

   // Request 4 bytes from CMPS10
   Wire.requestFrom(ADDRESS, 4); 
   // Wait for bytes to become available   
   while(Wire.available() < 4);              
   highByte = Wire.read();           
   lowByte = Wire.read();            
   pitch = Wire.read();              
   roll = Wire.read();               
}

void getData()
{
   // Calculate full bearing
   bearing = ((highByte<<8)+lowByte)/10;  

   // Calculate decimal place of bearing   
   fine = ((highByte<<8)+lowByte)%10;   
}

void displayData(int b, int f, int p, int r)
{      
  Serial.print("Bearing = ");                      
  Serial.print(b);                               
  Serial.print(".");
  Serial.print(f);
  Serial.print("     Pitch = ");
  Serial.print(p);
  Serial.print("     Roll = ");
  Serial.println(r);
} 
