#include <Wire.h>

#define ADDRESS 0x60        // Defines address of CMPS10


void setup(){
  Wire.begin();  // Conects I2C
  Serial.begin(9600);
}

void loop(){
   byte highByte, lowByte, fine;              // highByte and lowByte store high and low bytes of the bearing and fine stores decimal place of bearing
   char pitch, roll;                          // Stores pitch and roll values of CMPS10, chars are used because they support signed value
   int bearing;                               // Stores full bearing
   
   Wire.beginTransmission(ADDRESS);           //starts communication with CMPS10
   Wire.write(2);                              //Sends the register we wish to start reading from
   Wire.endTransmission();

   Wire.requestFrom(ADDRESS, 4);              // Request 4 bytes from CMPS10
   while(Wire.available() < 4);               // Wait for bytes to become available
   highByte = Wire.read();           
   lowByte = Wire.read();            
   pitch = Wire.read();              
   roll = Wire.read();               
   
   bearing = ((highByte<<8)+lowByte)/10;      // Calculate full bearing
   fine = ((highByte<<8)+lowByte)%10;         // Calculate decimal place of bearing
   
   display_data(bearing, fine, pitch, roll);  // Display data to the LCD03
   
   delay(1000);
}

void display_data(int b, int f, int p, int r){    // pitch and roll (p, r) are recieved as ints instead oif bytes so that they will display corectly as signed values.
  
  Serial.print("Bearing = ");                      // Display the full bearing and fine bearing seperated by a decimal poin on the LCD03
  Serial.print(b);                               
  Serial.print(".");
  Serial.println(f);
  Serial.print("Pitch = ");
  Serial.println(p);
  Serial.print("Roll = ");
  Serial.println(r);
  delay(1000);
} 
