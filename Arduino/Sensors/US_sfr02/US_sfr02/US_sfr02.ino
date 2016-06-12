

#include <Wire.h>

void setup()
{
  Wire.begin();                // join i2c bus (address optional for master)
  Serial.begin(9600);          // start serial communication at 9600bps
  Serial.println("OK");
}

int reading = 0;

void loop()
{
  Wire.beginTransmission(112); 

  Wire.write(byte(0x00));     
  Wire.write(byte(0x51));     
  Wire.endTransmission();    

  delay(70);                   

  Wire.beginTransmission(112); // transmit to device #112
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting

  Wire.requestFrom(112, 2);    // request 2 bytes from slave device #112

  // step 5: receive reading from sensor
  if (2 <= Wire.available())   // if two bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    Serial.print(reading);   // print the reading
    Serial.println("cm");
  }

  delay(250);                  // wait a bit since people have to read the output :)
}
