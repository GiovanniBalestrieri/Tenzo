

#include <Wire.h>

int reading = 0;
double sonarTimer= 0;
long int contSonarRoutine= 0;
long int maxsonarTimer = 0;
double sonarTimeTot =0;

void setup()
{
  Wire.begin();                
  Serial.begin(115200);
  Serial.println("OK");
}


void loop()
{
  delay(70);                   

  Wire.beginTransmission(112); // transmit to device #112
  Wire.write(byte(0x02));      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting
  
  sonarTimer = micros();
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


  contSonarRoutine++;  
  sonarTimer = micros() - sonarTimer;
  if (maxsonarTimer <= sonarTimer)
    maxsonarTimer = sonarTimer;
  sonarTimeTot = sonarTimeTot + sonarTimer;
    Serial.print(sonarTimer);   // print the reading

  delay(1000);                  // wait a bit since people have to read the output :)
}
