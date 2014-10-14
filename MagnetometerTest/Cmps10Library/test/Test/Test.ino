/****************************************************************
*                  Arduino CMPS10 example code                  *
*                    CMPS10 running I2C mode                    *
*                    by James Henderson, 2012                   *
*****************************************************************/
#include<Wire.h>
#include <CMPS10.h>
                                    // Defines address of CMPS10

    // Defines software serial port for LCD03
CMPS10 my_compass;

void setup(){  
  Serial.begin(9600);   
  Wire.begin();
}

void loop(){                             // Stores full bearing
        
   float bearing =  my_compass.bearing();
   int pitch = my_compass.pitch();
   int roll = my_compass.roll();
   float acceleration = my_compass.acceleration_x();
   
   display_data(bearing,  pitch, roll);  // Display data to the LCD03
   Serial.println("Acc : ");
   Serial.print(acceleration);
   delay(100);
}

void display_data(int b, int p, int r)
{   
  Serial.print("Bearing = ");                      // Display the full bearing and fine bearing seperated by a decimal poin on the LCD03
  Serial.print(b);       
  Serial.print("  ");

  delay(5);

  Serial.print("Pitch = ");
  Serial.print(p);
  Serial.print(" ");

  delay(5);
  
  Serial.print("Roll = ");
  Serial.print(r);
  Serial.print(" ");
  Serial.println();
}

