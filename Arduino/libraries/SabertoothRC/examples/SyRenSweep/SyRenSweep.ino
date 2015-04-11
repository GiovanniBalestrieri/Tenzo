// Sweep Sample
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.

#include <Servo.h>

Servo SR; // We'll name the SyRen servo channel object SR.
          // For how to configure the SyRen, see the DIP Switch Wizard for
          //   http://www.dimensionengineering.com/datasheets/SyrenDIPWizard/start.htm
          // Be sure to select RC Microcontroller Mode for use with this sample.
          //
          // Connections to make:
          //   Arduino Pin 9  ->  SyRen S1
          //   Arduino GND    ->  SyRen 0V
          //   Arduino VIN    ->  SyRen 5V (OPTIONAL, if you want the SyRen to power the Arduino)
          //
          // SyRen accepts servo pulses from 1000 us to 2000 us.
          // We need to specify the pulse widths in attach(). 0 degrees will be full reverse,
          // 180 degrees will be full forward. Sending a servo command of 90 will stop the motor.

// Notice this attach() call. The second and third arguments are important.
// With a single argument, the range is 44 to 141 degrees, with 92 being stopped.
// With all three arguments, we can use 0 to 180 degrees, with 90 being stopped.          
void setup()
{
  SR.attach(9, 1000, 2000);
}

void loop()
{
  int power;
  
  // Ramp the servo channel from 0 to 180 (full reverse to full forward),
  // waiting 20 ms (1/50th of a second) per value.
  for (power = 0; power <= 180; power ++)
  {
    SR.write(power);
    delay(20);
  }
  
  // Now go back the way we came.
  for (power = 180; power >= 0; power --)
  {
    SR.write(power);
    delay(20);
  }
}
