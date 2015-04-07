// Sweep Sample
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.

#include <Servo.h>

Servo ST1, ST2; // We'll name the Sabertooth servo channel objects ST1 and ST2.
                // For how to configure the Sabertooth, see the DIP Switch Wizard for
                //   http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
                // Be sure to select RC Microcontroller Mode for use with this sample.
                //
                // Connections to make:
                //   Arduino Pin 9  ->  Sabertooth S1
                //   Arduino Pin 10 ->  Sabertooth S2
                //   Arduino GND    ->  Sabertooth 0V
                //   Arduino VIN    ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
                //
                // Sabertooth accepts servo pulses from 1000 us to 2000 us.
                // We need to specify the pulse widths in attach(). 0 degrees will be full reverse, 180 degrees will be
                // full forward. Sending a servo command of 90 will stop the motor. Whether the servo pulses control
                // the motors individually or control throttle and turning depends on your mixed mode setting.

// Notice these attach() calls. The second and third arguments are important.
// With a single argument, the range is 44 to 141 degrees, with 92 being stopped.
// With all three arguments, we can use 0 to 180 degrees, with 90 being stopped.
void setup()
{
  ST1.attach( 9, 1000, 2000);
  ST2.attach(10, 1000, 2000);
}

void loop()
{
  int power;
  
  // Ramp both servo channels from 0 to 180 (full reverse to full forward),
  // waiting 20 ms (1/50th of a second) per value.
  for (power = 0; power <= 180; power ++)
  {
    ST1.write(power);
    ST2.write(power);
    delay(20);
  }
  
  // Now go back the way we came.
  for (power = 180; power >= 0; power --)
  {
    ST1.write(power);
    ST2.write(power);
    delay(20);
  }
}
