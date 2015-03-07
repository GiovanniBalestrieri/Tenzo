
/************************************************************
 *******************          Wiring:         **************
*************************************************************

Arduino Pin A0  ->  AttoPilot V
Arduino Pin A1  ->  AttoPilot I  -> doesn't work
Arduino Pin 9  ->  Sabertooth S1
Arduino Pin 10 ->  Sabertooth S2
Arduino GND    ->  Sabertooth 0V
Arduino GND    ->  AttoPilot GND
Arduino VIN    ->  Sabertooth 5V (OPTIONAL, Sabertooth powers Arduino)

*/

#include <Servo.h>

// Sabertooth control pins
// Sabertooth accepts servo pulses from 1000 us to 2000 us.
// We need to specify the pulse widths in attach(). 0 degrees will be full reverse, 180 degrees will be
// full forward. Sending a servo command of 90 will stop the motor. Whether the servo pulses control
// the motors individually or control throttle and turning depends on your mixed mode setting.
  
Servo ST1, ST2;

int VRaw; //This will store our raw ADC data
int IRaw;
float VFinal; //This will store the converted data
float IFinal;

void setup() 
{
  Serial.begin(9600);
  
    
// Notice these attach() calls. The second and third arguments are important.
// With a single argument, the range is 44 to 141 degrees, with 92 being stopped.
// With all three arguments, we can use 0 to 180 degrees, with 90 being stopped.
  ST1.attach( 9, 1000, 2000);
  ST2.attach(10, 1000, 2000);
}


void loop() 
{ 
  sabertooth();
  attoPilot();  
}

void attoPilot()
{
  //Measurement
  VRaw = analogRead(A0);
  IRaw = analogRead(A1);  // doesn't work ...
  
  //Conversion
  VFinal = VRaw/49.44; //45 Amp board
  
  IFinal = IRaw/14.9; //45 Amp board
    
  Serial.print(VFinal);
  Serial.println("   Volts");
  Serial.print(IFinal);
  Serial.println("   Amps");
  Serial.println("");
  Serial.println("");
  delay(200);   
}

void sabertooth()
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
