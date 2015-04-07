/*
AttoPilot Current and Voltage Sensing Demo
N.Poole, Sparkfun Electronics, 2011

"I don't care what you do with it, and neither does the script." (apathyware)

Physical Connections:
-------------------------
Arduino  | Peripherals
-------- | --------------
Pin 3  --- SerLCD "RX"
Pin A0 --- AttoPilot "V"
Pin A1 --- AttoPilot "I"
GND ------ AttoPilot "GND"
GND ------ SerLCD "GND"
5V ------- SerLCD "VCC"

This demo will read the Voltage and Current from the "AttoPilot Voltage and Current Sense Board,"
convert the raw ADC data to Volts and Amps and display them as floating point numbers on the
Serial Enabled LCD. (If you would like to do without the Serial LCD, I have included commented code
for reading the results through the Serial Terminal.)

*/

//Don't include this library if you are not using a Serial LCD
//#include <NewSoftSerial.h>
//NewSoftSerial LCD(2, 3);

int VRaw; //This will store our raw ADC data
int IRaw;
float VFinal; //This will store the converted data
float IFinal;

void setup() {

//LCD.begin(9600);
//backlightSerLcd(100);

/* Use this setup code instead if you want to read 
the data into the serial terminal.
*/
Serial.begin(9600);
}


void loop() { 
  //Cleanup for LCD (Don't include this line if you are 
  //using a serial terminal instead.
//  clearSerLcd();
  
  //Measurement
  VRaw = analogRead(A0);
  IRaw = analogRead(A1);
  Serial.print("analog");
  Serial.println(IRaw);
  //Conversion
  VFinal = VRaw/49.44; //45 Amp board
  //VFinal = VRaw/12.99; //90 Amp board
  //VFinal = VRaw/12.99; //180 Amp board  
  
  IFinal = IRaw/14.9; //45 Amp board
  //IFinal = IRaw/7.4; //90 Amp board
  //IFinal = IRaw/3.7; //180 Amp board

  /* 
  //Display
  LCD.print(VFinal);
  LCD.print("   Volts");
  LCD.print(0xFE, BYTE);
  LCD.print(192, BYTE);
  LCD.print(IFinal);
  LCD.print("   Amps");
  delay(200);
  */
  
  
  //Alternate Display code for terminal.
  //If you wish to use the terminal instead of an
  //LCD, use this display code instead of the above.
  
  Serial.print(VFinal);
  Serial.println("   Volts");
  Serial.print(IFinal);
  Serial.println("   Amps");
  Serial.println("");
  Serial.println("");
  delay(200);
  
  
}

//SerialLCD Functions, Don't include if you are not using an LCD
//These functions were borrowed from the Arduino Playground
//(http://www.arduino.cc/playground/Code/SfLCD2)

/*
void clearSerLcd(){
  LCD.print(0xFE, BYTE);   //command flag
  LCD.print(0x01, BYTE);   //clear command.
  delay(50);
}
*/

/*
void backlightSerLcd(int thePercentage){  //turns on the backlight
  LCD.print(0x7C, BYTE);   //command flag for backlight stuff
  int theValue = map(thePercentage, 0,100,128,157); // maps percentage to what SerLCD wants to see
  LCD.print(theValue, BYTE);    //light level.
  delay(50);  
}
*/



