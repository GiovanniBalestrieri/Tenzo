#include <SoftwareSerial.h>

int bluetoothTx = 13;
int bluetoothRx = 12;

SoftwareSerial bluetooth(bluetoothRx, bluetoothTx);

char GotChar;
byte getData;

void setup()
{
  Serial.begin(9600);  // Begin the serial monitor at 9600bps
  Serial.println("Testing Bluetooth Mate Gold");
  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  //bluetooth.print("$");  // Print three times individually
  //bluetooth.print("$");
  //bluetooth.print("$");  // Enter command mode
  //delay(100);  // Short delay, wait for the Mate to send back CMD
  //bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  //bluetooth.begin(9600);  // Start bluetooth serial at 9600
  int cont =0;
  Serial.flush();
  bluetooth.flush();
}

void loop()
{
  //Read from usb serial to bluetooth
  if(Serial.available())
  {
    GotChar = Serial.read();
    Serial.println();
    Serial.print("Sending ");
    Serial.print(GotChar);
    Serial.println(" to Bluetooth");
    bluetooth.write(GotChar);
  }
  
  //Read from bluetooth and write to usb serial
  if(bluetooth.available())
  {    
    //getData =  bluetooth.read();
    char getData =  bluetooth.read();
    Serial.println(getData);
//    Serial.print("Receiving [byte]");
//    Serial.print(getData);
//    Serial.print(" [char] ");    
//    Serial.print((char) getData);
    Serial.println(" from Bluetooth");
  }
  
  //bluetooth.print(cont);
  //delay(1000);
  //cont ++;
}
