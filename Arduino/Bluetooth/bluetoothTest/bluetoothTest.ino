#include <SoftwareSerial.h>

//int bluetoothTx = 10;
//int bluetoothRx = 11;

//SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

void setup()
{
  //Setup usb serial connection to computer
  Serial.begin(115200);
//  //Setup Bluetooth serial connection to android
//  bluetooth.begin(115200);
//  bluetooth.print("$$$");
//  delay(100);
//  bluetooth.println("U,9600,N");
//  bluetooth.begin(9600);
  Serial.println("Setup completed");
}

void loop()
{
  //Read from bluetooth and write to usb serial
  //if(bluetooth.available())
  //{
    /*
    char toSend = (char)bluetooth.read(); 
    Serial.print(" Receiving: ");
    Serial.println(toSend);
  }*/

  //Read from usb serial to bluetooth
  if(Serial.available())
  {
    char modeS = Serial.read(); 
    
    if (modeS == 'a')
    {
      Serial.print(" TakeOff ");
    }
    if (modeS == 'L')
    {
      Serial.print(" Land ");
    }
    if (modeS == 'p')
    {
      Serial.print(" Pid ");
    }
    
//    Serial.print(" Writing Blu: ");
//    Serial.println(modeS);
    //bluetooth.print(modeS);
  }
}
