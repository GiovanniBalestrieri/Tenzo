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
      Serial.println(" TakeOff ");
    }
    if (modeS == 'L')
    {
      Serial.println(" Land ");
    }
    if (modeS == 'p')
    {
      Serial.println(" Pid ");
    }
    if (modeS == 't')
    {
      Serial.println("o,12,1,-4,");
    }
    if (modeS == 's')
    {
      Serial.println("c,p,");
    }
    
//    Serial.print(" Writing Blu: ");
//    Serial.println(modeS);
    //bluetooth.print(modeS);
  }
}
