// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

#include "DHT.h"
#include <SPI.h>
#include <SD.h>

File myFile;

#define DHTPIN 2     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

DHT dht(DHTPIN, DHTTYPE);

int lux;
float temperature,humidity;
boolean printCSV = false;
boolean printSerial = false;
boolean verbosity = false;

const int chipSelect = 10;

void setup() 
{
  Serial.begin(115200);
  setupDht();
  setupSD();
}

void setupDht()
{
  dht.begin();
  pinMode(0,INPUT);
}

void setupSD()
{
  Serial.print("Initializing SD card...");
  pinMode(chipSelect, OUTPUT);
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("[OK]");
  while (!Serial) 
 {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop()
{
  readLux();
  getTempHum();
  printData();
  SerialRoutine();
  writeSdToRoutine();
}

void writeSdToRoutine()
{  
  sdWrite("[Temperature]     ");
  sdWrite(String(temperature,5));
  sdWrite("        [Humidity] ");
  sdWrite(String(humidity,5));
  sdWrite("        [LUX]      ");
  sdWrite(String(lux,5));
  sdWrite("\n");
  // timerSec = micros()-secRoutine;
  // lastTimeToRead = micros();
  readSdFile();
}

void readLux()
{
  lux = analogRead(0);  
}

void getTempHum()
{
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();  
}

void printData()
{
  if (verbosity)
  {
    if (printSerial)
    {
      printNormal();
    }
    else if (printCSV)
    {
      printCSVData();
    }
  }
}

void printNormal()
{
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" LUX:  ");
  Serial.println(lux);  
}

void printCSVData()
{
  Serial.print("H,");
  Serial.print(humidity);
  Serial.print(",");
  Serial.print(temperature);
  Serial.print(" , ");
  Serial.println(lux);
  Serial.print(",");
  Serial.print("Z");
}

void sdWrite(String message)
{
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) 
  {
    myFile.println(message);
    // close the file:
    myFile.close();
    Serial.println("Wrote to file.");
  } 
  else 
  {
    // if the file didn't open, print an error:
    Serial.println("ERROR opening test.txt");
  }   
}

void readSdFile()
{
  // re-open the file for reading:
  myFile = SD.open("test.txt");
  if (myFile) 
  {
    Serial.println();
    Serial.println("[Content] :");
    Serial.println();

    // read from the file until there's nothing else in it:
    while (myFile.available()) 
    {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
    Serial.println();
    Serial.println();
  } else 
  {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void SerialRoutine()
{
  // TODO 
  if (Serial.available())
  {
    Serial.println(" received ");
    char t = Serial.read();
    if (t == 'c')
    {
      Serial.println("- Toggle CSV mode ... [OK]");
      printCSV = !printCSV;
    }
    else if (t == 'n')
    {
      Serial.println("- Toggle Normal mode ... [OK]");
      printSerial = !printSerial;
    }    
    else if (t == 'p')
    {
      Serial.println("- Toggle Verbosity level ... [OK]");
      verbosity = !verbosity;
    }    
  }
}



