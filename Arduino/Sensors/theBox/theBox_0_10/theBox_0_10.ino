/*********************************************************************/
/*                             - TheBox -                            */
/*                                                                   */
/*             DHT22 - Temperature and humidity sensor               */
/*                       uSD reader logging sys                      */
/*                                Lux                                */
/*                                                                   */
/*             Contacts:                                             */
/*                      Alessio Cavone Aka Champion                  */
/*                         Ivan B  Aka KillerBM                      */
/*                            Gepp Aka UserK                         */
/*                                                                   */
/*             PinOUT  SD CARD                                       */
/*                        Vcc  - 3.3                                 */
/*                     Miso/DO - 12                                  */
/*                     Mosi/DI - 11                                  */
/*                         SCK - 13                                  */
/*                          CS - 4                                   */
/*             PinOUT  DHT22                                         */
/*                        Vcc  - 5 V                                 */
/*                         DATA - 2                                  */
/*                                                                   */
/*********************************************************************/

#include "DHT.h"
#include <SPI.h>
#include <SD.h>

File myFile;

#define DHTPIN 2     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

DHT dht(DHTPIN, DHTTYPE);

int luxVal;
float temperature,humidity;
boolean printCSV = false;
boolean printSerial = false;
boolean verbosity = false;
boolean verbosityLevel = 5;
boolean printSD = false;

const int chipSelect = 10;

// Strings Log
char temp[25];
char hum[25];
char lux[25];
char dio[25];

String tempT;
String luxT;
String humT;
String dioT;



void setupDht()
{
  dht.begin();
  pinMode(0,INPUT);
}

void setupSD()
{
  Serial.print("Initializing SD card...");
  pinMode(chipSelect, OUTPUT);
  if (!SD.begin(4)) 
  {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("[OK]");
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void setup() 
{
  Serial.begin(115200);
  setupDht();
  setupSD();
}

void loop()
{
  readLux();
  getTempHum();
  printData();
  SerialRoutine();
  writeSdToRoutine();
}

// #SD #SDWRITE
void writeSdToRoutine()
{  
  if (printSD)
  {
    initLog();
  
    sdWrite("[Temperature]     ");
    sdWrite(tempT);
    sdWrite("        [Humidity] ");
    sdWrite(humT);
    sdWrite("        [LUX]      ");
    sdWrite(luxT);
    sdWrite("\n");
    // timerSec = micros()-secRoutine;
    // lastTimeToRead = micros();
    readSdFile();
  }
}

// # LUX
void readLux()
{
  luxVal = analogRead(0);  
  if (verbosityLevel > 3)
     Serial.println("Ok LUx");
}

// #TEMP #HUMIDITY #HUM
void getTempHum()
{
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();  
  if (verbosityLevel > 3)
     Serial.println("Ok Temp");
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
  Serial.println(luxVal);  
}

void printCSVData()
{
  Serial.print("H,");
  Serial.print(humidity);
  Serial.print(",");
  Serial.print(temperature);
  Serial.print(" , ");
  Serial.println(luxVal);
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
    else if (t == '+')
    {
      verbosityLevel++;
      Serial.print("- Verbosity level changed :]");
      Serial.print(verbosityLevel);
    }    
    else if (t == '-')
    {
      verbosityLevel--;
      Serial.print("- Verbosity level changed :]");
      Serial.print(verbosityLevel);
    }    
    else if (t == 'p')
    {
      Serial.println("- Toggle Verbosity level ... [OK]");
      verbosity = !verbosity;
    }      
    else if (t == 's')
    {
      Serial.println("- Toggle SDCARD verbosity ... [OK]");
      printSD = !printSD;
    }    
  }
} 

void initLog()
{
  tempT = dtostrf(temperature,5,2,temp);
  luxT = dtostrf(luxVal,5,2,lux);
  dioT = dtostrf(humidity,5,2,dio);
  humT = dtostrf(humidity,5,2,hum);
}

