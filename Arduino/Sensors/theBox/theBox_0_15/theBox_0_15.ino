/*********************************************************************/
/*                             - TheBox -                            */
/*                                                                   */
/*             DHT22 - Temperature and humidity sensor               */
/*                       uSD reader logging sys                      */
/*                                Lux                                */
/*                                                                   */
/*             Contacts:                                             */
/*                      Alessio Aka BrainDev                         */
/*                      Ivan B  Aka KillerBM                         */
/*                         Gepp Aka UserK                            */
/*                                                                   */
/*             PinOUT  SD CARD                                       */
/*                        Vcc  - 3.3                                 */
/*                     Miso/DO - 12                                  */
/*                     Mosi/DI - 11                                  */
/*                         SCK - 13                                  */
/*                          CS - 4                                   */
/*                                                                   */
/*             PinOUT  DHT22                                         */
/*                        Vcc  - 5 V                                 */
/*                         DATA - 2                                  */
/*                                                                   */
/*             Photoresistance                                       */
/*                        A0  -  lux                                 */
/*                                                                   */
/*                                                                   */
/*********************************************************************/
/*                                                                   */
/*                                                                   */
/*           TODO:                                                   */
/*                                                                   */
/*                                                                   */
/*********************************************************************/

#include "DHT.h"
#include <SPI.h>
#include <SoftwareSerial.h>

SoftwareSerial blu(10, 11); // RX, TX 

// Update Current Version
#define VERSION 1.25

#define LUX 0
#define HUM 1
#define TMP 2

#define DHTPIN 4     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

DHT dht(DHTPIN, DHTTYPE);

int luxVal;
float temperature,humidity;

boolean printCSV = false;
boolean printSerial = false;
boolean verbosity = true;
int verbosityLevel = 2;
boolean printSD = false;

const int chipSelect = 10;

String tempT;
String luxT;
String humT;
String timeT;
String messageToSdWrite;

// Timers Countera
long timerLoop, timer0, timer1, timer2;
int TLperiod = 5;
unsigned int periodeTimerLoopMillis = 0;

// Periods
unsigned int SdTimerPeriod = 5000;
unsigned int sensorPeriod = 60000; // 1 min
unsigned int sensorPeriodLong = 5*60000; // 5


// Define #history #HST
float historyTmp[40];
float historyHum[40];
float historyLux[40];
int histSize = 15;
int countHistLux = 0;
int countHistTmp = 0;
int countHistHum = 0;

void setupDht()
{
  dht.begin();
  pinMode(0,INPUT);
}

void initTimer()
{
  timerLoop = 0; 
  timer0 = 0;
  timer1 = 0;
  timer2 = 0;
}

void setupBlu()
{
 Serial.println("Enter AT commands:");
 blu.begin(9600);  // HC-05 default speed in AT command more 
}

void setup() 
{
  setupBlu();
  Serial.begin(115200);
  Serial.print("Welcome to Fujin v: ");
  blu.print("Welcome to Fujin v: ");
  Serial.println(VERSION);
  blu.println(VERSION);
  setupDht();
  //setupSD();
  
  //setupEsp();
  // Yhis should be the last one
  initTimer();
}

void loop()
{
  updateTimer();
  sensorRoutine();
  SerialRoutine();
}

double convertSec2millisTimerL(int a)
{
  periodeTimerLoopMillis = 1000*a;
  return periodeTimerLoopMillis;
}

int convertMillis2Min(float a)
{
  return  a/1000;
}

double convertMin2Millis(double a)
{
  return a*1000;
}

//  #SENSOR #ROUTINE
void sensorRoutine()
{
  convertSec2millisTimerL(TLperiod);
  if ( timerLoop % periodeTimerLoopMillis == 0)
  {
    readLux();
    getTempHum();
    printData();
  }
  
  if ( timerLoop % SdTimerPeriod == 0)
  {
    //writeSdToRoutine();
  }
  
  if ( timerLoop % sensorPeriod == 0)
  {
    // 
    add2history(luxVal,LUX);
    add2history(humidity,HUM);
    add2history(temperature,TMP);
  }
}

// Update timers
void updateTimer()
{
  timerLoop = millis();
  if (verbosityLevel >=8)
  {
    Serial.println("[Sakura] Timer updates");
  }
}

void add2history(float data,int value)
{
 if (value == LUX)
 {
  if (verbosityLevel >= 3)
  {
   Serial.print("L,");
   Serial.print(data);
   Serial.print(",");
   Serial.println(countHistLux);
  
   blu.print("L,");
   blu.print(data);
   blu.print(",");
   blu.println(countHistLux);
  }
   
  if (countHistLux==histSize)
  {   
    // from the 2nd element of the array
    for (int i=1;i<histSize;i++)
    {
       historyLux[i-1] = historyLux[i];
    }
  }     
  
  historyLux[countHistLux-1] = data;
   
  if (countHistLux < histSize)
    countHistLux++;
 } 
 else if ( value == TMP)
 {
    if (verbosityLevel >= 3)
    {
     Serial.print("T,");
     Serial.print(data);
     Serial.print(",");
     Serial.println(countHistTmp);
     
     blu.print("T,");
     blu.print(data);
     blu.print(",");
     blu.println(countHistTmp);
    }
   
   if (countHistTmp==histSize)
   {     
     // from the 2nd element of the array
     for (int i=1;i<histSize;i++)
     {
       historyTmp[i-1] = historyTmp[i];
     }
   }
     
   // add to tmp array
   if(countHistTmp==histSize)
     historyTmp[countHistTmp-1] = data;
   else if(countHistTmp<histSize)
     historyTmp[countHistTmp] = data;
          
   if (countHistTmp < histSize)
     countHistTmp++;
 } 
 else if (value == HUM)
 {
    if (verbosityLevel >= 3)
    {
     Serial.print("H,");
     Serial.print(data);
     Serial.print(",");
     Serial.println(countHistHum);
     
     
     blu.print("H,");
     blu.print(data);
     blu.print(",");
     blu.println(countHistHum);
    }
   
   if (countHistHum==histSize)
   {     
     // from the 2nd element of the array
     for (int i=1;i<histSize;i++)
     {
       historyHum[i-1] = historyHum[i];
     }
   }
   // add to humid array
   historyHum[countHistHum-1] = data;
      
   if (countHistHum < histSize)
     countHistHum++;   
 }
}

// # LUX
void readLux()
{
  luxVal = analogRead(0); 
  if (verbosityLevel >= 3)
     Serial.println("Ok LUx");     
}

// #TEMP #HUMIDITY #HUM
void getTempHum()
{
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  if (verbosityLevel >= 3)
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
  
  
  blu.print("Humidity: ");
  blu.print(humidity);
  blu.print(" %\t");
  blu.print("Temperature: ");
  blu.print(temperature);
  blu.print(" LUX:  ");
  blu.println(luxVal);  
}

void printCSVData()
{
  Serial.print("H,");
  Serial.print(humidity);
  Serial.print(",");
  Serial.print(temperature);
  Serial.print(",");
  Serial.print(luxVal);
  Serial.print(",");
  Serial.println("Z");
  
  
  blu.print("H,");
  blu.print(humidity);
  blu.print(",");
  blu.print(temperature);
  blu.print(",");
  blu.print(luxVal);
  blu.println(",Z");
}

// #SERIAL
void SerialRoutine()
{
  // TODO 
  if (Serial.available() || blu.available())
  {
    char t = Serial.read();
    char b = blu.read();
    if (t == 'c' || b == 'c')
    {
      printCSV = !printCSV;
      if (printCSV)
      {
        Serial.println("- CSV mode = TRUE");
        blu.println("- CSV mode = TRUE");
      }
      else
      {
        Serial.println("- CSV mode = FALSE");
        blu.println("- CSV mode = FALSE");
      }
    }
    else if (t == 'n' || b == 'n')
    {
      printSerial = !printSerial;
      if (printSerial)
      {
        Serial.println("- Normal mode = TRUE");
        blu.println("- Normal mode = TRUE");
      }
      else
      {
        Serial.println("- Normal mode = FALSE");
        blu.println("- Normal mode = FALSE");
      }
    }
    else if (t == '+' || b == '+')
    {
      verbosityLevel++;
      Serial.println("- Verbosity level changed :]\n");
      Serial.print(verbosityLevel);
    }    
    else if (t == 'e')
    {
      //setupEsp();
    }    
    else if (t == '-' || b == '-')
    {
      verbosityLevel--;
      Serial.println("- Verbosity level changed :]\n");
      Serial.print(verbosityLevel);
    }    
    else if (t == 'p' || b == 'p')
    {
      verbosity = !verbosity;
      if (verbosity)
        Serial.println("- Verbosity = TRUE");
      else
        Serial.println("- Verbosity = FALSE");
    }      
    else if (t == 's' || b == 's')
    {
      Serial.println("- Toggle SDCARD verbosity ... [OK]");
      printSD = !printSD;
      if (printSD)
        Serial.println("- SD = TRUE");
      else
        Serial.println("- SD = FALSE");
    }
    else if (t == 'h' || b == 'h')
    {      
      Serial.println("\n");      
      Serial.println("Help:");
      Serial.println("Fujin returns three float values in order:");
      Serial.println("Temperature - Humidity - Lux");
      Serial.println("");
      
      Serial.println("+/- : adjust verbosity level");
      Serial.println("c : toggle CSV mode");
      Serial.println("n : toggle Normal mode");
      Serial.println("s : toggle SD verbosity");
      Serial.println("p : toggle general verbosity");   
      Serial.println();
      Serial.println("l: print LUX history");
      Serial.println("t: print TMP history");
      Serial.println("h: print HUM history");
      
      
      blu.println("\n");      
      blu.println("Help:");
      blu.println("Fujin returns three float values in order:");
      blu.println("Temperature - Humidity - Lux");
      blu.println("");
      
      blu.println("+/- : adjust verbosity level");
      blu.println("c : toggle CSV mode");
      blu.println("n : toggle Normal mode");
      blu.println("s : toggle SD verbosity");
      blu.println("p : toggle general verbosity");   
      blu.println();
      blu.println("l: print LUX history");
      blu.println("t: print TMP history");
      blu.println("h: print HUM history");
      
    }
    else if (t == 't' || b == 't')
    {
      for (int i=0;i<histSize;i++)
      {
        Serial.println(historyTmp[i]);
        blu.println(historyTmp[i]);
      }
    }
    else if (t == 'y' || b == 'y')
    {
      for (int i=0;i<histSize;i++)
      {
        Serial.println(historyHum[i]);
        blu.println(historyHum[i]);
      }
    }
    else if (t == 'l' || b == 'l')
    {
      for (int i=0;i<histSize;i++)
      {
        Serial.println(historyLux[i]);
        blu.println(historyLux[i]);
      }
    }  
    else if (t == 'v' || b == 'v')
    {
      storeThePast();
    }  
    else if (t == 'a' || b == 'a')
    {
      Serial.println();
      Serial.print("Temp ");
      Serial.print("\t-\t");
      Serial.print("Humidity");
      Serial.print("\t-\t");
      Serial.println("Lux");
      Serial.println();
      for (int i=0;i<histSize;i++)
      {
        Serial.print(historyTmp[i]);
        Serial.print("\t-\t");
        Serial.print(historyHum[i]);
        Serial.print("\t-\t");
        Serial.println(historyLux[i]);
      }
      
      
      blu.println();
      blu.print("Temp ");
      blu.print("\t-\t");
      blu.print("Humidity");
      blu.print("\t-\t");
      blu.println("Lux");
      blu.println();
      for (int i=0;i<histSize;i++)
      {
        blu.print(historyTmp[i]);
        blu.print("\t-\t");
        blu.print(historyHum[i]);
        blu.print("\t-\t");
        blu.println(historyLux[i]);
      }      
    }  
  }
}

// TODO resituisci media degli ultimi tre minuti, ultimi 10, mezz'ora, ora, 6 ore.Test fattibilità
void storeThePast()
{
  /*
  for (int i=0;i<histSize;i++)
  {
    Serial.print(historyTmp[i]);
    Serial.print("\t-\t");
    Serial.print(historyHum[i]);
    Serial.print("\t-\t");
    Serial.println(historyLux[i]);
  }  
  */
}
