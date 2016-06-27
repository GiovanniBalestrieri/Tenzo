// include the SD library:
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"

RTC_DS1307 RTC;

File myFile;
File my1;
File folder;

uint8_t sd_answer;
const int chipSelect = 4;    

char path1[] = "arduino/file1";
char output[10];

unsigned long timer;
 
void setup()
{
 // Open serial communications and wait for port to open:
  Serial.begin(115200); 
  
  Wire.begin();
  RTC.begin();
  RTC.adjust(DateTime(__DATE__, __TIME__));
  
  Serial.print("\nInitializing SD card...");
  pinMode(53, OUTPUT); 
 
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Check SD:");
  } else {
   Serial.println("Wiring is correct and a card is present."); 
  }
  
  folder = SD.open("/arduino");
  printDirectory(folder,0);
  folder.close();
  
  myFile = SD.open(path1, FILE_WRITE);
  if (myFile)
  {
    Serial.println("Writing to file1.txt.");
    sd_answer = myFile.println("Init");
    myFile.close();
  }  

  folder = SD.open("/arduino");
  printDirectory(folder,0);
  folder.close();  
}

 
void loop(void) {   
  timer = millis();
  SerialRoutine(); 

  if (timer % 1000 == 0)
  {
    appendDate();
    Serial.println("END\n");
    timer = 0;
  }
}

void deleteFile(String path)
{
  if (SD.exists(path))
  {
    Serial.println("file1 exists. Delete");  
    SD.remove(path);      
    //sd_answer = myFile.println("\n\tNew Session: ");
  } else {
    Serial.println("file1 DOES NOT exist");    
  }
}

void SerialRoutine()
{
  if (Serial.available())
  {
    char t = Serial.read();
    if (t=='p')
    {
      readFile(path1);
    }
    else if (t == 'd')
    {
      deleteFile(path1);
    }
  }
}

void readFile(String path)
{
  Serial.println("\nContent of file:");
  myFile = SD.open(path);
  if (myFile)
  {
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close();
  }
}

void appendDate()
{
  DateTime now = RTC.now();

  myFile = SD.open(path1,FILE_WRITE);
  myFile.print(now.day());
  myFile.print('/');
  //We print the month
  myFile.print(now.month());
  myFile.print('/');
  //We print the year
  myFile.print(now.year());
  myFile.print(' ');
  //We print the hour
  myFile.print(now.hour());
  myFile.print(':');
  //We print the minutes
  myFile.print(now.minute());
  myFile.print(':');
  //We print the seconds
  myFile.print(now.second());
  myFile.println();
  myFile.close();  
}


void printDirectory(File dir, int numTabs)
{
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
