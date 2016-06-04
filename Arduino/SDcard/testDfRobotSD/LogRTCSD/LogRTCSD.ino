// include the SD library:
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 RTC;

File myFile;
File myFile1;
File my1;
File my2;
File my3;

File folder;

uint8_t sd_answer;
 
// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
const int chipSelect = 4;    
char testFile[] = "file1.txt";
char exFile[] = "file2.txt";

char path1[] = "arduino/file1";
char output[10];

int cont = 0;
 
void setup()
{
 // Open serial communications and wait for port to open:
  Serial.begin(115200); 
  Wire.begin();
  RTC.begin();
  //If we remove the comment from the following line, we will set up the module time and date with the computer one
  RTC.adjust(DateTime(__DATE__, __TIME__));
  
  Serial.print("\nInitializing SD card...");
  pinMode(53, OUTPUT); 
 
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card is inserted?");
    Serial.println("* Is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    return;
  } else {
   Serial.println("Wiring is correct and a card is present."); 
  }

  
  folder = SD.open("/LOG");
  //root =
  //folder.ls(LS_R | LS_DATE | LS_SIZE);
  printDirectory(folder,0);
  folder.close();

  if (SD.exists(exFile))
  {
    Serial.println("file2 exists. Removing...");  
    SD.remove(exFile);      
  } else {
    Serial.println("file2 DOES NOT exist");    
  }
  
  myFile = SD.open(exFile, FILE_WRITE);
  if (myFile)
  {
    Serial.println("Writing to file1.txt.");
    sd_answer = myFile.println("aaaaaaaaaaaaaaaaaaaaaaaa"); 
    sd_answer = myFile.println("bbbbbbbbbbbbbbbbbbbbbbbb"); 
    sd_answer = myFile.println("cccccccccccccccccccccccc"); 
    sd_answer = myFile.println("dddddddddddddddddddddddd"); 
    sd_answer = myFile.println("eeeeeeeeeeeeeeeeeeeeeeee"); 
    if (sd_answer == 1)
      Serial.println("\n.\n");
    myFile.close();
  }  
  
  if (SD.exists(path1))
  {
    Serial.println("file2 exists. Removing...");  
    SD.remove(path1);      
  } else {
    Serial.println("file1 DOES NOT exist");    
  }
  my1 = SD.open(path1, FILE_WRITE); 
  //my2 = SD.open(path2, FILE_WRITE); 
  //my3 = SD.open(path3, FILE_WRITE); 
  my1.println("init 1");
  //my2.println("init 2");
  //my3.println("init 3");
    my1.close();
    //my2.close();
    //my3.close();

 
  
  folder = SD.open("/arduino");
  //root =
  //folder.ls(LS_R | LS_DATE | LS_SIZE);
  printDirectory(folder,0);
  folder.close();
  
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
 
void loop(void) {   

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
  
  
  Serial.println("Reading file2.txt...");
  myFile = SD.open(exFile);
  if (myFile)
  {
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close();
  }
 
 
  delay(1000);
  Serial.println("END\n");

  cont++;
}
