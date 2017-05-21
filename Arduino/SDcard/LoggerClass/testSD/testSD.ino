#include "Logs.h"

RTC_DS1307 RTC;
DateTime now;

// Init Log
Logs logger = Logs();

unsigned long timer;
File myFile;
uint8_t sd_answer;


void setup() {
  Serial.begin(115200);
  Serial.println("Welcome\n");
  RTC.begin();
  Serial.println("Welcome\n");
  //RTC.adjust(DateTime(__DATE__, __TIME__));
  //Serial.println("Welcome\n");
  // put your setup code here, to run once:

  boolean tmp;
  tmp = logger.init();
  Serial.println("Welcome\n");
  if (tmp)
    Serial.println("[ OK ] LOG");
  else      
    Serial.println("[ KO ] SDCARD\nContinuing without log:");

  logger.logSession();
}

void loop() 
{   
  timer = millis();
  SerialRoutine(); 

  if (timer % 1000 == 0)
  {
    appendDate(logPath);
    Serial.println("END\n");
    logger.logStates(0,0,1);
    logger.logStates(1,1,1);
    timer = 0;
  }
}


void deleteFile(String path)
{
  if (SD.exists(path))
  {
    Serial.println("file exists. Delete");  
    SD.remove(path);      
    //sd_answer = myFile.println("\n\tNew Session: ");
  } else {
    Serial.println("file1 DOES NOT exist");    
  }

  myFile = SD.open(path, FILE_WRITE);
  if (myFile)
  {
    Serial.println("File created");
    sd_answer = myFile.println("DELETED");
    sd_answer = myFile.println(myFile);
    myFile.close();
  }  
  else
  {
    Serial.println("file1.txt NON Esiste");
  }
}

void SerialRoutine()
{
  if (Serial.available())
  {
    char t = Serial.read();
    if (t=='p')
    {
      readFile(logPath);
    }
    else if (t == 'd')
    {
      deleteFile(logPath);
    }
    else if (t == 'l')
    {
      Serial.println();
      Serial.println(logger.checkFileLog());
    }
    else if (t == 'w')
    {
      Serial.println();
      Serial.println(logger.checkFileWcet());
    }
    else if (t == 'e')
    {
      Serial.println();
      Serial.println(logger.checkFileError());
    }
    else if (t == 'n')
    {
      Serial.println();
      Serial.println(logger.checkFileWarning());
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

void appendDate(String path)
{
  DateTime now = RTC.now();

  myFile = SD.open(path,FILE_WRITE);
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

