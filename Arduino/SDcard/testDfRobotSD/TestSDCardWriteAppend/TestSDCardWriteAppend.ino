// include the SD library:
#include <SD.h>

File myFile;
File myFile1;

uint8_t sd_answer;
 
// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
const int chipSelect = 4;    
char testFile[] = "file1.txt";
char exFile[] = "file2.txt";

char output[10];

int cont = 0;
 
void setup()
{
 // Open serial communications and wait for port to open:
  Serial.begin(115200); 
 
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

  if (SD.exists(testFile))
  {
    Serial.println("TEST exists. Removing...");  
    SD.remove(testFile);  
  } else {
    Serial.println("TEST DOES NOT exist");    
  }

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

  
  myFile1 = SD.open(testFile, FILE_WRITE);
  if (myFile1)
  {
    Serial.println("Writing to file2.txt.");
    sd_answer = myFile1.println("ok init");// read from the file until there's nothing else in it:
    if (sd_answer == 1)
      Serial.println("\n.\n");
    myFile1.close();
  }
  
}
 
 
void loop(void) {   
  myFile = SD.open(exFile,FILE_WRITE);
  myFile1 = SD.open(testFile,FILE_WRITE);
  myFile.print(cont);
  myFile1.print(cont);
  myFile1.close();
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
  
 Serial.println("Reading file1.txt...");
  myFile1 = SD.open(testFile);
  if (myFile1)
  {
    // read from the file until there's nothing else in it:
    while (myFile1.available()) {
      Serial.write(myFile1.read());
    }
    myFile1.close();
  }
  
  delay(1000);
  Serial.println("END\n");

  cont++;
}
