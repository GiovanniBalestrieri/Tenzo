#include "SIM900.h" 
#include <SoftwareSerial.h>
#include "inetGSM.h"
 
InetGSM inet;
 
int k=0; 
int j=0; 
char msg[150]; 
boolean found=false; 
char data; 
int numdata; 
char inSerial[50]; 
int i=0; 
boolean started=false;
 
void setup() 
{ 
  Serial.begin(9600);
  Serial.println("GSM Shield testing."); 
  if (gsm.begin(2400))
  { 
   Serial.println("\nstatus=READY"); 
   started=true; 
  }
  else
  Serial.println("\nstatus=IDLE");
  
  if(started)
  {
    if (inet.attachGPRS("internet.wind", "", ""))
      Serial.println("status=ATTACHED");
    else
      Serial.println("status=ERROR");
    delay(1000);
    numdata=inet.httpPOST("www.evildeejay.it", 80, "/test/test.php", "name=Marco&age=24",msg, 50);
  } 
};
 
void loop() 
{ 
serialswread(); 
};

void serialswread()
{ 
  gsm.SimpleRead(); 
}
