#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

//uint8_t data[10];

#define MAX_LEN 100

boolean check = 0 ;

RH_RF95 driver;
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

unsigned char buffer[47]; // or worst case message size 70, 47: 2 mess

typedef struct mcTag {  // 8 bytes for versions < 6 it was 13 bytes
  unsigned char srcAddr;
  unsigned char dstAddr;
  unsigned char versionX;  //long
  unsigned char numCmds;
  unsigned char hdrLength;
  unsigned char cmdLength;
  unsigned char totalLen; // short
  unsigned char crc;  // short
}
MyControlHdr;

typedef struct ctrTag { // 17 bytes
  unsigned char cmd;
  long param1;
  long param2;
  long param3;
  long param4;
}
MyCommand;

MyControlHdr * pCtrlHdr = (MyControlHdr *)(&buffer[0]);

uint8_t data[] = "12.5";
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void setup() {

  Serial.begin(9600);
  while (!Serial) ;

  if (!manager.init()) {
    Serial.println("init failed");
  }
  driver.setTxPower(23, false);
  Serial.println("Ready");
}

void loop() {

  //Serial.println("\nSending to server\n");
  // Send a message to manager_server
  float a = 12.4, b = 13.2, c = 43.0;
  assembleAndSend(a,b,c);
  acquisisciMess();

  if ( check) {
    if (!manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS)) {
      Serial.println("Failed");
    }
    check = 0;
  }
  
  uint8_t len = sizeof(buf);
  uint8_t from;
  
  if (manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
    Serial.print(" Received: ");
    Serial.println((char*)buf);

   
  } 
  
  delay(500);
}

