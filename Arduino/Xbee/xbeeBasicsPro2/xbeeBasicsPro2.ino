/**
 *  Wiring on Arduino UNO
 *  Connect Dout to Pin 2 (Arduino pin(2) Rx , Dout)
 *  Connect Din to Pin 4 (Arduino pin(4) Tx , Din)
 *  Wiring on Arduino Mega 
 *  Connect Dout to Pin 12 (Arduino pin(12) Rx , Dout)
 *  Connect Din to Pin 13 (Arduino pin(13) Tx , Din)
 *
 *
 *  Comments:
 *  Arduino does not have a function to convert a 32-bit
 *  long value into two 16-bit words but you can add your
 *  own makeLong() capability by adding the following line
 *  to the top of your sketch:
 **/

#define makeLong(hi, low)  ((hi) << 16 & (low))
#define highWord(w) ((w) >> 16)
#define lowWord(w) ((w) & 0xffff)

#include <SoftwareSerial.h>

uint8_t pinRx = 12 , pinTx = 13; // the pin on Arduino
long BaudRateSerial = 9600;
long BaudRateXbee = 19200;
char GotChar;
byte getData;

byte loBytew1, hiBytew1,loBytew2, hiBytew2;
int loWord,hiWord;

// Xbee SoftwareSerial initialization
SoftwareSerial xbee(pinRx, pinTx); // RX, TX
int buffSize = 33;
byte bufferBytes[33];

// Serial Protocol
int versionArduinoProtocol = 1;
long versionProtocol;
long cmd1;
long cmd2;
long cmd3;
long cmd4;

typedef struct mcTag {
     unsigned char srcAddr;
     unsigned char dstAddr;
     unsigned long versionX;
     unsigned char numCmds;
     unsigned char hdrLength;
     unsigned char cmdLength;
     unsigned short totalLen;
     unsigned short crc; } MyControlHdr;

  typedef struct ctrTag {
    unsigned char cmd;
    long param1;
    long param2;
    long param3;
    long param4;   } MyCommand;

  unsigned char buffer[255];  // or worst case message size
    
    int cont =0;
  MyControlHdr * pCtrlHdr = (MyControlHdr *)(&buffer[0]);

void setup() 
{
  Serial.begin(BaudRateSerial);
  Serial.println( "Welcome to the XBee Communication Test" );
  Serial.print("BaudRate:");
  Serial.println(BaudRateXbee);
  Serial.print(" Rx Pin#");
  Serial.println(pinRx,DEC);
  Serial.print(" Tx Pin#");
  Serial.println(pinTx,DEC);
  // set the data rate for the SoftwareSerial port
  xbee.begin( BaudRateXbee );
  xbee.println("Setup Completed!");
}

void loop() 
{
  // Reads from serial monitor 
  if (Serial.available()) 
  {
    GotChar = Serial.read();
    // sends char to xbee
    xbee.print(GotChar);
  }
  
  if(xbee.available()>0)
  {
    if (xbee.available() >= buffSize)
    {     
      for (int j=0;j<buffSize;j++)
      {
        bufferBytes[j]=xbee.read();
        Serial.print("Received: ");
        Serial.print(bufferBytes[j]);
        Serial.println();
      }
      
      if (bufferBytes[0] == 2 && bufferBytes[1]==1)
      {
         /**
           * Message has been correctly sent by MATLAB and delivered to the Arduino 
           * Decoding Header
           **/
         
         // Assembling VERSION long skip first two bytes
         
         //hiBytew1 = bufferBytes[2];
         //Serial.println(hiBytew1,BIN);
         //loBytew1 = bufferBytes[3];
         //Serial.println(loBytew1,BIN);
         //loWord = word(hiBytew1, loBytew1);
         //Serial.println(loWord,BIN);
         hiBytew2 = bufferBytes[4];
         //Serial.println(hiBytew2,BIN);
         loBytew2 = bufferBytes[5];
         //Serial.println(loBytew2,BIN);
         hiWord = word(hiBytew2, loBytew2);
         //versionProtocol = makeLong( hiWord, loWord);
         versionProtocol = hiWord;
         Serial.println();
         Serial.print("Version");
         Serial.print(versionProtocol);
         if (versionProtocol != versionArduinoProtocol)
         Serial.println("Warning Sync Repos, different serial protocols");
         
         //  number cmd
         int numCmd = bufferBytes[6];
         int headL = bufferBytes[7];
         int cmdL = bufferBytes[8];
         //  total Length
         hiBytew2 = bufferBytes[9];
         //Serial.println(hiBytew2,BIN);
         loBytew2 = bufferBytes[10];
         //Serial.println(loBytew2,BIN);
         short totL = word(hiBytew2, loBytew2);
         Serial.println();
         Serial.print("Tot Len");
         Serial.println(totL);
         // CRC
         hiBytew2 = bufferBytes[11];
         //Serial.println(hiBytew2,BIN);
         loBytew2 = bufferBytes[12];
         //Serial.println(loBytew2,BIN);
         short crc = word(hiBytew2, loBytew2);
         Serial.println();
         Serial.print("Crc");
         Serial.println(crc);
         
         /**
           * Decoding Command
           **/
         int type = bufferBytes[13];
         Serial.println();
         Serial.print("typeCMD: ");
         Serial.print(type);
         //  CMD1
         hiBytew2 = bufferBytes[16];
         //Serial.println(hiBytew2,BIN);
         loBytew2 = bufferBytes[17];
         //Serial.println(loBytew2,BIN);
         cmd1 = word(hiBytew2, loBytew2);
         Serial.println();
         Serial.print("CMD1: ");
         Serial.println(cmd1);
         
         //  CMD2
         hiBytew2 = bufferBytes[20];
         //Serial.println(hiBytew2,BIN);
         loBytew2 = bufferBytes[21];
         //Serial.println(loBytew2,BIN);
         cmd2 = word(hiBytew2, loBytew2);
         Serial.println();
         Serial.print("CMD2: ");
         Serial.println(cmd2);
         
         //  CMD3
         hiBytew2 = bufferBytes[24];
         //Serial.println(hiBytew2,BIN);
         loBytew2 = bufferBytes[25];
         //Serial.println(loBytew2,BIN);
         cmd3 = word(hiBytew2, loBytew2);
         Serial.println();
         Serial.print("CMD3: ");
         Serial.println(cmd3);
         
         //  CMD4
         hiBytew2 = bufferBytes[28];
         //Serial.println(hiBytew2,BIN);
         loBytew2 = bufferBytes[29];
         //Serial.println(loBytew2,BIN);
         cmd4 = word(hiBytew2, loBytew2);
         Serial.println();
         Serial.print("CMD4: ");
         Serial.println(cmd4);
         
         
      }    
    }
    else  
    {
      getData = xbee.read();  
          
      /*
      Serial.println();
      Serial.print("Received: ");
      Serial.print(getData);
      Serial.print("         Bin:  ");
      Serial.print(getData,BIN);
      Serial.print("         Hex:  ");
      Serial.print(getData,HEX);
      Serial.print("         cont:  ");
      Serial.print(cont);
      Serial.println();
      */
      if(getData == 16)
      {       	 
        byte message[] = {0x11};
        xbee.write(message, sizeof(message));
        Serial.println();
        Serial.print(" Sending to Matlab:  ");
        Serial.println();
        Serial.print(message[0]);      
      }
      else if (getData == 18)
      {
        Serial.println(" Matlab communication established. ");
      }
      else if(getData == 19)
      {  	 
        Serial.println(" [Warning] Matlab communication problem. ");
      }
      else if(getData == 20)
      {  	 
        byte message[] = {0x15};
        xbee.write(message, sizeof(message));
        Serial.println();
        Serial.print(" Sending to Matlab:  ");
        Serial.println(message[0]);  
        Serial.println(" Disconnected. ");
        Serial.println(); 
      }
      else if (getData == 23)
      {      
        MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
        // first command in this message
        pMyCmd->cmd = 1;      // indicates communication
        pMyCmd->param1 = 10;  // 
        pMyCmd->param2 =11;
        pMyCmd->param3 =12;
        pMyCmd->param4 =1300;
        
        pMyCmd++;           // moves pointer to next command position in message
        pMyCmd->cmd = 2;      // indicates communication
        pMyCmd->param1 = 20;  // 
        pMyCmd->param2 =21;
        pMyCmd->param3 =22;
        pMyCmd->param4 =2300;
        
        pMyCmd++;           // moves pointer to next command position in message
        pMyCmd->cmd = 3;      // indicates communication
        pMyCmd->param1 = 20;  // 
        pMyCmd->param2 =21;
        pMyCmd->param3 =22;
        pMyCmd->param4 =2300;
        //pCtrlHdr->crc = calcCrc(buffer, pCtrlHdr->totalLen); // crc function
        Serial.println();
        Serial.print("size of command: ");
        Serial.print(sizeof(MyCommand));
        Serial.print("numCmd: ");
        Serial.print(sizeof(MyCommand)/17);
        Serial.println();
        
        sendCommand();
      }
    }
    cont++;   
  }
}

void sendCommand()
{
  // Build Header
  pCtrlHdr->srcAddr = 1;
  pCtrlHdr->dstAddr = 2;    // maybe you'll make 2555 a broadcast address? 
  pCtrlHdr->versionX = 13;    // possible way to let a receiver know a code version
  pCtrlHdr->numCmds = 4;    // how many commands will be in the message
  pCtrlHdr->hdrLength = sizeof(MyControlHdr );  // tells receiver where commands start
  pCtrlHdr->cmdLength = sizeof(MyCommand );     // tells receiver size of each command 
  // include total length of entire message
  pCtrlHdr->totalLen= sizeof(MyControlHdr ) + (sizeof(MyCommand) * pCtrlHdr->numCmds); 
  pCtrlHdr->crc = 21;   // dummy temp value
 
  for (int v=0;v<=sizeof(buffer);v++)
  xbee.write(buffer[v]);  
}

//CRC-8 - algoritmo basato sulle formule di CRC-8 di Dallas/Maxim
//codice pubblicato sotto licenza GNU GPL 3.0
byte CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}
