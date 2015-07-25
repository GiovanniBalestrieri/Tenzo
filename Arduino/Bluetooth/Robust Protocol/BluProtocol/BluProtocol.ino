/*******************************************************************
 *             Arduino & Bluetooth Mate Gold Module                *
 *                    by Balestrieri Giovanni                      *
 *                       AKA UserK, 2013                           *
 *******************************************************************/



#include <SoftwareSerial.h>


boolean printBlue = false;
boolean processing = false;
boolean printMotorsVals = false;
boolean printPIDVals = false;
boolean printSerialInfo = false;
boolean printSerial = false;
boolean printTimers = false; // true
boolean printAccs = false;
boolean printOmegas = false;
boolean sendBlueAngle = false;
boolean printVerboseSerial = false;

/**
 * Print sensor's value
 */ 
boolean printSetupInfo = true;
boolean printAckCommands = true;
boolean printRawKalman= false;
boolean printRawAcc = false;
boolean printRawGyro= false;
boolean printGyro = false;
boolean printRawCompass= false;
boolean printCompass = false;
boolean printKalman = false; 
boolean printThrottle = false;


byte loBytew1, hiBytew1,loBytew2, hiBytew2;

/**
 * Modes
 */
int connAck = 0;
int takeOff = 0;
int hovering = 0;
int landed = 1;
int tracking = 0;
int warning = 0;

byte modeS;
byte mode;
boolean statusChange = false; // remove


// Control Sensors data transmission
boolean sendAccToMatlab = false;
boolean sendGyroToMatlab = false;
boolean sendEstToMatlab = false;
boolean sendMagnToMatlab = false;
boolean sendMotorsToMatlab = false;
boolean sendRollToMatlab = false;
boolean sendPitchToMatlab = false;
boolean sendYawToMatlab = false;
boolean sendAltToMatlab = false;
boolean sendPidState = false;
boolean sendLandAck = false;
boolean sendTakeOffAck = false;
boolean sendHoverState = false;
boolean sendTenzoState = false;
boolean sendConnAck = false;
boolean sendDisconnAck = false;
boolean storeAccData = false;
boolean accDataReady = false;


// Serial Protocol
int versionArduinoProtocol = 6;
boolean matlab = false;
int modeserial = 0;
long cmd1;
long cmd2;
long cmd3;
long cmd4;
int numCommandToSend = 0;
int numFilterValuesToSend = 0;

int inputBuffSize = 26;
int outputBuffSize = 30;
byte bufferBytes[22];

// Serial IDs
int arduinoAdd = 1;
int MatlabAdd = 2;

int motorsID = 1;
int accID = 2;
int gyroID = 3;
int magnID = 4;
int estID = 5;
int sonicID = 6;
int gpsID = 7;
int baroID = 8;
int rollConsID = 9;
int pitchConsID = 10;
int yawConsID = 11;
int altitudeConsID = 12;
int rollAggID = 13;
int pitchAggID = 14;
int yawAggID = 15;
int altitudeAggID = 16;
int takeOffID = 17;
int iHoverID = 18;
int landID = 19;
int enableMotorsID = 20;
int sendConsPidRollID = 21;
int sendConsPidPitchID = 22;
int sendConsPidYawID = 23;
int sendConsPidAltID = 24;
int sendAggPidRollID = 25;
int sendAggPidPitchID = 26;
int sendAggPidYawID = 27;
int sendAggPidAltID = 28;
int tenzoStateID = 30;
int connID = 31;
int accValuesID = 32;

int cmdLength = 17;
int shortCmdLength = 9;
int headerLength = 8;//13;

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

unsigned char buffer[47];  // or worst case message size 70, 47: 2 mess

MyControlHdr * pCtrlHdr = (MyControlHdr *)(&buffer[0]);

void setup()
{  
  Serial.begin(115200); 
  Serial.println("Hello");
}


void loop()
{  
  while(true)
  {
    serialRoutine();   
    sendDataSensors(matlab);
    //delay(20);
  }
} 

void sendDataSensors(boolean device)
{
  if (!device)
  {
    // Not yet connected. 
    if (sendConnAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      numCommandToSend++;

      // Channel 31: Connection
      pMyCmd->cmd = connID;
      pMyCmd->param1 = 2; 
      sendConnAck = false;
      sendCommandToMatlab(); 
      matlab = true;     
    }
   else if (sendDisconnAck)
   {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      numCommandToSend++;

      // Channel 31: Disconnection
      pMyCmd->cmd = connID;
      pMyCmd->param1 = 10; 
      sendDisconnAck = false;
      matlab = false;
      sendCommandToMatlab();      
    } 
  }
  else if (device)
  {    
    long cmdTmp1;
    long cmdTmp2;
    long cmdTmp3;
    long cmdTmp4;

    if (sendConnAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      numCommandToSend++;

      // Channel 31: Connection
      pMyCmd->cmd = connID;
      pMyCmd->param1 = 2; 
      sendConnAck = false;
      sendCommandToMatlab(); 
    } 
    else if (sendDisconnAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      numCommandToSend++;

      // Channel 31: Disconnection
      pMyCmd->cmd = connID;
      pMyCmd->param1 = 10; 
      sendDisconnAck = false;
      matlab = false;
      sendCommandToMatlab();      
    }
    else if (sendTenzoState)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);

      numCommandToSend++;
      pMyCmd->cmd = tenzoStateID;

        pMyCmd->param1 = 1;
        pMyCmd->param3 = 0; 
        pMyCmd->param2 = 1; 

      sendCommandToMatlab();  
      sendTenzoState = false;
    }
    else if (storeAccData && accDataReady)
    {
      //////////////////////////// Send values to Matlab /// Done in other way
    }
    else if (sendHoverState && sendTakeOffAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);

      pMyCmd->cmd = takeOffID;
   
        pMyCmd->param1 = 1; 

      sendTakeOffAck = false;

      pMyCmd++;           // Second Message

      // Channel 18: Hovering
      pMyCmd->cmd = iHoverID;
        pMyCmd->param1 = 1; 
        
      sendHoverState = false;

      sendCommandToMatlab();  
    }
    else if (sendTakeOffAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);

      pMyCmd->cmd = takeOffID;

        pMyCmd->param1 = 1; 

      sendTakeOffAck = false;
      
      sendCommandToMatlab();  
    }
    else if (sendHoverState && sendLandAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      // Channel 18: Hovering
      pMyCmd->cmd = iHoverID;
        pMyCmd->param1 = 1; 
      sendHoverState = false;

      pMyCmd++;           // Second Message
      // Channel 19: Landing
      pMyCmd->cmd = landID;

        pMyCmd->param1 = 0; 
      sendLandAck = false;

      // Send two commands to Matlab. numCommandsToSend = 2;
      sendCommandToMatlab();  
    }   
    else if (sendHoverState)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      // Channel 18: Hovering
      pMyCmd->cmd = 18;
        pMyCmd->param1 = 1; 
      sendHoverState = false;
      
      sendCommandToMatlab();  
    }
    else if (sendLandAck)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      // Channel 19: Landing
      pMyCmd->cmd = landID;
 
        pMyCmd->param1 = 0; 
      sendLandAck = false;
      sendCommandToMatlab();
    }
    else if (sendAccToMatlab && matlab)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      pMyCmd->cmd = accID;

      pMyCmd->param1 = 1*100;   
      pMyCmd->param2 = 2*100;
      pMyCmd->param3 = 3*100; 

      numCommandToSend++;

      sendCommandToMatlab();  

      sendAccToMatlab = false;
    }
    else if (sendGyroToMatlab  && matlab)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      pMyCmd->cmd = gyroID;

      pMyCmd->param1 = 4*100;   
      pMyCmd->param2 = 5*100;
      pMyCmd->param3 = 6*100; 
      numCommandToSend++;

      sendCommandToMatlab();  
      
      sendGyroToMatlab = false;
    }
    else if (sendEstToMatlab && sendMagnToMatlab && matlab)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);

      numCommandToSend++;
      pMyCmd->cmd = magnID;

      pMyCmd->param1 = 7;   
      pMyCmd->param2 = 8;
      pMyCmd->param3 = 9*100;

      pMyCmd++;           // moves pointer to next command position in message
      numCommandToSend++;
      pMyCmd->cmd = estID;

      pMyCmd->param1 = 10;   
      pMyCmd->param2 = 11;
      pMyCmd->param3 = 12;

      sendCommandToMatlab(); 

      sendEstToMatlab = false;
      sendMagnToMatlab = false;
    }
    else if (sendMotorsToMatlab && matlab)
    {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      pMyCmd->cmd = enableMotorsID;
      numCommandToSend++;

      pMyCmd->param1 = 23; 
      
      sendCommandToMatlab();  
      sendMotorsToMatlab = false;
    }
    else if (sendPidState)
    {
      if (sendAltToMatlab)
      {
        MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
        pMyCmd->cmd = 12;
        numCommandToSend++;

        pMyCmd->param1 = 13*1000;   
        pMyCmd->param2 = 14*1000;
        pMyCmd->param3 = 15*1000;
        pMyCmd->param4 = 16;

        pMyCmd++;           // moves pointer to next command position in message

        pMyCmd->cmd = 16;
        numCommandToSend++;      

        pMyCmd->param1 = 17*1000;   
        pMyCmd->param2 = 18*1000;
        pMyCmd->param3 = 19*1000;

        pMyCmd->param4 = 25;
        sendAltToMatlab = false;
      }
      else if (sendRollToMatlab)
      {
        MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
        pMyCmd->cmd = 9;
        numCommandToSend++;

        pMyCmd->param1 = 20*1000;   
        pMyCmd->param2 = 21*1000;
        pMyCmd->param3 = 22*1000;
        pMyCmd->param4 = 23;
               
        sendRollToMatlab = false;
      }

      sendPidState = false;
      sendCommandToMatlab(); 
    }  
  } 
}

void sendCommandToMatlab()
{
  // Build Header
  pCtrlHdr->srcAddr = 1;
  pCtrlHdr->dstAddr = 2;    // maybe you'll make 2555 a broadcast address? 
  pCtrlHdr->versionX = versionArduinoProtocol;    // possible way to let a receiver know a code version
  pCtrlHdr->numCmds = numCommandToSend;    // how many commands will be in the message
  if (!printBlue)
  {
    Serial.println();
    Serial.print("Sending # commands:");
    Serial.print(numCommandToSend);
    Serial.println();
  }
  pCtrlHdr->hdrLength = sizeof(MyControlHdr );  // tells receiver where commands start
  pCtrlHdr->cmdLength = sizeof(MyCommand );     // tells receiver size of each command 
  // include total length of entire message
  pCtrlHdr->totalLen = sizeof(MyControlHdr ) + (sizeof(MyCommand) * pCtrlHdr->numCmds);
  if (!printBlue)
  {
    Serial.println();
    Serial.print("Total length: ");
    Serial.print(pCtrlHdr->totalLen);
    Serial.println();
  }
  pCtrlHdr->crc = 21;   // dummy temp value

  for (int v=0;v<=sizeof(buffer);v++)
  {
    Serial.write(buffer[v]);  
  }
  numCommandToSend = 0;
}


void serialRoutine()
{
  if (Serial.available()) //inputBuffSize
  {                
      Serial.print("Received: ");
      Serial.println(Serial.available());
      for (int j=0;j<=inputBuffSize;j++)
      {
        bufferBytes[j] = Serial.read();
      }
      //Serial.println("K");
      
      
      for (int j=0;j<=inputBuffSize;j++)
      {
          Serial.println(bufferBytes[j]);
          delay(10);
      }  
         
      boolean temp = false;
      if (bufferBytes[0] == 2 && bufferBytes[1]==1 && temp)
      {        
        Serial.println("K");
        /**
         * Message has been correctly sent by Android and delivered to the Arduino 
         * Decoding Header
         **/

        // Assembling VERSION long skip first two bytes
        // previous version was 2 bytes value
        // now, just one 
        //hiBytew2 = bufferBytes[4];
        ////Serial.println(hiBytew2,BIN);
        //loBytew2 = bufferBytes[5];
        ////Serial.println(loBytew2,BIN);
        //hiWord = word(hiBytew2, loBytew2);
        ////versionProtocol = makeLong( hiWord, loWord);
        //int versionProtocol = hiWord;
        int versionProtocol = bufferBytes[2];
        //Serial.println(versionProtocol);
        //  number cmd
        int numCmd = bufferBytes[3];
        //Serial.println(bufferBytes[3]);
        //Serial.println(versionProtocol);
        int headL = bufferBytes[4];
        //Serial.println(headL);
        int cmdL = bufferBytes[5];
        //Serial.println(cmdL);
        //Serial.println(bufferBytes[5]);
        //  total Length
        //hiBytew2 = bufferBytes[9];
        ////Serial.println(hiBytew2,BIN);
        //loBytew2 = bufferBytes[10];
        //Serial.println(loBytew2,BIN);
        //short totL = word(hiBytew2, loBytew2);
        int totL = bufferBytes[6];
        //Serial.println(totL);
        // CRC
        //hiBytew2 = bufferBytes[11];
        //// Serial.println(hiBytew2,BIN);
        //loBytew2 = bufferBytes[12];
        //// Serial.println(loBytew2,BIN);
        //short crc = word(hiBytew2, loBytew2);
        int crc = bufferBytes[7];
        // Serial.println(crc);
        /**
         * Decoding Command
         **/
        int type = bufferBytes[8];
        Serial.print("type  :");
        Serial.println(type);
        if (printVerboseSerial && !printBlue)
        {
          Serial.println();
          Serial.print("Version");
          Serial.print(versionProtocol);
          if (versionProtocol != versionArduinoProtocol && !printBlue)
            Serial.println("Warning Sync Repos, different serial protocols");
          Serial.println();
          Serial.print("Tot Len");
          Serial.println(totL);
          Serial.println();
          Serial.print("Crc");
          Serial.println(crc);
          Serial.print("TYPE");
          Serial.println(type);
        }
        if (type == connID)
        {
          //Serial.println("Connection");
          // Connection channel
          int alto = bufferBytes[9];
          Serial.println(alto);
          int basso = bufferBytes[10];
            Serial.println(basso);
          hiBytew2 = bufferBytes[11];
            Serial.println(hiBytew2);
          loBytew2 = bufferBytes[12];
            Serial.println(loBytew2);
          float conAck = *((float*)(bufferBytes + 9));
          
          //float  conAck1 = (speedLong << 8) | speedArray[i]
            Serial.print("RCon 1:  "); 
            Serial.println(conAck); 
          //conAck = word(hiBytew2, loBytew2);
            //Serial.println(conAck); 

          if (conAck == 1)
          {
            Serial.println("ConAck = 1");
            if (!matlab)
            {
              sendConnAck = true;
              sendTenzoState = true;
            }
            else 
            {
              sendConnAck = true;
              sendTenzoState = true;
              // Connection requested but already connected
              if (printVerboseSerial && !printBlue)
              {
                Serial.println();
                Serial.println(" Warning!! Something wrong during CONnection sync.");
                Serial.print("Connection requested but already connected");
                Serial.println();
              }
            }
          }
          else if (conAck == 3)
          {
            Serial.println("ConAck 3");
            // Communication problems
            if (!matlab)
            {
              sendConnAck = true;
              sendTenzoState = true;
              if (printVerboseSerial && !printBlue)
              {
                Serial.println();
                Serial.println(" Warning!! Communication problems. Sending again.");
                Serial.println();
              }
            }
            else 
            {
              sendConnAck = true;
              sendTenzoState = true;
              // Impossible: Connection requested but already connected
              if (printVerboseSerial && !printBlue)
              {
                Serial.println();
                Serial.println(" Warning!! Something wrong during CONnection sync.");
                Serial.print("Connection requested but already connected");
                Serial.println();
              }
            }
          }
          else if (conAck == 0)
          {
            Serial.println("ConAck 0");
            if (matlab)
            {
              sendDisconnAck = true;
            }
            else 
            {
              sendDisconnAck = true;
              Serial.println("Sending DisconnAck");
              // Impossible: Disconnection requested but already disconnected
              if (printVerboseSerial)
              {
                Serial.println();
                Serial.println(" Warning!! Something wrong during DISconnection sync.");
                Serial.print("Disconnection requested but already disconnected");
                Serial.println();
              }
            }
          }
          else
          {
            //  Shouldn't get here - Wrong mess received
            if (printVerboseSerial)
            {
              Serial.println();
              Serial.println(" Warning!! Third Condition.");
              Serial.println();
            }
          }
        }
        else if (type == tenzoStateID)
        {
          hiBytew2 = bufferBytes[16];
          loBytew2 = bufferBytes[17];
          int remoteTakeOffTemp = word(hiBytew2, loBytew2);

          hiBytew2 = bufferBytes[20];
          loBytew2 = bufferBytes[21];
          int remoteHoverTemp = word(hiBytew2, loBytew2);

          hiBytew2 = bufferBytes[24];
          loBytew2 = bufferBytes[25];
          int remoteLandTemp = word(hiBytew2, loBytew2);

        }
        else if (type == motorsID)
        {
          // Motors : same cmd   

          stopSendingToMatlab();

          hiBytew2 = bufferBytes[16];
          loBytew2 = bufferBytes[17];
          int motorSpeedChange = word(hiBytew2, loBytew2);
          motorSpeedChange = motorSpeedChange - 100;

          hiBytew2 = bufferBytes[20];
          loBytew2 = bufferBytes[21];
          int m2 = word(hiBytew2, loBytew2);
          m2 = m2 - 100;

          hiBytew2 = bufferBytes[24];
          loBytew2 = bufferBytes[25];
          int m3 = word(hiBytew2, loBytew2);
          m3 = m3 -100;

          hiBytew2 = bufferBytes[28];
          loBytew2 = bufferBytes[29];
          int m4 = word(hiBytew2, loBytew2);
          m4 = m4 -100;

        if (type == accID)
        {
          // Send Acc Values
          stopSendingToMatlab();
          sendAccToMatlab = true;
        }
        else if (type == gyroID)
        {
          // Send Gyro Values
          stopSendingToMatlab();
          sendGyroToMatlab = true;
        }
        else if (type == magnID || type == estID)
        {
          // Send Est + Magn Values
          stopSendingToMatlab();
          sendMagnToMatlab = true;
          sendEstToMatlab = true;
        }
        else if (type == takeOffID)
        {
          // Channel 17: Take Off
          stopSendingToMatlab();

          hiBytew2 = bufferBytes[16];
          loBytew2 = bufferBytes[17];
          int altitudeRef = word(hiBytew2, loBytew2);
          
        }
        else if (type == landID)
        {
          // Channel 19: Land
          stopSendingToMatlab();

          hiBytew2 = bufferBytes[16];
          loBytew2 = bufferBytes[17];
          landSpeed = word(hiBytew2, loBytew2);
          
        }
        else if (type == iHoverID)
        {
          // Channel 18: Hovering
          if (initialized)
          {
            // Hover with PID
            stopSendingToMatlab();

            hiBytew2 = bufferBytes[16];
            loBytew2 = bufferBytes[17];
            int enab = word(hiBytew2, loBytew2);             
            if (printVerboseSerial)
            {
              Serial.println();
              Serial.print("Channel: 18   enab Pid: ");
              Serial.print(enab);
              Serial.println();
            }
          }
          else
          {
            Serial.println();
            Serial.print("Received Hovering command but Tenzo is not flying!! WARNING!!");
            Serial.println();             
          }
        }
        else if (type == enableMotorsID) // Deprecated
        {
          // Request Motors Data  type: 20  
          stopSendingToMatlab();

          hiBytew2 = bufferBytes[16];
          loBytew2 = bufferBytes[17];
          int enab = word(hiBytew2, loBytew2);
          if (enab==0)
            sendMotorsToMatlab = false;
          else if (enab == 1)
            sendMotorsToMatlab = true;
        }
        else if (type == sendConsPidRollID || type == sendAggPidRollID)
        {
          // Request Motors Data   type: 21      || 25  
          stopSendingToMatlab();
          sendPidState = true;
          sendRollToMatlab = true;          
          if (printVerboseSerial)
          {
            Serial.println();
            Serial.print("Channel: 21||25   roll Pid? ");
            Serial.println();
          }
        }
        else if (type == sendConsPidPitchID || type == sendAggPidPitchID)
        {
          // Request Motors Data   type: 22      || 26 
          stopSendingToMatlab();
          sendPidState = true;
          sendPitchToMatlab = true;         
          if (printVerboseSerial)
          {
            Serial.println();
            Serial.print("Channel: 22||26   pitch Pid? ");
            Serial.println();
          }
        }
        else if (type == sendConsPidYawID || type == sendAggPidYawID)
        {
          // Request Motors Data   type: 23      || 27
          stopSendingToMatlab();
          sendPidState = true;
          sendYawToMatlab = true;         
          if (printVerboseSerial)
          {
            Serial.println();
            Serial.print("Channel: 23||27   yaw Pid? ");
            Serial.println();
          }
        }
        else if (type == sendConsPidAltID || type == sendAggPidAltID)
        {
          // Request Motors Data   type: 24      || 28
          stopSendingToMatlab();
          sendPidState = true;
          sendAltToMatlab = true;         
          if (printVerboseSerial)
          {
            Serial.println();
            Serial.print("Channel: 24||28   alt Pid? ");
            Serial.println();
          }
        }
        else 
        {
          stopSendingToMatlab();

          hiBytew2 = bufferBytes[16];
          loBytew2 = bufferBytes[17];
          cmd1 = word(hiBytew2, loBytew2);

          hiBytew2 = bufferBytes[20];
          loBytew2 = bufferBytes[21];
          cmd2 = word(hiBytew2, loBytew2);

          hiBytew2 = bufferBytes[24];
          loBytew2 = bufferBytes[25];
          cmd3 = word(hiBytew2, loBytew2);

          hiBytew2 = bufferBytes[28];
          loBytew2 = bufferBytes[29];
          cmd4 = word(hiBytew2, loBytew2);

          if (type == 16)
          {
            // Pid AGG ALTITUDE 
            int a  = (double) cmd1/1000;
            int b = (double) cmd2/1000;
            int c = (double) cmd3/1000;
            int d = cmd4;
            sendPidState = true;
            sendAltToMatlab = true;
         }          
      }
    } // Message to Arduino from Matlab
  }
  else if (Serial.available())
  {
    char modeS = Serial.read(); 
    
    if (modeS == 'a')
    {
      initialize();
      Serial.println("initialize");
    }
    if (modeS == 'b')
    {
      Serial.println('K');   
    }    
    if (modeS == 'K')
    {
      connAck = 1;
      printBlue = true;
    }
    if (modeS == 'L')
    {
      Serial.println(" Land ");
    }
    if (modeS == 't')
    {
      char t = Serial.read();
      if (t=='e')
        sendBlueAngle = true;
      else if (t=='d')
      {
        Serial.println('A');    
        sendBlueAngle = false;
      }   
    }
    else if (modeS == 'L')
    {
      if (!processing && printBlue)
        Serial.println("Landing");
      land();
    }
  }
  
  timerSec = micros()-secRoutine;
  lastTimeToRead = micros();

  if (timerSec >= 1000000)
  {
    secRoutine = micros();
    if (!processing && printTimers && !printBlue)
    {
      //Serial.print(cont);
      Serial.print("[sample/sec] ");
      Serial.print(contSamples);
      Serial.print("    Ctrl: ");
      Serial.println(countCtrlAction);
      Serial.print("    tservo: ");
      Serial.println(servoTime);
      Serial.println();
    }
    cont=0;      
    contSamples=0;      
    contCalc=0; 
    countCtrlAction=0;
  }

  timerRoutine = micros()-kMRoutine;

  // The following loop runs every 5ms
  if (timerRoutine >= deltaT*1000) 
  {      
    kMRoutine = micros();
    count += 1;
    if (count >= 1)
    {
      count = 0;
      
      //control();  
      controlW();
      
      //servoTime = micros();
      motorSpeedPID(throttle, OutputWPitch, OutputWRoll, OutputWYaw, OutputAltitude);
      //servoTime = micros() - servoTime;
      if (processing && printSerial)
      {
        printSerialAngle();
      }
      if (printBlue && sendBlueAngle)
        printSerialAngleBlue();
      if (printAccs)
        printAcc();
      if (printOmegas)
         printOmega();
      //if (printTimers)
        // printT();
      if (printPIDVals)
        printPidValues();
      if (printMotorsVals)
        printMotorsValues();
    }
  }
}


void stopSendingToMatlab()
{  
  sendAccToMatlab = false;
  sendGyroToMatlab = false;
  sendMagnToMatlab = false;
  sendEstToMatlab = false;
  sendRollToMatlab = false;
  sendMotorsToMatlab = false;
  sendPitchToMatlab = false;
  sendYawToMatlab = false;
  sendAltToMatlab = false;
} 
