int headerLength = 8;//13;
boolean sendToClient = false;
boolean verbosity = true;
int versionArduinoProtocol = 1;
long cmd1;
long cmd2;
long cmd3;
long cmd4;
int numCommandToSend = 0;





void assembleAndSend(float a, float b, float c) {
      MyCommand * pMyCmd = (MyCommand *)(&buffer[sizeof(MyControlHdr)]);
      pMyCmd->cmd = '1';

      pMyCmd->param1 = a*100;   
      pMyCmd->param2 = b*100;
      pMyCmd->param3 = c*100; 

      numCommandToSend++;

      sendCommand();  

      if (verbosity)
      {
        Serial.println();
        Serial.print("Sending Payload\n");
        Serial.println(pMyCmd->cmd);
        Serial.println(pMyCmd->param1);
        Serial.println(pMyCmd->param2);
        Serial.println(pMyCmd->param3);
        Serial.println(); 
      } 
  sendToClient = false;
}

void sendCommand()
{
  // Build Header
  pCtrlHdr->srcAddr = 1;
  pCtrlHdr->dstAddr = 2;    // maybe you'll make 2555 a broadcast address? 
  pCtrlHdr->versionX = 1;    // versionArduinoProtocol possible way to let a receiver know a code version
  pCtrlHdr->numCmds = numCommandToSend;    // how many commands will be in the message
  if (verbosity)
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
  if (verbosity)
  {
    Serial.println();
    Serial.print("Total length: ");
    Serial.print(pCtrlHdr->totalLen);
    Serial.println();
  }
  pCtrlHdr->crc = 21;   // dummy temp value

//  String payload = (String) buffer;
  //buffer.getBytes(data, payload.length());

  /*
  for (int v=0;v<=sizeof(buffer);v++)
  {
    Serial.write(buffer[v]);  
  }
  */
  numCommandToSend = 0;
}







