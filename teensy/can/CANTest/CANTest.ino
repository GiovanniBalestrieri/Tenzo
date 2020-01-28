#include <kinetis_flexcan.h>
#include <FlexCAN.h>

int led = 13;
FlexCAN CANbus(125000); 
static CAN_message_t msg,rxmsg;
static uint8_t hex[17] = "0123456789abcdef";

int txCount,rxCount;
unsigned int txTimer,rxTimer;



// -------------------------------------------------------------
void setup(void)
{
  CANbus.begin();
  pinMode(led, OUTPUT);
  digitalWrite(led, 1);

  delay(1000);
  Serial.println(F("Teensy 3.2 CAN"));
}


// -------------------------------------------------------------
void loop(void){
  if (CANbus.available()){
      Serial.write("Receiving");
      
      while ( CANbus.read(rxmsg) ) {
        
      digitalWrite(led, 1);
      for (int i = 0; i < rxmsg.len; i++){
        //hexDump( sizeof(rxmsg), (uint8_t *)&rxmsg );
        Serial.write(rxmsg.buf[i]);
      }
    }
  }
  
    digitalWrite(led, 0);

}
