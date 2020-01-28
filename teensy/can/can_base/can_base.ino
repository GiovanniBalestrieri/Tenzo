// Teensy 3.6 test

// This demo runs only on Teensy 3.6
// The CAN0 and the CAN1 integrated modules exchange datas.

// No external transceiver is required, connect together (use "short lines" i.e < 10 cm):
//  - CAN0TX: pin #3
//  - CAN0RX: pin #4
//  - CAN1TX: pin #33
//  - CAN1RX: pin #34
// CAN0TX and CAN1TX are configured as open-collector outputs
// CAN0RX and CAN1RX are configured with input pullups enabled

//——————————————————————————————————————————————————————————————————————————————

#include <ACAN.h>

//——————————————————————————————————————————————————————————————————————————————

static const uint32_t BIT_RATE = 1000 * 1000 ;

//——————————————————————————————————————————————————————————————————————————————

void setup () {
  //--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
  //--- Start serial
  Serial.begin (38400) ;
  //--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  //--- Configure CAN0
  ACANSettings settings (BIT_RATE) ;
  settings.mTxPinIsOpenCollector = true ;
  settings.mRxPinHasInternalPullUp = true ;
  uint32_t errorCode = ACAN::can0.begin (settings) ;
  
  if (0 == errorCode) {
    Serial.println ("can0 ok") ;
  }else{
    Serial.print ("Error can0: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————

static unsigned gBlinkLedDate = 0 ;
static unsigned gReceivedFrameCount0 = 0 ;
static unsigned gReceivedFrameCount1 = 0 ;
static unsigned gSentFrameCount0 = 0 ;
static unsigned gSentFrameCount1 = 0 ;

static const unsigned MESSAGE_COUNT = 10 * 1000 ;

void loop () {
  CANMessage frame ;
//--- Send messages via CAN0
  if (gSentFrameCount0 < MESSAGE_COUNT) {
  //--- Make an even identifier for can0
    frame.id = millis () & 0x7FE ;
    frame.len = 8 ;
    for (uint8_t i=0 ; i<8 ; i++) {
      frame.data [i] = i ;
    }
    const bool ok = ACAN::can0.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount0 += 1 ;
    }
  }
//--- Receive frame from CAN0
  if (ACAN::can0.available ()) {
    ACAN::can0.receive (frame) ;
    gReceivedFrameCount0 ++ ;
  }
}

//——————————————————————————————————————————————————————————————————————————————
