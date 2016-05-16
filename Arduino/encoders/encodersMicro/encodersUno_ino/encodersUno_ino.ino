// Define encoder pin Interrupt
#define pinEncoderBS 5

volatile int MSverso = 0;
volatile long int SX_Enc_Ticks = 0;
volatile boolean SX_Enc_BSet;

const byte interruptPin = 2;

void setup()
{
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(interruptPin, MSencVel, CHANGE);
  Serial.begin(115200);
}



void loop(){
  //if (SX_Enc_BSet)
    Serial.println(SX_Enc_Ticks);
}

void MSencVel()
{
  SX_Enc_BSet = digitalRead(pinEncoderBS);
  SX_Enc_Ticks -= SX_Enc_BSet ? -1:+1;  
}


