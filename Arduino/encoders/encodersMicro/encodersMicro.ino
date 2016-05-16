// Define encoder pin Interrupt
#define pinEncoderBS 5

volatile int MSverso = 0;
volatile long int SX_Enc_Ticks = 0;
volatile boolean SX_Enc_BSet2;
volatile boolean SX_Enc_BSet3;
volatile int lastEncoded = 0;

const byte interruptPin2 = 2; // digital pin 0
const byte interruptPin3 = 3; // digital pin 1

void setup()
{
  pinMode(interruptPin2, INPUT_PULLUP);
  pinMode(interruptPin3, INPUT_PULLUP);
  
  //attachInterrupt(interruptPin2, MSencVel, RISING);
  //attachInterrupt(interruptPin3, MSencVelFall, FALLING);
  
  attachInterrupt(interruptPin2, updateEncoder, CHANGE);   
  attachInterrupt(interruptPin3, updateEncoder, CHANGE); 
  Serial.begin(115200);
}



void loop(){
  //if (SX_Enc_BSet)
    Serial.println(SX_Enc_Ticks);
}

void updateEncoder(){
  int MSB = digitalRead(interruptPin2); //MSB = most significant bit
  int LSB = digitalRead(interruptPin3); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
    SX_Enc_Ticks ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) 
    SX_Enc_Ticks --;

  lastEncoded = encoded; //store this value for next time
}

