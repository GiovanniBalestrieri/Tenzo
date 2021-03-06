/** 
 **                     |_.-*| THOR 2015 |*-._|
 **
 **  [Test] We are using Timer2 in order to change PWM frequency > 30 KHz --> Noise Free.
 **
 **  Timer 1 : 
 **           pin 9  (OC2B) -> M2
 **           pin 10 (OC2A) -> M1
 **
 **  Wiring:
 **           pin 4    :    M1 Dir
 **           pin 7    :    M2 Dir
 **           pin 2    :    Enc_M1_A
 **           pin 3    :    Enc_M2_A 
 **           pin 5    :    Enc_M1_B
 **           pin 6    :    Enc_M2_B 
 **
 */

int E1 = 9;     //M1 Speed Control
int E2 = 10;     //M2 Speed Control
int M1 = 4;     //M1 Direction Control
int M2 = 7;     //M1 Direction Control
int counter=0;

int speed1 = 0,speed2 = 0;
 
#define pinEncoderBS 5
#define pinEncoderBD 6

volatile int MSverso = 0;
volatile int MDverso = 0;

volatile long int SX_Enc_Ticks = 0;
volatile boolean SX_Enc_BSet;

volatile long int DX_Enc_Ticks = 0;
volatile boolean DX_Enc_BSet;



boolean verbose = true;
boolean rosSerial = true;
boolean printEnc = true; 



// Timer settings
// Initialize Timer
void setupTimersAndEncoders()
{
  pinMode(4, OUTPUT); 
  pinMode(7, OUTPUT); 
  pinMode(10, OUTPUT); 
  pinMode(9, OUTPUT);  
  
  
  cli();
  
  //TCCR1A = 0;
  TCCR1B = 0;
  
  // No prescaler:
  TCCR1B |=  (1 << CS10);
  // Set CS20 and CS22 bits for 8 prescaler:
  //TCCR1B |=  (1 << CS11);
  // Set CS20 and CS22 bits for 64 prescaler:
  //TCCR1B |= (1 << CS10) | (1 << CS11);
  // Set CS20 and CS22 bits for 256 prescaler:
  //TCCR1B |= (1 << CS12);  
  // Set CS20 and CS22 bits for 1024 prescaler:
  //TCCR1B |= (1 << CS12) | (1 << CS10);  
  
  
  //OCR1A=maxPWM;
  
  // Define encoder pin Interrupt
  attachInterrupt(0, MSencVel, RISING);
  attachInterrupt(1, MDencVel, RISING);
  // enable global interrupts:
  sei(); 
  
  // interrupt pins from encoders
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  
  // Stop motors
  digitalWrite(E1,LOW);   
  digitalWrite(E2,LOW); 
  stopMotors();
 
  if (verbose)
  {
     Serial.println("[ OK ] Init PWMs"); 
  }
}

void setupSerial()
{  
  //Set Baud Rate
  Serial.begin(115200);      
}
 
void setup(void) 
{ 
  setupTimersAndEncoders();
  setupSerial();
  
} 
 
void loop(void) 
{  
  motorsRoutine();
  updateEncoders();
  SerialRoutine();
  handleOverflow();  
}

void motorsRoutine()
{ 
  if (speed1>=0)
    avanceM1(speed1);
  else
    arriereM1(-speed1);
  if (speed2 >=0)
    avanceM2(speed2);
  else
    arriereM2(-speed2); 
}

void updateEncoders()
{
  if (printEnc)
  {
    Serial.print("e");
    Serial.print("\t");
    Serial.print(SX_Enc_Ticks);
    Serial.print("\t");
    Serial.println(DX_Enc_Ticks);
  }
}

void SerialRoutine()
{
 if(Serial.available())
 {
    char val = Serial.read();
    if(val != -1)
    {
      switch(val)
      {
        case 'w'://Move Forward
          speed1 = 255;
          speed2 = 255;
          advance (255,255);   //move forward in max speed
          break;        
          
        case 's'://Move Backward
          back_off (255,255);   //move back in max speed
          break;
        case 'a'://Turn Left
          turn_L (100,100);        
          break;      
         
        case 'b'://Turn Left
          speed1 = 80;
          speed2 = 80;
          advance(80,80);        
          break;       
        case 'd'://Turn Right
          turn_R (100,100);
          break;
        case 'z':
          Serial.println("Hello");
          break;
        case 'x':
          stopMotors();
          break;
          
          
        case 't':
          speed1 = speed1 + 10;
          Serial.print("M1:    ");
          Serial.println(speed1);
          break;
        case 'g':
          speed1 = speed1 - 10;
          Serial.print("M1:    ");
          Serial.println(speed1);
          break;
         
        case 'y':
          speed1 = speed1 + 2;
          Serial.print("M1:    ");
          Serial.println(speed1);
          break;
        case 'h':
          speed1 = speed1 - 2;
          Serial.print("M1:    ");
          Serial.println(speed1);
          break;
          
        case 'p':
          speed2 = speed2 + 10;
          Serial.print("M2:    ");
          Serial.println(speed2);
          break;
        case 'l':
          speed2 = speed2 - 10;
          Serial.print("M2:    ");
          Serial.println(speed2);
          break;
          
        case 'o':
          speed2 = speed2 + 2;
          Serial.print("M2:    ");
          Serial.println(speed2);
          break;
        case 'k':
          speed2 = speed2 - 2;
          Serial.print("M2:    ");
          Serial.println(speed2);
          break;
        case 'n':
          printEnc = !printEnc;
          break;
      }
    }
    else 
      stopMotors();  
  } 
}

void MSencVel()
{
  SX_Enc_BSet = digitalRead(pinEncoderBS);
  SX_Enc_Ticks -= SX_Enc_BSet ? -1:+1;   
}

void MDencVel()
{
  DX_Enc_BSet = digitalRead(pinEncoderBD);
  DX_Enc_Ticks += DX_Enc_BSet ? -1:+1;
}


void handleOverflow()
{
  //handles overflow OK
  if(abs(DX_Enc_Ticks)>60000)
  {
    DX_Enc_Ticks=0;
    //M1oldPos = -M1deltaPos;
    //Serial.println("overflow");
  } 
  if(abs(DX_Enc_Ticks)>60000)
  {
    DX_Enc_Ticks=0;
    //M2oldPos = -M2deltaPos;
    //Serial.println("overflow ");
  }
}

 
void stopMotors(void)                    //Stop
{
  digitalWrite(E1,0); 
  digitalWrite(M1,LOW);    
  digitalWrite(E2,0);   
  digitalWrite(M2,LOW);    
}   
void advance(char a,char b)          //Move forward
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,HIGH);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
}  

void avanceM1(char a)          //Move forward
{
  analogWrite (E1,a);      
  digitalWrite(M1,HIGH);
} 

void arriereM1(char a)          //Move forward
{
  analogWrite (E1,a);      
  digitalWrite(M1,LOW);
}  

void avanceM2(char b)          //Move forward
{
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
}  

void arriereM2(char b)          //Move forward
{
  analogWrite (E2,b);    
  digitalWrite(M2,LOW);
}  

void back_off (char a,char b)          //Move backward
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);   
  analogWrite (E2,b);    
  digitalWrite(M2,LOW);
}
void turn_L (char a,char b)             //Turn Left
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);    
  analogWrite (E2,b);    
  digitalWrite(M2,HIGH);
}
void turn_R (char a,char b)             //Turn Right
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);    
  analogWrite (E2,b);    
  digitalWrite(M2,LOW);
}
