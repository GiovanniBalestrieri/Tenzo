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
 **           pin A4   :    MPU_SDA
 **           pin A5   :    MPU_SCL
 **
 */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

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
boolean verboseSerialRead = true;
boolean verboseSerialAnalyze = false;
boolean rosSerial = true;
boolean printEnc = true; 
boolean printMotors = true;
boolean printQuat = true;

boolean data= false;

// Serial variables
const double baudrate = 115200;
String inString = "";
String inComingString = "";
char prefix;

// IMU variables

MPU6050 mpu;
#define OUTPUT_QUAT
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity v

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

// Initialize Timer
void setupTimersAndEncoders()
{
  pinMode(4, OUTPUT); 
  pinMode(7, OUTPUT); 
  pinMode(10, OUTPUT); 
  pinMode(9, OUTPUT);  
 
  // Disable global interrputs
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
  Serial.begin(baudrate);      
}
 
void setup(void) 
{ 
  setupTimersAndEncoders();
  setupSerial();
  setupIMU();
  
} 
 
void loop(void) 
{  
  motorsRoutine();  
  mpuRoutine();
  updateEncoders();
  SerialRoutine();
  handleOverflow();  
}

void mpuRoutine()
{
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    if (!verbose)
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      
      printQuaternions();  
}

void setupIMU()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #endif
    
    if (verbose)
      Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    
    if (!verbose){
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    }
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {
        // turn on the DMP, now that it's ready
    if (!verbose)
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
    if (!verbose)
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
    if (!verbose)
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else 
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
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
 while(Serial.available()>0)
 {
   int readChar = Serial.read();
   if (verboseSerialRead)
   {
     Serial.print("received: ");
     Serial.println((char)readChar);
     Serial.println(readChar);
   }
   if (readChar != 10)
   {
     if (verboseSerialRead)
     {
       Serial.println("A");
     }
     inComingString += (char) readChar;
   }
   else 
   {
     if (verboseSerialRead)
       Serial.println("             B");
     data = true;
   }
 }
 if (data)
 {
   // Analyze string
   if (verboseSerialAnalyze)
   {
     Serial.println(inComingString);
   }
   analyzeIncomingString(inComingString);
   data = false;
   inComingString="";
 }
}
     
void analyzeIncomingString(String cmd)
{
  // Panic cmd '^'
    if(cmd[0] != 94)
    {
      if (cmd[0] == 's')
      {       
        boolean m1 = false,m2 = false;
        
        if (verboseSerialAnalyze)
        {
         Serial.println("Motor CMD");
        }
        char val2 = cmd[1];
        
        if (verboseSerialAnalyze)
        {
         Serial.print("read: ");
         Serial.println((int) val2);
         Serial.println(val2);
        }
        if ((int) val2 == 44)
        {  
          boolean neg1 = false, neg2 = false;
          int readC = 0;
          int i = 1;
          while( readC != 44 && (1+i) <= cmd.length())
          {
            if (verboseSerialAnalyze)
            {
              Serial.print("Dentro 1:  ");
              Serial.print(cmd[1+i]);
              Serial.print("  i: ");
              Serial.println(i);
            }
            readC = cmd[1+i];
            if (readC == 45)
            {
              neg1 = true;
            }
            if (isDigit(readC))
            {
              inString += (char) readC;
            }
            i++;
          }
          double speed1Candidate;
          if (neg1)
            speed1Candidate = - (inString.toInt());
          else            
            speed1Candidate = (inString.toInt());
          if (verboseSerialAnalyze)
          {
            Serial.print("cmd 1: ");
            Serial.println(speed1Candidate);
          }
          if (readC == 44)
            m1 = true;
          // readC is now a comma, asci 44
           inString = "";
          readC = 0;
          int j  = 0;
          while( readC != 44 && (1+i+j) <= cmd.length())
          {
            if (verboseSerialAnalyze)
            {
              Serial.print("Dentro 2:  ");
              Serial.print(cmd[1+i+j]);
              Serial.print("   j:  ");
              Serial.println(j);
            }
            readC = cmd[1+i+j];
            if (readC == 45)
              neg2 = true;
            if (isDigit(readC))
            {
              inString += (char) readC;
            }
            if (readC == 10)
              break;
            j++;
          }
          if (readC == 44)
            m2 = true;
          double speed2Candidate;
          if (neg2)
            speed2Candidate = -(inString.toInt());
          else
            speed2Candidate = inString.toInt();
          if (verboseSerialAnalyze)
          {
            Serial.print("cmd  2: ");
            Serial.println(speed2Candidate);
          }
          //Serial.println(readC); 
          inString = "";
          i=0;
          j=0;    
          neg1 = false;
          neg2 = false;
          
          if (m1 && m2)
          {
            if (speed1Candidate>255)
              speed1=255;
            else if (speed1Candidate<-255)
              speed1=-255;
            else
              speed1=speed1Candidate;
            
            if (speed2Candidate>255)
              speed2=255;
            else if (speed2Candidate<-255)
              speed2=-255;
            else 
              speed2=speed2Candidate;
          }
        }
        else
        {
          Serial.println("Warning motor command corrupted");
        }       
      } 
      else if (cmd[0] == 'm')
      {
        printMotors = !printMotors;
      } 
      else if (cmd[0] == 'e')
      {
        printEnc = !printEnc;
      } 
      else if (cmd[0] == 'a')
      {
        verboseSerialAnalyze = !verboseSerialAnalyze;
      } 
      else if (cmd[0] == 'q')
      {
        printQuat = !printQuat;
      } 
    }
    else 
    {
      // If cmd received = '^' then stop
      stopMotors();  
      Serial.println("STOP");
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

void motorsRoutine()
{ 
  if (speed1>=0)
    avance(1,speed1);
  else
    arriere(1,-speed1);
  if (speed2 >=0)
    avance(2,speed2);
  else
    arriere(2,-speed2); 
    
  if (rosSerial && printMotors)
  {
    Serial.print("s");
    Serial.print("\t");
    Serial.print(speed1);
    Serial.print("\t");
    Serial.print(speed2);  
    Serial.print("\n");
  }
}

void arriere(int motor, char a)          //Move forward
{
  if (motor == 1)
  {
    analogWrite (E1,a);      
    digitalWrite(M1,LOW);
  }
  else if (motor == 2)
  {
    analogWrite (E2,a);    
    digitalWrite(M2,LOW);    
  }
}  

void avance(int motor, char b)          //Move forward
{
  if (motor == 1)
  {
    analogWrite (E1,b);    
    digitalWrite(M1,HIGH);
  }
  else if (motor == 2)
  {
    analogWrite (E2,b);    
    digitalWrite(M2,HIGH);
  }
}  
 
void stopMotors(void)                 
{
  digitalWrite(E1,0); 
  digitalWrite(M1,LOW);    
  digitalWrite(E2,0);   
  digitalWrite(M2,LOW);    
}  

void printQuaternions()
{
  // display quaternion values in easy matrix form: w x y z
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  if (printQuat)
  {
    Serial.print("q");
    Serial.print(",");
    Serial.print(q.x); Serial.print(",");
    Serial.print(q.y); Serial.print(",");
    Serial.print(q.z); Serial.print(",");
    Serial.println(q.w);
  }
}
