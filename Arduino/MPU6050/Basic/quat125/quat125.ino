#include "I2Cdev.h"
#include "Pid.h"

boolean printBlue = false;
boolean processing = true;
boolean printMotorsVals = false;
boolean printPIDVals = false;
boolean printSerial = false;
boolean printTimers = true;
boolean printAnglesEst = false;

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_EULER

//#define OUTPUT_READABLE_YAWPITCHROLL

//#define OUTPUT_READABLE_REALACCEL

//#define OUTPUT_READABLE_WORLDACCEL

//#define OUTPUT_TEAPOT
// MPU control/status vars
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
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

#define Kp_1 2.0//5.2f
#define Ki_1 0.0//0.3f
#define Kd_1 0.0//0.0f

Pid pidSx = Pid(-Kp_1, Ki_1, Kd_1);
Pid pidDx = Pid(Kp_1, Ki_1, Kd_1);

// Timers Cont
unsigned long timerISR =0, timerPid=0, timerEuler=0, timerMotors=0;
long contIsr = 0, contMotors = 0, contEuler = 0, contPid = 0;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() 
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #endif
    Serial.begin(9600);
    // initialize device
    if (!processing)
      Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    if (!processing){
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
    if (!processing)
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
    if (!processing)
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
    if (!processing)
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
    
    // ISR
    
    cli(); // Stops interrupts
  
    TCCR2A = 0;// set entire TCCR1A register to 0
    TCCR2B = 0;// same for TCCR1B
    OCR2A=700; //16*10^6/(20Hz*1024) - 1 = 780 -> 20 Hz 
    TCCR2A |= (1 << WGM21); // turn on CTC mode
    TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22); // Set CS10 and CS12 bits for 1024 prescaler
    TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt
        
    sei(); //enable global interrupts
    
}

ISR(TIMER2_COMPA_vect)  // Interrupt service routine @ 200 Hz
{
  cli();
  timerISR = micros();
  
  contIsr++;
  
  timerISR = micros() - timerISR; 
  sei();
}

long TCAMP = 50000;
int count = 0;
int donTouch = 1;
long tOld = 0;

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() 
{
  mpuRoutine();
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  if(micros()-tOld >= TCAMP)
  {
    tOld = micros();
    count += 1;
    if(count >= donTouch) // Runs @ 2 Hz
    {
      count = 0;
      getEuler();
      serialRoutine();
      contIsr=0; // resets ISR counter      
    }
  }      
}

void serialRoutine()
{
      printSerialAngle();
      #ifdef OUTPUT_READABLE_EULER
          printEulerSerial();
      #endif
  
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          printYPRSerial();
      #endif 
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
    if (!processing)
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
  
}

void printYPRSerial()
{  
  if (!processing)
  {
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
  }
}

void printEulerSerial()
{
  if (!processing)
  {
    // display Euler angles in degrees
    //mpu.dmpGetQuaternion(&q, fifoBuffer);
    //mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180/M_PI);
  }
}

void getEuler()
{
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
}

void printSerialAngle()
{
  if (!printBlue)
  { 
    //mpu.dmpGetQuaternion(&q, fifoBuffer);
    //mpu.dmpGetEuler(euler, &q);
    Serial.print(",");
    Serial.print(euler[0] * 180/M_PI);
    Serial.print(",");
    Serial.print(euler[1] * 180/M_PI);
    Serial.print(",");
    Serial.print(euler[2] * 180/M_PI);
    Serial.println();
  }
}

