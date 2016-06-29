#include <Servo.h>
#include <Wire.h>
#include "tenzo.h"

// Se fai operazioni sui float o double su una variabile utilizzata come contatore
// aggiornato in una ISR -> usa una variabile d'appoggio tipo cont_safe usa:
// sei()
// cont_safe = cont
// cli()

// Init User Experience agent Sakura
Ux sakura;

// Init Propulsion handler
Propulsion tenzoProp(sakura.getM(1),sakura.getM(2),sakura.getM(3),sakura.getM(4));

// Init FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

// Init Sonar
Sonar ux1 = Sonar();

// Init RTC
RTC_DS1307 RTC;
DateTime now;

// Init Log
Logs logger = Logs();
        
// Init scheduler with MAX_TASKS
Scheduler scheduler = Scheduler(MAX_TASKS);

NonLinearPid cascadeRollPid(consKpCascRoll, consKiCascRoll, consKdCascRoll);
NonLinearPid cascadeRollPidW(consKpCascRollW, consKiCascRollW, consKdCascRollW);
NonLinearPid cascadePitchPid(consKpCascPitch, consKiCascPitch, consKdCascPitch);
NonLinearPid cascadePitchPidW(consKpCascPitchW, consKiCascPitchW, consKdCascPitchW);
NonLinearPid cascadeYawPid(consKpCascYaw, consKiCascYaw, consKdCascYaw);
NonLinearPid cascadeYawPidW(consKpCascYawW, consKiCascYawW, consKdCascYawW);
NonLinearPid cascadeAltPid(consKpCascAlt, consKiCascAlt, consKdCascAlt);

        
// Median Filter
MedianFilter medianGyroX(3,0);
MedianFilter medianGyroY(3,0);
MedianFilter medianGyroZ(3,0);

void setupTimerInterrupt()
{
  // Timer settings
  // Initialize Timer
  noInterrupts();
  TCCR3A = 0;
  TCCR3B = 0;

  // Set compare match register to the desired timer count
  OCR3A=14; //16*10^6/(1000Hz*1024)-1 = 77 -> 1068 Hz 
  //OCR3A=77; //16*10^6/(200Hz*1024)-1 = 77 -> 200 Hz 
  //OCR3A=193; //16*10^6/(80Hz*1024)-1 = 193 -> 80 Hz 
  //OCR3A=103; //16*10^6/(150Hz*1024)-1 = 103 -> 150 Hz 
  //OCR3A=143; //16*10^6/(109Hz*1024)-1 = 143 -> 109 Hz s
  //OCR3A=780; //16*10^6/(20Hz*1024)-1 = 780 -> 20 Hz 
  //OCR3A=2000; //16*10^6/(8Hz*1024)-1 = 780 -> 8 Hz 
  //OCR3A=50; //16*10^6/(308Hz*1024)-1 = 50 -> 308 Hz 

  TCCR3B |= (1 << WGM32);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR3B |= (1 << CS30) | (1 << CS32);
  // enable timer compare interrupt:
  TIMSK3 |= (1 << OCIE3B);

  // enable global interrupts:
  interrupts();
 
  if (!sakura.getProcessing())
  {
     Serial.println("[ OK ] Init Timers"); 
  }
}

void setupCtx()
{
  initializedSetup = true;
  k = micros();
  if (!sakura.getProcessing())
  {
     Serial.println("[ OK ] Init ctx"); 
  }
}

void setupCommunication()
{
  Wire.begin();
  Serial.begin(115200); 
  //Serial.begin(sakura.getBaudRate()); 
  if (!sakura.getProcessing())
  {
    Serial.println("[ Ok ] InitCOM ");
  }
  
  Serial1.begin(115200); 
  //Serial.begin(sakura.getBaudRate()); 
  if (!sakura.getProcessing())
  {
    Serial1.println("[ Ok ] InitCOM ");
  }
}

void setupIMU()
{ 
  sixDOF.init(); //init the Acc and Gyro
  delay(5);
  if (!sakura.getProcessing())
  {
    Serial.println("[ Ok ] IMU ");
  }
}
void setupSonar()
{ 
  ux1.init();
  if (!sakura.getProcessing())
  {
    Serial.println("[ Ok ] SONAR ");
  }
}

volatile int pinInit = 4;
volatile int pinEnd = 5;

void setupPinOut()
{
  pinMode(pinInit, OUTPUT); 
  pinMode(pinEnd, OUTPUT);
  digitalWrite(pinInit, LOW);  
  digitalWrite(pinEnd, LOW);
  if (!sakura.getProcessing())
  {
    Serial.println("[ OK ] Debug Pins Oscilloscope");
  }
}

void setupRTC()
{
  RTC.begin();
  //If we remove the comment from the following line, we will set up the module time and date with the computer one
  RTC.adjust(DateTime(__DATE__, __TIME__));  
  if (!sakura.getProcessing())
  {
    Serial.println("[ OK ] RTC");
  }
}

void setupLog()
{
  logger.init();
}

void setupPropulsion()
{
  tenzoProp.calibrateOnce();
  tenzoProp.init();  
}

void setup() {
  setupPinOut();
  setupCommunication();
  setupIMU();
  setupSonar();
  setupRTC();
  setupTimerInterrupt();  
  scheduler.initTaskset(); 
  scheduler.createTasks();
  setupCtx();  
  setupPropulsion();
  setupLog();   
  sakura.welcome();
}

void getYPR()
{  
    eulerTimer = micros();
  
    // Preemptable section
    //sei();            
      // [max] 9800 us [avg] 4450 us
      sixDOF.getYawPitchRoll(angles);  
    //cli();
    
    contEulerSamples++; 
    eulerTimer = micros() - eulerTimer;
    if (maxeulerTimer <= eulerTimer)
      maxeulerTimer = eulerTimer;
    eulerTimeTot = eulerTimeTot + eulerTimer;
}


void getGyro()
{  
    gyroTimer = micros();
  
    // Preemptable section
    //sei();            
      // [max] 9800 us [avg] 4450 us
    acquireGyro();
    //cli();
    
    contGyroSamples++; 
    gyroTimer = micros() - gyroTimer;
    if (maxgyroTimer <= gyroTimer)
      maxgyroTimer = gyroTimer;
    gyroTimeTot = gyroTimeTot + gyroTimer;
}

void getAngVelYPR()
{    
    gyroTimer = micros();
    eulerTimer = micros();
    
    //sei();            
      // [max]  6570 us [avg] 6290 us -> 155 Hz
    acquireGyroYPR();
    //cli();
    
    contEulerSamples++; 
    eulerTimer = micros() - eulerTimer;
    if (maxeulerTimer <= eulerTimer)
      maxeulerTimer = eulerTimer;
    eulerTimeTot = eulerTimeTot + eulerTimer;
    contGyroSamples++;        
    gyroTimer = micros() - gyroTimer;
    if (maxgyroTimer <= gyroTimer)
      maxgyroTimer = gyroTimer;
    gyroTimeTot = gyroTimeTot + gyroTimer; 

    //Serial.println("t\t\t\t\tANGLESSS");
}

void controlCycle()
{
  controlTimer = micros();
  // [max] 1300 us [avg] 1230 us      
  controlCascade();  // [OK]
  
  countCtrlCalc++;  
  controlTimer = micros() - controlTimer;
  if (maxcontrolTimer <= controlTimer)
    maxcontrolTimer = controlTimer;
  controlTimeTot = controlTimeTot + controlTimer;
}


void sonarPreRoutine()
{
  ux1.init();
}


void sonarRoutine()
{  
  sonarTimer = micros();
  
  // [max] 1300 us [avg] 1230 us     
  altitudeSonar = ux1.getDistance();
  
  contSonarRoutine++;  
  sonarTimer = micros() - sonarTimer;
  if (maxsonarTimer <= sonarTimer)
    maxsonarTimer = sonarTimer;
  sonarTimeTot = sonarTimeTot + sonarTimer;
}

void getDateTimeRTC()
{
  rtcTimer = micros();
  
  now = RTC.now();
  
  contRtcRoutine++;  
  rtcTimer = micros() - rtcTimer;
  if (maxrtcTimer <= rtcTimer)
    maxrtcTimer = rtcTimer;
  rtcTimeTot = rtcTimeTot + rtcTimer;
}

void loop() {  
  timerSec = micros() - secRoutine;

  //scheduler.checkPeriodicTasks();
  bestId = scheduler.schedule();
  
  switch(bestId)
  {
    case(1):
      // Task 1
      getAngVelYPR();
      controlCycle();
      scheduler.jobCompletedById(bestId);
      break;

    case(2):
      // Task 2
      SerialRoutine();
      scheduler.jobCompletedById(bestId);
      break;

    case(3):
      // Task 3
      Serial.println("\t\t\t\t\t\t\t\t\t\tGPS");
      scheduler.jobCompletedById(bestId);
      break;
      
    case(4):
      // Task 4 !!! DP !! #Sonar setup
      sonarPreRoutine();      
      scheduler.jobCompletedById(bestId);
      break;
      
    case(5):
      // Task 5 !!! DP !! #Sonar read
      sonarRoutine();      
      scheduler.jobCompletedById(bestId);
      break;
      
    case(6):
      // Task 6 !!! FP !! Dummy
      getDateTimeRTC();
      scheduler.jobCompletedById(bestId);
      Serial.print("\t\t\tRTC:");
      Serial.println(now.second());
      break;      
  }
    
  
  // #LOOP
  if (timerSec >= 1000000)
  {
    for(int i = 1; i<=scheduler.num_tasks;i++)
    {
      if (scheduler.isTaskAlive(i))
      {
        Serial.print("T"); Serial.print(i);
        Serial.print("\t(");
        Serial.print(scheduler.getTaskPeriod(i));
        Serial.print(",e,");
        Serial.print(scheduler.getTaskPriority(i));
        Serial.print(")\tJob queue:  ");
        Serial.print(scheduler.getJobReleased(i));
        Serial.print("\t");
        Serial.println(scheduler.getTaskLabel(i));
      }
    }
    computeAverageExecTime();
    
    UXRoutine();
    
    resetCounters();
    
    secRoutine = micros();
  }
  
}


void UXRoutine()
{
    printTimersSched();
    printRoutine();
}

void computeAverageExecTime()
{
    // Compute average servo readings time
    if (countServoAction >  0)
      servoTimeTot = servoTimeTot/countServoAction;
    else 
      servoTimeTot = -999;
        
    // Compute average euler readings time
    if (contEulerSamples != 0)
      eulerTimeTot = eulerTimeTot/contEulerSamples;
    else 
      eulerTimeTot = -999;
        
     
    // Compute average gyro readings time
    if (contGyroSamples >  0)
      gyroTimeTot = gyroTimeTot/contGyroSamples;
    else 
      gyroTimeTot = -999;
        
    
    // Compute average controlCascade readings time
    if (countCtrlCalc > 0)
      controlTimeTot = controlTimeTot/countCtrlCalc;
    else 
      controlTimeTot = -999;
        
    
    // Compute average isr readings time
    if (countISR >  0)
      isrTimeTot = isrTimeTot/countISR;
    else 
      isrTimeTot = -999;
        
    
    // Compute average SerialRoutine time
    if (contSerialRoutine >  0)
      serialTimeTot = serialTimeTot/contSerialRoutine;
    else 
      serialTimeTot = -999;
        
    
    // Compute average SonarRoutine time
    if (contSonarRoutine > 0)
      sonarTimeTot = sonarTimeTot/contSonarRoutine;
    else 
      sonarTimeTot = -999;

      // Compute average SonarRoutine time
    if (contRtcRoutine > 0)
      rtcTimeTot = rtcTimeTot/contRtcRoutine;
    else 
      rtcTimeTot = -999;
        
}

void resetCounters()
{    
    //cont=0;         
    contCalc=0; 
    countCtrlCalc=0;
    countServoAction=0;  
    contSerialRoutine=0;    
    contGyroSamples=0;     
    contSonarRoutine=0;     
    contRtcRoutine=0;     
    contEulerSamples=0;   
    
    servoTimeTot = 0;
    eulerTimeTot = 0;
    gyroTimeTot = 0;
    controlTimeTot = 0;
    sonarTimeTot = 0;
    rtcTimeTot = 0;
    countISR = 0;
}

void calcAngle() // #ISR
{
  // From Gyro
  dt = micros()-k;
  if (!sakura.getGyroFilterFlag())
  {
    phi=phi+(float) x*(float)dt/1000000.0;
    
    thetaOLD = theta; 
    theta=theta+(float) y*(float) dt/1000000.0;

    psi=psi+(float) z*(float) dt/1000000.0; 
  }
  else if (sakura.getGyroFilterFlag())
  {  
    phi=phi+wF[0]*(float)dt/1000000.0;

    thetaOLD = theta;
    theta=theta+wF[1]*(float) dt/1000000.0;

    psi=psi+wF[2]*(float) dt/1000000.0;  
  }
   
  if (sakura.getAccFilterFlag())
  {
    angleXAcc = (atan2(-aF[0],-aF[2])) * RAD_TO_DEG;
    angleYAcc = (atan2(-aF[1],-aF[2])) * RAD_TO_DEG;
  }
  else 
  {
    // From Acc
    angleXAcc = (atan2(-aax,-aaz)) * RAD_TO_DEG;
    angleYAcc = (atan2(-aay,-aaz)) * RAD_TO_DEG;
  }
  k=micros();  
}

void wFilter(volatile float val[])
{
  val[0] = (1-alphaW)*val[0] + alphaW*wxm1;
  val[1] = (1-alphaW)*val[1] + alphaW*wym1;
  val[2] = (1-alphaW)*val[2] + alphaW*wzm1;
  
  wxm1 = val[0];
  wym1 = val[1];
  wzm1 = val[2];
}

void aFilter(volatile float val[])
{
  val[0] = (1-alphaA)*val[0] + alphaA*axm1;
  val[1] = (1-alphaA)*val[1] + alphaA*aym1;
  val[2] = (1-alphaA)*val[2] + alphaA*azm1;
  
  axm1 = val[0];
  aym1 = val[1];
  azm1 = val[2];
}


ISR(TIMER3_COMPB_vect) // #ISR
{ 
  isrTimer = micros();
  // increments scheduler ticks
  ticks++;
  contCtrl++;
  // update Tasks info
  scheduler.checkPeriodicTasks();
  //if (scheduler.trigger_schedule)
    //bestId = scheduler.schedule();
  
  //digitalWrite(pinEnd, LOW);
  digitalWrite(pinInit, HIGH);
  

    if (contCtrl == ctrlPeriod)
    {
      servoTimer = micros();     
      // [max] 250 us [avg] 240 us   
      tenzoProp.setSpeeds(tenzoProp.getThrottle(), OutputCascPitchW, OutputCascRollW, OutputCascYawW, OutputCascAlt);
      // update counter control  
  
      countServoAction++;  
      servoTimer = micros() - servoTimer;
      
      if (maxservoTimer <= servoTimer)
        maxservoTimer = servoTimer;
      servoTimeTot = servoTimeTot + servoTimer;

      contCtrl = 0;
    }
        
  cont++;    
  countISR++;  
  
  // Compute isr time
  isrTimer = micros() - isrTimer;
  if (maxisrTimer <= isrTimer)
    maxisrTimer = isrTimer;
  isrTimeTot = isrTimeTot + isrTimer;  
  
  digitalWrite(pinInit, LOW);
}

void acquireGyroYPR()  // ISR
{
  sixDOF.getYawPitchRollGyro(angles,wVal);
  
  if (sakura.getGyroFilterFlag())
  {    
    medianGyroX.in(wVal[0]);
    wVal[0] = medianGyroX.out();   
   
    medianGyroY.in(wVal[1]);
    wVal[1] = medianGyroY.out();   
   
    medianGyroZ.in(wVal[2]);
    wVal[2] = medianGyroZ.out();    
  }   
}

void acquireGyro() // ISR
{     
  sixDOF.getGyroValues(wVal);
  
  if (sakura.getGyroFilterFlag())
  {    
    medianGyroX.in(wVal[0]);
    wVal[0] = medianGyroX.out();   
   
    medianGyroY.in(wVal[1]);
    wVal[1] = medianGyroY.out();   
   
    medianGyroZ.in(wVal[2]);
    wVal[2] = medianGyroZ.out();    
  }   
}

void SerialRoutine()
{
  serialTimer = micros();
  
  if (Serial.available())
  {
      char t = Serial.read();
      
      //if (t == '1')
      //  tenzoProp.stopAll();      
      if (t == 'c')
      {
        // Serial communication        
        Serial.println("K");
        sendStatesRemote = true;
      }      
      if (t == 'X')
      {
        // Serial communication        
        Serial.println("X");
        sendStatesRemote = true;
      } 
      else if (t == 'r')
        tenzoProp.resetMotors();
      else if (t == 'q')
      {
        tenzoProp.setThrottle(tenzoProp.getThrottle() + 10);
        if (!sakura.getProcessing())
        {
          Serial.print("m,");
          Serial.print(tenzoProp.getThrottle());
          Serial.println(",z");
        }
      }
      else if (t == 'a')
      {
        tenzoProp.setThrottle(tenzoProp.getThrottle() - 10);
        
        if (!sakura.getProcessing())
        {
          Serial.print("m,");
          Serial.print(tenzoProp.getThrottle());
          Serial.println(",z");
        }
      }
      // PID values modification from remote
      else if (t == 'u')
      { 
         k1ReadChar = Serial.read();
         // Read Pid values Requested csv format
         if (k1ReadChar == 44)
         {
           readChar2 = Serial.parseInt();
           
           if (readChar2 == 31)
           {
             // show roll Cons
             sendPidVal(0,0);           
           }
           else if (readChar2 == 32)
           {
             // show roll agg              
             sendPidVal(0,1);              
           }
           else if (readChar2 == 33)
           {
             // w Roll cons 
             sendPidVal(3,0);  
           } 
           else if (readChar2 == 34)
           {
             // show pitch cons  
             sendPidVal(1,0);              
           }
           else if (readChar2 == 35)
           {
             // pitch agg        
             sendPidVal(1,1);      
           }
           else if (readChar2 == 36)
           {           
             // w pitch cons 
             sendPidVal(4,0); 
           }  
           else if (readChar2 == 37)
           {
             // yaw cons
             sendPidVal(2,0);   
           }
           else if (readChar2 == 38)
           {
             // yaw agg        
             sendPidVal(2,1);      
           }
           else if (readChar2 == 39)
           {
             // w yaw cons
             sendPidVal(5,0); 
           } 
           // Setting pid values from remote #pid #remote
           // u,4,kp,ki,kd,set,z
           else if (readChar2 == 4)
           {
             // comma
             kReadChar = Serial.read();
             if (kReadChar == 44)
             {
               //Serial.println("KODIO");
               // read mode
               k3ReadInt = Serial.parseInt();
               //Serial.println(k3);
               
               kReadChar = Serial.read();
               if (kReadChar == 44)
               {
                 // read kp
                 readPropVal = Serial.parseFloat();
               //Serial.println(p);
                 
                 kReadChar = Serial.read();
                 if (kReadChar == 44)
                 {
                   // read ki
                   readIntVal = Serial.parseFloat();
               //Serial.println(i);
                   
                   kReadChar = Serial.read();
                   if (kReadChar == 44)
                   {
                     // read kd
                     readDerVal = Serial.parseFloat();
               //Serial.println(d);
                     
                     kReadChar = Serial.read();
                     if (kReadChar == 44)
                     {
                       // read new SetPoint
                       readSetVal = Serial.parseFloat();
               //Serial.println(set);
                     }
                   }
                 }
               }
               
               // Security Check [ IMPORTANT ]
               if (readPropVal>20)
                  readPropVal=20;
               if (readPropVal<0)
                  readPropVal=0;
               if (readDerVal>6)
                  readDerVal=6;
               if (readDerVal<0)
                  readDerVal=0;
               if (readIntVal>6)
                  readIntVal=6;
               if (readIntVal<0)
                  readIntVal=0;
                  // ADD setpoint check pensaci un po'
              
               if (k3ReadInt == 31)
               {
                 // show roll Cons
                 consKpCascRoll = readPropVal;
                 consKdCascRoll = readDerVal;
                 consKiCascRoll = readIntVal;
                 SetpointCascRoll = readSetVal;
                 
                 // Send actual Vals     
                 sendPidVal(0,0);           
               }
               else if (k3ReadInt == 32)
               {
                 // show roll agg  
                 aggKpCascRoll = readPropVal;
                 aggKdCascRoll = readDerVal;
                 aggKiCascRoll = readIntVal;
                 SetpointCascRoll = readSetVal;
                 
                 // Send actual Vals            
                 sendPidVal(0,1);              
               }
               else if (k3ReadInt == 33)
               {
                 // w Roll cons
                 consKpCascRollW = readPropVal;
                 consKdCascRollW = readDerVal;
                 consKiCascRollW = readIntVal;
                 SetpointCascRollW = readSetVal;
                 
                 // Send actual Vals  
                 sendPidVal(3,0);  
               } 
               else if (k3ReadInt == 34)
               {
                 // show pitch cons  
                 consKpCascPitch = readPropVal;
                 consKdCascPitch = readDerVal;
                 consKiCascPitch = readIntVal;
                 SetpointCascPitch = readSetVal;
                 
                 // Send actual Vals 
                 sendPidVal(1,0);              
               }
               else if (k3ReadInt == 35)
               {
                 // pitch agg 
                 aggKpCascPitch = readPropVal;
                 aggKdCascPitch = readDerVal;
                 aggKiCascPitch = readIntVal;
                 SetpointCascPitch = readSetVal;
                 
                 // Send actual Vals        
                 sendPidVal(1,1);      
               }
               else if (k3ReadInt == 36)
               {           
                 // w pitch cons 
                 consKpCascPitchW = readPropVal;
                 consKdCascPitchW = readDerVal;
                 consKiCascPitchW = readIntVal;
                 SetpointCascPitchW = readSetVal;
                 
                 // Send actual Vals 
                 sendPidVal(4,0); 
               }  
               else if (k3ReadInt == 37)
               {
                 // yaw cons
                 consKpCascYaw = readPropVal;
                 consKdCascYaw = readDerVal;
                 consKiCascYaw = readIntVal;
                 SetpointCascYaw = readSetVal;
                 
                 // Send actual Vals 
                 sendPidVal(2,0);   
               }
               else if (k3ReadInt == 38)
               {
                 // yaw agg  
                 aggKpCascYaw = readPropVal;
                 aggKdCascYaw = readDerVal;
                 aggKiCascYaw = readIntVal;
                 SetpointCascYaw = readSetVal;      
                 sendPidVal(2,1);      
               }
               else if (k3ReadInt == 39)
               {
                 // w yaw cons 
                 consKpCascYawW = readPropVal;
                 consKdCascYawW = readDerVal;
                 consKiCascYawW = readIntVal;
                 SetpointCascYawW = readSetVal;
                 
                 // Send actual Vals
                 sendPidVal(5,0); 
               }
             }             
           }
         }
      }
      else if (t == 'v')
      {        
        //if (!sakura.getProcessing())
        //{
          //Serial.println(tenzoProp.getThrottle());
          Serial.println();
          Serial.print("V,W - P|I|D: ");
          Serial.print(consKpCascRollW);
          Serial.print(" | ");
          Serial.print(consKiCascRollW);
          Serial.print(" | ");
          Serial.println(consKdCascRollW);
          Serial.print("V,A - P|I|D: ");
          Serial.print(consKpCascRoll);
          Serial.print(" | ");
          Serial.print(consKiCascRoll);
          Serial.print(" | ");
          Serial.println(consKdCascRoll);
        //}
      }      
      else if (t == 'x')
      {
        resetMotorsPidOff();
        
        tenzoProp.detachAll();
        
        // To add in method
      }    
      else if (t == 'i')
      {
        initialize();
        // Do not alter max serial Routine exec time
        initializing = true;
      }
      else if (t == 'L')
      {
        land();
        // Do not alter max serial Routine exec time
        landing = true;
      }
      else if (t == 'm')
      {
         sakura.setPrintMotorValsUs(!sakura.getPrintMotorValsUs()); 
      }
      else if (t == 'n')
      {
         sakura.setPrintAccs(!sakura.getPrintAccs()); 
      }
      else if (t == 'o')
      {
         //sakura.setPrintOmegas(!sakura.getPrintOmegas()); 
         printOmega();
      }
      else if (t == 'e')
      {
         //sakura.setPrintOmegas(!sakura.getPrintOmegas()); 
         printSerialAngleFus();
      }
      else if (t == 'z')
      {
        //sakura.setPrintPIDVals(!sakura.getPrintPIDVals());
        //ux1.printAltitude();
      }
      else if (t == 'p')
      {
        enablePid = !enablePid;
        changePidState(enablePid);
        //if (enablePid)
          //Serial.println("ce,z");
        //else if (!enablePid)
          //Serial.println("cd,z");
        sendStatesRemote = true;
      }
      else if (t == 'l')
      {
        printAcc();
      }         
      else if (t == 'b')
      {
        char k = Serial.read();
        if (k=='e')
          sakura.setSendBlueAngle(true);
        else if (k=='d')
          sakura.setSendBlueAngle(false);
      }         
      else if (t == ',')
      {
        // request pid val
        SetpointCascRoll = SetpointCascRoll - 1;
        Serial.print("V,SetpointCascRoll:  ");
        Serial.println(SetpointCascRoll);
      }         
      else if (t == ';')
      {
        SetpointCascRoll = SetpointCascRoll + 1;
        Serial.print("V,SetpointCascRoll:  ");
        Serial.println(SetpointCascRoll);
      }         
      else if (t == '.')
      {
        /*
        consKiWRoll = consKiWRoll + 0.05;
        Serial.print("\t\t\t\t\t\t\t\t\t");
        Serial.println(consKiWRoll);
        */
      }         
      else if (t == ':')
      {
        /*
        consKiWRoll = consKiWRoll - 0.05;
        if (consKiWRoll<=0)
          consKiWRoll = 0;
        Serial.print("\t\t\t\t\t\t\t\t\t");
        Serial.println(consKiWRoll);
        */
      }         
      else if (t == '-')
      {
        ux1.printAltitude();          
      }         
      else if (t == '_')
      {
        /*
        consKdWRoll = consKdWRoll - 0.02;
        if (consKdWRoll<=0)
          consKdWRoll = 0;
        Serial.print("\t\t\t\t\t\t\t\t\t");
        Serial.println(consKdWRoll);
        */
      }     
      else if (t == 't')
      {
        Serial.println("\t\t\tPorco Dia ");
      }         
      else if (t == 'y')
      {
        SetpointCascRoll = SetpointCascRoll + 1;
        Serial.print("V,SetpointCascRoll:  ");
        Serial.println(SetpointCascRoll);
      }    
      else if (t == 'd')
      {
        consKpCascRoll = consKpCascRoll - 0.05;
        if (consKpCascRoll<0)
          consKpCascRoll = 0;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(0,0);
        }          
      }         
      else if (t == 'D')
      {
        consKpCascRoll = consKpCascRoll + 0.05;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(0,0);
        }
      }     
      else if (t == 'f')
      {
        consKiCascRoll = consKiCascRoll - 0.05;
        if (consKiCascRoll<0)
          consKiCascRoll = 0;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(0,0); 
        }
      }         
      else if (t == 'F')
      {
        consKiCascRoll = consKiCascRoll + 0.05;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(0,0);
        }
      }     
      else if (t == 'g')
      {
        consKdCascRoll = consKdCascRoll - 0.05;
        if (consKdCascRoll<0)
          consKdCascRoll = 0;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(0,0);
        }
      }         
      else if (t == 'G')
      {
        consKdCascRoll = consKdCascRoll + 0.05;
        if (verbosePidValuesFeedback)
        {
           sendPidVal(0,0); 
        }
      }    
      else if (t == 'h')
      {
        consKpCascRollW = consKpCascRollW - 0.05;
        if (consKpCascRollW<0)
          consKpCascRollW = 0;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(3,0);
        } 
      }         
      else if (t == 'H')
      {
        consKpCascRollW = consKpCascRollW + 0.05;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(3,0);
        } 
      }     
      else if (t == 'j')
      {
        consKiCascRollW = consKiCascRollW - 0.05;
        if (consKiCascRollW<0)
          consKiCascRollW = 0;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(3,0);
        } 
      }         
      else if (t == 'J')
      {
        consKiCascRollW = consKiCascRollW + 0.05;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(3,0);
        } 
      }     
      else if (t == 'k')
      {
        consKdCascRollW = consKdCascRollW - 0.05;
        if (consKdCascRollW<0)
          consKdCascRollW = 0;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(3,0);
        } 
      }         
      else if (t == 'K')
      {
        consKdCascRollW = consKdCascRollW + 0.05;
        if (verbosePidValuesFeedback)
        {
          sendPidVal(3,0);
        } 
      }  
      else if (t == 'w')
      {
        alphaW = alphaW + 0.05;
        if (alphaW>=1)
          alphaW = 1;
        if (verboseFilterAccMatlab)
        {
          sendAlphaW();
        } 
      }     
      else if (t == 'W')
      {
        alphaW = alphaW + 0.01;
        if (alphaW>=1)
          alphaW = 1;
        if (verboseFilterAccMatlab)
        {
          sendAlphaW();
        } 
      }     
      else if (t == 's')
      {
        alphaW = alphaW - 0.05;
        if (alphaW<=0)
          alphaW = 0;
        if (verboseFilterAccMatlab)
        {
          sendAlphaW();
        } 
      }             
      else if (t == 'S')
      {
        alphaW = alphaW - 0.01;
        if (alphaW<=0)
          alphaW = 0;
        if (verboseFilterAccMatlab)
        {
          sendAlphaW();
        } 
      }       
      /*      
      else if (t == 'w')
      {
        alphaA = alphaA + 0.001;
        if (alphaA>=1)
          alphaA = 1;
        if (verboseFilterAccMatlab)
        {
          sendAlphaAcc();
        } 
      }     
      else if (t == 'W')
      {
        alphaA = alphaA + 0.01;
        if (alphaA>=1)
          alphaA = 1;
        if (verboseFilterAccMatlab)
        {
          sendAlphaAcc();
        } 
      }     
      else if (t == 's')
      {
        alphaA = alphaA - 0.001;
        if (alphaA<=0)
          alphaA = 0;
        if (verboseFilterAccMatlab)
        {
          sendAlphaAcc();
        } 
      }             
      else if (t == 'S')
      {
        alphaA = alphaA - 0.01;
        if (alphaA<=0)
          alphaA = 0;
        if (verboseFilterAccMatlab)
        {
          sendAlphaAcc();
        } 
      }       
      */
  }
  contSerialRoutine++;
  serialTimer = micros() - serialTimer;
  
    if (maxserialTimer <= serialTimer && !initializing && !landing)
      maxserialTimer = serialTimer;
  
  serialTimeTot = serialTimeTot + serialTimer;
  
  // restore initializing and landing to false
  initializing = false;
  landing = false;

  //Serial.println("\t\t\t\t\t\t\t\t\t\t\t\tSERIALEEE");
}

// Send accelerometer filter value to Matlab for dynamic change
void sendAlphaAcc()
{
  Serial.print("f,");
  Serial.print(alphaA);
  Serial.println(",z");
}

// Send gyro filter value to Matlab for dynamic change
void sendAlphaW()
{
  Serial.print("f,");
  Serial.print(alphaW);
  Serial.println(",z");
}

// Send pid value feedback to App
void sendPidVal(int which,int mode)
{
  /*           WHICH:
   * Roll = 0
   * Pitch = 1 
   * Yaw = 2
   * W Roll = 3
   * W Pitch = 4 
   * W Yaw = 5
   *           MODE
   * Cons = 0
   * Agg = 1
   */
  if (which == 0)
  {
    if (mode == 0)
    {
      Serial.print("rc,");
      Serial.print(consKpCascRoll);
      Serial.print(",");
      Serial.print(consKiCascRoll);
      Serial.print(",");
      Serial.print(consKdCascRoll);
      Serial.print(",");
      Serial.print(SetpointCascRoll);
      Serial.println(",z");
    } 
    else if (mode = 1)
    {
      Serial.print("ra,");
      Serial.print(aggKpCascRoll);
      Serial.print(",");
      Serial.print(aggKiCascRoll);
      Serial.print(",");
      Serial.print(aggKdCascRoll);
      Serial.print(",");
      Serial.print(SetpointCascRoll);
      Serial.println(",z");
    }                     
  }
  if (which == 1)
  {
    if (mode == 0)
    {
      Serial.print("pc,");
      Serial.print(consKpCascPitch);
      Serial.print(",");
      Serial.print(consKiCascPitch);
      Serial.print(",");
      Serial.print(consKdCascPitch);
      Serial.print(",");
      Serial.print(SetpointCascPitch);
      Serial.println(",z");
    } 
    else if (mode = 1)
    {
      Serial.print("pa,");
      Serial.print(aggKpCascPitch);
      Serial.print(",");
      Serial.print(aggKiCascPitch);
      Serial.print(",");
      Serial.print(aggKdCascPitch);
      Serial.print(",");
      Serial.print(SetpointCascPitch);
      Serial.println(",z");
    }                     
  }
  if (which == 2)
  {
    if (mode == 0)
    {
      Serial.print("yc,");
      Serial.print(consKpCascYaw);
      Serial.print(",");
      Serial.print(consKiCascYaw);
      Serial.print(",");
      Serial.print(consKdCascYaw);
      Serial.print(",");
      Serial.print(SetpointCascYaw);
      Serial.println(",z");
    } 
    else if (mode = 1)
    {
      Serial.print("ya,");
      Serial.print(aggKpCascYaw);
      Serial.print(",");
      Serial.print(aggKiCascYaw);
      Serial.print(",");
      Serial.print(aggKdCascYaw);
      Serial.print(",");
      Serial.print(SetpointCascYaw);
      Serial.println(",z");
    }                     
  }
  if (which == 3)
  {
    if (mode == 0)
    {
      Serial.print("rw,");
      Serial.print(consKpCascRollW);
      Serial.print(",");
      Serial.print(consKiCascRollW);
      Serial.print(",");
      Serial.print(consKdCascRollW);
      Serial.print(",");
      Serial.print(SetpointCascRollW);
      Serial.println(",z");
    }     
  }
  if (which == 4)
  {
    if (mode == 0)
    {
      Serial.print("pw,");
      Serial.print(consKpCascPitchW);
      Serial.print(",");
      Serial.print(consKiCascPitchW);
      Serial.print(",");
      Serial.print(consKdCascPitchW);
      Serial.print(",");
      Serial.print(SetpointCascPitchW);
      Serial.println(",z");
    } 
  }
  if (which == 5)
  {
    if (mode == 0)
    {
      Serial.print("yw,");
      Serial.print(consKpCascYawW);
      Serial.print(",");
      Serial.print(consKiCascYawW);
      Serial.print(",");
      Serial.print(consKdCascYawW);
      Serial.print(",");
      Serial.print(SetpointCascYawW);
      Serial.println(",z");
    } 
  }
}

void printTimersSched()
{
    if (sakura.getPrintTimers())
    {
      // Print Samples rate: [sample/sec] \t execTime \t wcet
      //ISR
      Serial.print("t,ISR: ");
      Serial.print(countISR);
      Serial.print("\t");
      Serial.print(isrTimeTot);
      Serial.print("\tMax ");
      Serial.print(maxisrTimer);
      
      Serial.print(",\nCtrl: ");
      Serial.print(countCtrlCalc);
      Serial.print("\t");
      Serial.print(controlTimeTot);
      Serial.print("\t Max");
      Serial.print(maxcontrolTimer);
      
      Serial.print(",\nGyro: ");
      Serial.print(contGyroSamples);
      Serial.print("\t");
      Serial.print(gyroTimeTot);
      Serial.print("\tMax ");      
      Serial.print(maxgyroTimer);
      
      Serial.print(",\nServo: ");
      Serial.print(countServoAction);
      Serial.print("\t");
      Serial.print(servoTimeTot);
      Serial.print("\tMax ");
      Serial.print(maxservoTimer);

      if (SONAR)
      {
        Serial.print(",\nSonar: ");
        Serial.print(contSonarRoutine);
        Serial.print("\t");
        Serial.print(sonarTimeTot);
        Serial.print("\tMax ");
        Serial.print(maxsonarTimer);
      }
      
      if (RTC_ON)
      {
        Serial.print(",\nRTC:\t ");
        Serial.print(contRtcRoutine);
        Serial.print("\t");
        Serial.print(rtcTimeTot);
        Serial.print("\tMax ");
        Serial.print(maxrtcTimer);
      }

      
      
      Serial.print(",\nSerial: ");
      Serial.print(contSerialRoutine);
      Serial.print("\t");
      Serial.print(serialTimeTot);
      Serial.print("\tMax ");
      Serial.println(maxserialTimer);
      
      if (enablePid)
      {
        Serial.print("\t");
        Serial.print(OutputCascRollW);
        // Print    timeservo: 
        Serial.print(",\t");        
        Serial.println(OutputCascPitchW);
      }
      
      //Serial.println(",z");
      Serial.println();
    }
}

void printRoutine()
{  
  /*  // #doing 
  */
  if (sakura.getPrintMotorValsUs())
  {
    Serial.print("V,  ");
    Serial.print(tenzoProp.getwUs1());
    Serial.print(" | ");
    Serial.print(tenzoProp.getwUs2());
    Serial.print(" | ");
    Serial.print(tenzoProp.getwUs3());
    Serial.print(" | ");
    Serial.println(tenzoProp.getwUs4());
  }
    
  if (sakura.getPrintAccs())
    printAcc();
  if (sakura.getPrintOmegas())
    printOmega();
    
  if (sakura.getSendBlueAngle())
  {
   printSerialAngleFus();
   //printSerialAngleNew();
  }
  
  if (sendStatesRemote)
    sendStates();
  //printT();
}

void sendStates()
{
  Serial.print("S,");
  Serial.print(initialized);
  Serial.print(",");
  Serial.print(hovering);
  Serial.print(",");
  Serial.print(landed);
  Serial.println(",z");
  
  sendStatesRemote = false;
}

void landFast()
{
  for (int j=tenzoProp.getThrottle(); j>40 ;j--)
  {
    tenzoProp.setSpeedWUs(j);
    //Serial.println(j);
    // Kind or brutal land
    delay(motorRampDelayFast);
  }  
  tenzoProp.idle();
  landed = 0;
}

void initializeFast()
{
  if (tenzoProp.getThrottle() == MIN_SIGNAL)
  {
   for (int j=MIN_SIGNAL; j<rampTill;j++)
   {
      tenzoProp.setSpeedWUs(j);
      //Serial.println(j);
      delay(motorRampDelayFast); 
   }
  }
  else if (tenzoProp.getThrottle()<=rampTill)
  {
   for (int j=tenzoProp.getThrottle() ; j<rampTill;j++)
   {
      tenzoProp.setSpeedWUs(j);
      //Serial.println(j);
      delay(motorRampDelayFast); 
   }
  }
  else if (tenzoProp.getThrottle()<=rampTill)
  {
   Serial.println("V,TODO: initialize FAST called unexpectedly");
  }
  tenzoProp.setThrottle(rampTill);
}


void initialize()
{
  if (!initialized)
  {
    if (!sakura.getProcessing())
        Serial.println("V,Initializing");
    initializing = true;
    tenzoProp.resetMotors();
    delay(500);
    for (int j=MIN_SIGNAL; j<rampTill;j++)
    {
      tenzoProp.setSpeeds(j, OutputCascPitchW, OutputCascRollW, OutputCascYawW, OutputCascAlt);
      //motorSpeed(j);
      if (!sakura.getProcessing())
        Serial.println(j);
      delay(motorRampDelayMedium); 
    }
    tenzoProp.setThrottle(rampTill);
    
    if (enablePid)
    {  
      changePidState(true);
    }
    else
    {
      changePidState(false);
    }    
    checkpoint = millis();
    initialized = true;    
    initializing = false;
    hovering = 0;
    landed = 0;
  }
  else
  {
    Serial.println();
    Serial.print("V,First Land ortherwise Tenzo will crash");
    Serial.println();
  }
  sendStatesRemote = true;
}


void resetMotorsPidOff()
{
  tenzoProp.setThrottle(MIN_SIGNAL);
  tenzoProp.setSpeedWUs(MIN_SIGNAL);
  // Sets timerStart to 0
  timerStart = 0;
  checkpoint = 0;
  // Disable Pid when motors are off
  if (enablePid)
  {
    // Change only if PID is enabled 
    // Tenzo has landed, no need to control
    changePidState(false);
  }
} 



void land()
{
  if (initialized)
  {
    landing = true;
    if(!sakura.getProcessing())
    {
      Serial.println();
      Serial.print("V,Landing protocol started...");
      Serial.print(tenzoProp.getThrottle());
      Serial.print(" ");
    }
    for (int j=tenzoProp.getThrottle(); j>MIN_SIGNAL ;j--)
    //for (int j=throttle; j>0 ;j--)
    {      
      tenzoProp.setSpeeds(j, OutputCascPitchW, OutputCascRollW, OutputCascYawW, OutputCascAlt);
      //motorSpeed(j);
      Serial.println(j);
      // Kind or brutal land
      if (landSpeed == 1)
        delay(motorRampDelayFast);
      else if (landSpeed == 2)
        delay(motorRampDelayMedium);
      else if (landSpeed == 3)
        delay(motorRampDelaySlow);
      else
        delay(motorRampDelayVerySlow);
    }
    resetMotorsPidOff();
    initialized = false;
    landing = false;;
    // updateStates
    landed=1;
    takeOff=0;
    hovering=0;
    sendStatesRemote = true;
  }
  else
  {
    Serial.println();
    Serial.print("V,Land command received but Tenzo is not Flying   !! WARNING !!");
    Serial.println();
    sendStatesRemote = true;
  }
}


void protocol1()
{  
  if (autoLand)
  {
    // If motors are on updates timer
    if (initialized)
      timerStart = millis() - checkpoint;
    // if Tenzo has already been initialized for a timeToLand period, then land
    if (initialized && timerStart>=timeToLand)
    {
      Serial.println("V,Time to Land my friend!");
      land();
    }
  }
}

void printOmega()
{
  Serial.print("o,");
  Serial.print(wVal[0]);
  Serial.print(",");
  Serial.print(wVal[1]);
  Serial.print(",");
  Serial.print(wVal[2]);
  Serial.println(",z");
}

void printAcc()
{
  if (!sakura.getAccFilterFlag())
  {
    Serial.print("a,");
    Serial.print(aax);
    Serial.print(",");
    Serial.print(aay);
    Serial.print(",");
    Serial.print(aaz);
    Serial.println(",z");
  }
  else
  {
    Serial.print("a,");
    Serial.print(aF[0]);
    Serial.print(",");
    Serial.print(aF[1]);
    Serial.print(",");
    Serial.print(aF[2]);
    Serial.println(",z");
  }
  
  /*
  Serial.print("a,");
  Serial.print(angleXAcc);
  Serial.print(",");
  Serial.print(angleYAcc);
  Serial.print(",");
  Serial.print(estXAngle);
  Serial.print(",");
  Serial.print(estYAngle);
  Serial.println(",z");
  */
  
}

void printSerialAngle()
{
  Serial.print(phi);
  Serial.print(",");
  Serial.print(theta);
  Serial.print(",");
  Serial.print(psi);
  Serial.print(",");
  Serial.print(angleXAcc);
  Serial.print(",");
  Serial.print(angleYAcc);
  Serial.print(",");
  Serial.print(estXAngle);
  Serial.print(",");
  Serial.print(estYAngle);
  Serial.print(",");
  Serial.print(bearing1);
  Serial.println(",z");
}

void printSerialAngleNew()
{
  Serial.print("P");
  Serial.print(",");
  Serial.print(phi);
  Serial.print(",");
  Serial.print(theta);
  Serial.print(",");
  Serial.print(psi);
  Serial.print(",");
  Serial.print(angleXAcc);
  Serial.print(",");
  Serial.print(angleYAcc);
  Serial.print(",");
  Serial.print(estXAngle);
  Serial.print(",");
  Serial.print(estYAngle);
  Serial.print(",");
  Serial.print(bearing1);
  Serial.println(",z");
}

void printSerialAngleFus()
{
  Serial.print("e");
  Serial.print(",");
  Serial.print(angles[0]);
  Serial.print(",");
  Serial.print(angles[1]);
  Serial.print(",");
  Serial.print(angles[2]);
  Serial.println(",z");
}

void controlCascade()  // ISR
{
  if (enablePid)
  {
    /*
     *
     *  Cascade Roll 
     *
     */
    if (enableRollPid)
    {
      InputCascRoll = angles[0];
      errorCascRoll = abs(SetpointCascRoll - angles[0]);
      
      if (inConsRoll)
      {
        if (errorCascRoll<=(thresholdRoll + 3))
        {
          cascadeRollPid.SetTunings(consKpCascRoll, consKiCascRoll, consKdCascRoll);
          inConsRoll = true; 
        }
        else if (errorCascRoll>thresholdRoll)
        {
          cascadeRollPid.SetTunings(aggKpCascRoll, aggKiCascRoll, aggKdCascRoll);
          inConsRoll = false;
        }
      }
      else
      {
        if (errorCascRoll<=(thresholdRoll))
        {
          cascadeRollPid.SetTunings(consKpCascRoll, consKiCascRoll, consKdCascRoll);
          inConsRoll = true; 
        }
        else if (errorCascRoll>thresholdRoll)
        {
          cascadeRollPid.SetTunings(aggKpCascRoll, aggKiCascRoll, aggKdCascRoll); 
          inConsRoll = false;
        }
      }
      
      OutputCascRoll = cascadeRollPid.Compute(SetpointCascRoll,InputCascRoll); 
      
      InputCascRollW = wVal[0];
      SetpointCascRollW = OutputCascRoll;
      //////////////////////////////////////
      //qSetpointCascRollW = 0;
      errorCascRollW = SetpointCascRollW - wVal[0]; 

      cascadeRollPidW.SetTunings(consKpCascRollW, consKiCascRollW, consKdCascRollW);

      OutputCascRollW = cascadeRollPidW.Compute(SetpointCascRollW,InputCascRollW);     
      /*
      if (sakura.getPrintPIDVals())
      {
        Serial.print("V,I: ");
        Serial.print(InputCascRoll);
        Serial.print(" O: ");
        Serial.print(OutputCascRoll);
        Serial.print(" Iw: ");
        Serial.print(InputCascRollW);
        Serial.print(" Ew: ");
        Serial.print(errorCascRollW);
        Serial.print(" O: ");
        Serial.print(OutputCascRollW);
        Serial.println();
      }
      */
    }
    else
    {
      OutputCascRollW = 0;
    }
    
    /*
     *
     *  Cascade Pitch 
     *
     */
    if (enablePitchPid)
    {
      InputCascPitch = angles[1];
      errorCascPitch = abs(SetpointCascPitch - angles[1]);
      
      if (inConsPitch)
      {
        if (errorCascPitch<=(thresholdPitch + 3))
        {
          cascadePitchPid.SetTunings(consKpCascPitch, consKiCascPitch, consKdCascPitch);
          inConsPitch = true; 
        }
        else if (errorCascPitch>thresholdRoll)
        {
          cascadePitchPid.SetTunings(aggKpCascPitch, aggKiCascPitch, aggKdCascPitch);
          inConsPitch = false;
        }
      }
      else
      {
        if (errorCascPitch<=(thresholdPitch))
        {
          cascadePitchPid.SetTunings(consKpCascPitch, consKiCascPitch, consKdCascPitch);
          inConsPitch = true; 
        }
        else if (errorCascPitch>thresholdPitch)
        {
          cascadePitchPid.SetTunings(aggKpCascPitch, aggKiCascPitch, aggKdCascPitch); 
          inConsPitch = false;
        }
      }
      
      OutputCascPitch = cascadePitchPid.Compute(SetpointCascPitch,InputCascPitch); 
      
      InputCascPitchW = wVal[1];
      SetpointCascPitchW = OutputCascPitch;
      //SetpointCascPitchW = 0;
      errorCascPitchW = SetpointCascPitchW - wVal[1]; 

      cascadePitchPidW.SetTunings(consKpCascPitchW, consKiCascPitchW, consKdCascPitchW);

      OutputCascPitchW = cascadePitchPidW.Compute(SetpointCascPitchW,InputCascPitchW);     
      
      /*
      if (sakura.getPrintPIDVals())
      {
        Serial.print("I: ");
        Serial.print(InputCascRoll);
        Serial.print(" O: ");
        Serial.print(OutputCascRoll);
        Serial.print(" Iw: ");
        Serial.print(InputCascRollW);
        Serial.print(" Ew: ");
        Serial.print(errorCascRollW);
        Serial.print(" O: ");
        Serial.print(OutputCascPitchW);
        Serial.println();
      }
      */
    }
    else
    {                        
      OutputCascPitchW = 0;
    }
  }
  
}


void changePidState(boolean cond)
{
  if (cond)
  {
    // Enable Cascade
    cascadeRollPid.restart();
    cascadeRollPid.SetOutputLimits(-limitPidMax, limitPidMax);
    cascadeRollPidW.restart();
    cascadeRollPidW.SetOutputLimits(-limitPidMax, limitPidMax);
    
    cascadePitchPid.restart();
    cascadePitchPid.SetOutputLimits(-limitPidMax, limitPidMax);
    cascadePitchPidW.restart();
    cascadePitchPidW.SetOutputLimits(-limitPidMax, limitPidMax);
    
    
    cascadeYawPid.restart();
    cascadeYawPid.SetOutputLimits(-limitPidMax, limitPidMax);
    cascadeYawPidW.restart();
    cascadeYawPidW.SetOutputLimits(-limitPidMax, limitPidMax);
    
    cascadeAltPid.restart();
    cascadeAltPid.SetOutputLimits(-limitPidMax, limitPidMax);
    

    enablePid = true;
    hovering = 1;
  }
  else
  { 
    // Cascade
    cascadeRollPid.pause();
    cascadeRollPidW.pause();
    cascadePitchPid.pause();
    cascadePitchPidW.pause();
    cascadeYawPid.pause();
    cascadeYawPidW.pause();
    cascadeAltPid.pause();
    
    OutputCascPitch = 0,OutputCascPitchW = 0;
    OutputCascRoll = 0, OutputCascRollW = 0;
    OutputCascYaw = 0, OutputCascYawW = 0;
    OutputCascAlt = 0;
    
    enablePid = false;
    hovering = 0;
  }
}
