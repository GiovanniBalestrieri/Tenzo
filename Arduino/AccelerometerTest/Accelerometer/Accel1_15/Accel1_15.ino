//#define DEBUGMODE

const float RESOLUTION=800; //0.8 v/g -> resolucion de 1.5g -> 800mV/g
const float VOLTAGE=3.3;  //voltage al que está conectado el acelerómetro

const float ZOUT_1G = 850;   // mv Voltage en Zout a 1G

const int NADJ  = 50;        // Número de lecturas para calcular el error

// Entradas analógicas donde van los sensores
const int xaxis = 0;
const int yaxis = 1;
const int zaxis = 2;

float XError,YError,ZError;
float xd,yd,zd,z,x,y;

// Acc Timers
unsigned long accTimer;
unsigned long lastAccTimer;
unsigned long timeToRead = 0;
unsigned long lastTimeToRead = 0;


// delta T control the routine frequency
float deltaT = 5;
float timerLoop = 0, timerReading = 0, timerSec = 0;
float timerRoutine = 0, count = 0;
float redingTime = 0, samplingTime = 0, calcTime =0, printTime = 0;
float k=0, kM1=0, kMReading = 0, kMRoutine=0, kMLoop=0, secRoutine=0;

byte mode;

// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);



boolean processing = false;
// Timer variables
volatile int cont = 0;
volatile int contSamples=0;
volatile int contCalc=0;

volatile float angleXAcc;
volatile float angleYAcc;

volatile float aax,aay,aaz;
volatile float alphaA = 0.2, alphaW = 0.8;

void setup()
{  
 Serial.begin(115200); // 9600 bps
 pinMode(xaxis,INPUT);
 pinMode(yaxis,INPUT);
 pinMode(zaxis,INPUT);

 XError =  AccelAdjust(xaxis);
 Serial.print("Error X: ");
 Serial.println(XError);
 YError =  AccelAdjust(yaxis);
 Serial.print("Error Y: ");
 Serial.println(YError);
 ZError =  AccelAdjust(zaxis);
 Serial.print("Error Z: ");
 Serial.println(ZError);
 ZError = ZError - ZOUT_1G;
 Serial.print("now: ");
 Serial.println(ZError);
 
 // Timer settings
   // Initialize Timer
  cli();
  TCCR3A = 0;
  TCCR3B = 0;

  // Set compare match register to the desired timer count
  //OCR3A=77; //16*10^6/(200Hz*1024)-1 = 77 -> 200 Hz 
  OCR3A=193; //16*10^6/(80Hz*1024)-1 = 193 -> 80 Hz 
  //OCR3A=780; //16*10^6/(20Hz*1024)-1 = 780 -> 20 Hz 
  //OCR3A=50; //16*10^6/(308Hz*1024)-1 = 50 -> 308 Hz 

  TCCR3B |= (1 << WGM32);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR3B |= (1 << CS30) | (1 << CS32);
  // enable timer compare interrupt:
  TIMSK3 |= (1 << OCIE3B);

  // enable global interrupts:
  sei(); 
 
 // ADC Tuning
 ADCSRA &= ~PS_128;  // remove bits set by Arduino library
 // you can choose a prescaler from above.
 // PS_16, PS_32, PS_64 or PS_128
 ADCSRA |= PS_32;    // set our own prescaler to 64 
 
}

void loop()
{
 //accRoutine(); 
 serialRoutine();
 delay(20);
} 

ISR(TIMER3_COMPB_vect)
{  
  // Mettilo altrimenti non funziona
  sei();
  // Update x, y, and z with new values 100 ns     
  //read values
  //getGyroValues(); 
  getAcc();
  calcAngle();
  cont++;
}

void getAcc() //ISR
{
   x=analogRead(xaxis);
   y=analogRead(yaxis);
   z=analogRead(zaxis);
  
   aax = (((x*5000.0)/1023.0)-XError)/RESOLUTION;
   aay = (((y*5000.0)/1023.0)-YError)/RESOLUTION;
   aaz = (((z*5000.0)/1023.0)-ZError)/RESOLUTION;
  
   // gets the value sample time
   accTimer = micros() - lastAccTimer;
   // updates last reading timer
   lastAccTimer = micros();  
}

void calcAngle() //ISR
{
  /*
   volatile float accs[3];        
   accs[0] = (((aax*5000.0)/1023.0)-XError)/RESOLUTION;  
   accs[1] = (((aay*5000.0)/1023.0)-YError)/RESOLUTION;
   accs[2] = (((aaz*5000.0)/1023.0)-ZError)/RESOLUTION;
   */
   /*
   accButter3(accs);
   axF = accs[0];
   ayF = accs[1];
   azF = accs[2];
   
   */
//  angleXAcc = (atan2(-accs[0],accs[2])) * RAD_TO_DEG;
//  angleYAcc = (atan2(accs[1],accs[2])) * RAD_TO_DEG;
  angleXAcc = (atan2(-aax,aaz)) * RAD_TO_DEG;
  angleYAcc = (atan2(aay,aaz)) * RAD_TO_DEG;
}

void wFilter(volatile float val[])
{
  val[0] = (1-alphaW)*x + alphaW*val[0];
  val[1] = (1-alphaW)*y + alphaW*val[1];
  val[2] = (1-alphaW)*z + alphaW*val[2];
}

void accFilter(volatile float val[])
{
  val[0] = (1-alphaA)*aax + alphaW*val[0];
  val[1] = (1-alphaA)*aay + alphaW*val[1];
  val[2] = (1-alphaA)*aaz + alphaW*val[2];
}

void serialRoutine()
{
  
  timerSec = micros()-secRoutine;
   lastTimeToRead = micros();
   
   if (timerSec >= 1000000)
  {
    secRoutine = micros();
    Serial.print(cont);
    Serial.print("    Smpl: ");
    Serial.print(contSamples);
    Serial.print("    calc: ");
    Serial.println(contCalc);
    /*
    Serial.print("    wX: ");
    Serial.print(wF[0]);
    Serial.print("    Wy: ");
    Serial.println(wF[1]);
    Serial.print("    Wz: ");
    Serial.println(wF[2]);
    */
    cont=0;      
    contSamples=0;      
    contCalc=0;      
  }

  timerRoutine = micros()-kMRoutine;
  if (timerRoutine >= deltaT*1000)
  {      
    kMRoutine = micros();    
    count += 1;
    if (count >= 3)
    {
      count = 0;
      //printSerialAngle();
      printAcc();
      //printT();
    }
  }
}

void printAcc()
{
  Serial.print(aax);
   Serial.print(",");
   Serial.print(aay);
   Serial.print(",");
   Serial.print(aaz);
   Serial.print(",");
   Serial.println("E");
}

void printSerialAngle()
{
   Serial.print(angleXAcc);
   Serial.print(",");
   Serial.println(angleYAcc);
}

float AccelAdjust(int axis)
{
 float acc = 0;
 for (int j=0;j<NADJ;j++)
 {
   float lectura=analogRead(axis);
   acc = acc + ((lectura*5000)/1023.0);
   delay(11); //número primo para evitar ciclos de lectura proporcionales
 }
 return acc/NADJ;
}
