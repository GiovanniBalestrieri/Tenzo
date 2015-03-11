/************************************************************
 *******************          Wiring:         **************
*************************************************************
Arduino Pin A0  ->  AttoPilot V
Arduino Pin A1  ->  AttoPilot I  -> doesn't work
Arduino Pin 9  ->  Sabertooth S1
Arduino Pin 10 ->  Sabertooth S2
Arduino Pin 2 -> encoder M1
Arduino GND    ->  Sabertooth 0V
Arduino GND    ->  AttoPilot GND
Arduino VIN    ->  Sabertooth 5V (OPTIONAL, Sabertooth powers Arduino)

*/

#define NUMEROIMPULSI 2078.4
#define DISTRUOTE 0.59
#define RAGGIORUOTA 0.13
#define KRAPP 1

#define TCAMP 5000

//variabili per encoder motore 1 (Lx)
volatile long int M1encoderPos = 0;
int M1deltaPos = 0;
long int M1oldPos = 0;
char M1verso = 1;

float M1velAng = 0;
float M1velLin = 0;

//variabili per encoder motore 2 (Dx)
volatile int M2encoderPos = 0;
int M2deltaPos = 0;
int M2oldPos = 0;
char M2verso = 1;

float M2velAng = 0;
float M2velLin = 0;



//variabili rover
float velLinAss = 0;
float omegaZAss = 0;
float velXAss = 0;
float velYAss = 0;  
float xAss = 0;
float yAss = 0;
float thetaZAss = 0;


//variabili controllore
long tOld = 0;





int count = 0;
#include <Servo.h>

// Sabertooth control pins
// Sabertooth accepts servo pulses from 1000 us to 2000 us.
// We need to specify the pulse widths in attach(). 0 degrees will be full reverse, 180 degrees will be
// full forward. Sending a servo command of 90 will stop the motor. Whether the servo pulses control
// the motors individually or control throttle and turning depends on your mixed mode setting.
  
Servo ST1, ST2;

int VRaw; //This will store our raw ADC data
int IRaw;
float VFinal; //This will store the converted data
float IFinal;

void setup() 
{
  Serial.begin(9600);
  attachInterrupt(0, M1encoder, RISING);
  
  
    
// Notice these attach() calls. The second and third arguments are important.
// With a single argument, the range is 44 to 141 degrees, with 92 being stopped.
// With all three arguments, we can use 0 to 180 degrees, with 90 being stopped.
  ST1.attach( 9, 1000, 2000);
  ST2.attach(10, 1000, 2000);
}


void loop() 
{ 
  if(micros()-tOld >= TCAMP){//sistemare overflow
    tOld = micros();
    misure();
    count += 1;
    if(count >= 10){
      count = 0;
      odometry();
    }
  }
  sabertooth();
  //attoPilot();  
}

void misure(){
  M1deltaPos = (M1encoderPos - M1oldPos); 
  M1oldPos = M1encoderPos;
  
  M2deltaPos = M1deltaPos;//(M2encoderPos - M2oldPos); 
  M2oldPos = M2encoderPos;
  
  M1velAng = (PI*M1deltaPos)/(180*NUMEROIMPULSI)*1000000/TCAMP;
  M2velAng = M1velAng;//(PI*M2deltaPos)/(180*NUMEROIMPULSI)*1000000/TCAMP;;
  
  M1velLin = M1velAng*RAGGIORUOTA;
  M2velLin = M1velLin;//M2velAng*RAGGIORUOTA;
  
  velLinAss = (M1velLin+M2velLin)/2;
  omegaZAss = (M1velLin-M2velLin)/DISTRUOTE;
  
  thetaZAss = thetaZAss + omegaZAss*TCAMP/(1000000);
  
  velXAss = velLinAss*sin(thetaZAss);
  velYAss = velLinAss*cos(thetaZAss);
  
  
  xAss = xAss + velXAss*TCAMP/(1000000);
  yAss = yAss + velYAss*TCAMP/(1000000);
  
}

void odometry(){
    Serial.print("thetaZAss: ");
    Serial.print(thetaZAss);
    Serial.print(" xAss: ");
    Serial.print(xAss);
    Serial.print(" yAss: ");
    Serial.print(yAss);
    Serial.print(" velYAss: ");
    Serial.print(velYAss);
    Serial.print(" velXAss: ");
    Serial.print(velXAss);
    Serial.print(" M1velLin: ");
    Serial.print(M1velLin);
    Serial.print(" M1velAng: ");
    Serial.println(M1velAng);
    

}

void attoPilot()
{
  //Measurement
  VRaw = analogRead(A0);
  IRaw = analogRead(A1);  // doesn't work ...
  
  //Conversion
  VFinal = VRaw/49.44; //45 Amp board
  
  IFinal = IRaw/14.9; //45 Amp board
    
  Serial.print(VFinal);
  Serial.println("   Volts");
  Serial.print(IFinal);
  Serial.println("   Amps");
  Serial.println("encoder");
  
  
  delay(200);   
}

void sabertooth()
{
  int power;
  
  // Ramp both servo channels from 0 to 180 (full reverse to full forward),
  // waiting 20 ms (1/50th of a second) per value.
  //for (power = 0; power <= 180; power ++)
  //{
    if (power >= 90){
       M1verso = 1; 
    }else{
       M1verso = -1; 
    }
    ST1.write(110);
    ST2.write(110);
    //delay(20);
    
  //}
  
  // Now go back the way we came.
//  for (power = 180; power >= 0; power --)
//  {
//    count += 1;
//    if (power >= 90){
//       verso = 1; 
//    }else{
//       verso = -1; 
//    }
//    ST1.write(power);
//    ST2.write(power);
//    delay(20);
//    if(count >= 20){
//       attoPilot();
//       count = 0; 
//    }
//  }
}

void M1encoder(){
   M1encoderPos += M1verso;  
}


void M2encoder(){
   M2encoderPos += M2verso;  
}
