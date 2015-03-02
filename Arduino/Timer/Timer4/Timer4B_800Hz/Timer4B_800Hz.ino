volatile int cont = 0;
#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void setup()
{
  Serial.begin(9600);
  
  
  servo1.attach(3);  
  servo2.attach(5);    
  servo3.attach(6);   
  servo4.attach(9);

  servo1.write(0);  
  servo2.write(0); 
  servo3.write(0); 
  servo4.write(0);

  // Initialize Timer
  cli();          // disable global interrupts
  TCCR4A = 0;     // set entire TCCR3A register to 0
  TCCR4B = 0;     // same for TCCR3B

  // set compare match register to desired timer count: 800 Hz
  OCR4A = 20; // 20; //800Hz 5; // 3 Hz
  // turn on CTC mode:
  TCCR4B |= (1 << WGM42);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR4B |= (1 << CS40) | (1 << CS42);
  // enable timer compare interrupt:
  TIMSK4 |= (1 << OCIE4A);
  // enable global interrupts:
  sei();
  Serial.println("Setup completed");
}

void loop()
{
  delay(1000);
  Serial.println(cont);
  cont = 0;
}

ISR(TIMER4_COMPA_vect)
{
  cont++;
}
