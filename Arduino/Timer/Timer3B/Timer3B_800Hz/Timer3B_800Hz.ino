volatile int cont = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);

  // Initialize Timer
  cli();          // disable global interrupts
  TCCR3A = 0;     // set entire TCCR3A register to 0
  TCCR3B = 0;     // same for TCCR3B

  // set compare match register to desired timer count: 800 Hz
  OCR3A = 20; // 20; //800Hz 5; // 3 Hz
  // turn on CTC mode:
  TCCR3B |= (1 << WGM32);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR3B |= (1 << CS30) | (1 << CS32);
  // enable timer compare interrupt:
  TIMSK3 |= (1 << OCIE3B);
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

ISR(TIMER3_COMPB_vect)
{
  cont++;
}
