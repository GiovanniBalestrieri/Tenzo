#define SECOND_US 1000000
#define NUM_BLADES 2
#define SEC_IN_MIN 60

volatile int rev_sec = 0;
volatile int rev_min = 0;
volatile float rad_sec = 0;
boolean working = false;
const byte interruptPin = 2;
volatile byte state = LOW;
volatile long counter = 0;
unsigned long timeTracker = 0;

boolean printRevSec = true;
boolean printRevMin = true;
boolean printRadSec = true;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  Serial.println("Yo");
  
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count, FALLING);
  pinMode(13, OUTPUT);
  
  timeTracker = micros();
}

void loop() {
  serialRoutine();
  computeRevolutions();
}

void computeRevolutions() {
  if (micros() - timeTracker >= SECOND_US) {
    rev_sec = counter/NUM_BLADES;
    rev_min = rev_sec*SEC_IN_MIN;
    rad_sec = rev_sec*3.1415/180;
    counter = 0;  
    timeTracker = micros();
  }  
}

void count() { // ISR
  counter++;
}

void serialRoutine() {
  if (printRevSec) {
    Serial.print("\trev/sec:\t");
    Serial.println(rev_sec);
  }else if (printRevMin) {
    Serial.print("\trev/min:\t");
    Serial.println(rev_min);
  }else if (printRadSec) {
    Serial.print("\trad/sec:\t");
    Serial.println(rad_sec);
  }
  
  if (Serial.available()>0){
    char t = Serial.read();

    if (t == 'a' && !working){
      working = true;
      Serial.print("Ya");
      digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);              // wait for a second
      digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);  
      working = false; 
    }
    else if (t == 'm' && !working){
      working = true;
      Serial.println("Switching to rev/min");
      printRevSec = false;
      printRevMin = true;
      printRadSec = false;
      working = false; 
    }
    else if (t == 'r' && !working){
      working = true;
      Serial.println("Switching to rad/sec");
      printRevSec = false;
      printRevMin = false;
      printRadSec = true;
      working = false; 
    }
    else if (t == 's' && !working){
      working = true;
      Serial.println("Switching to rev/sec");
      printRevSec = true;
      printRevMin = false;
      printRadSec = false;
      working = false; 
    }
  }           
}

