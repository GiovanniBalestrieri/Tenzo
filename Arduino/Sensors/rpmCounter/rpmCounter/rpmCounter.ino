#define SECOND_US 1000000

int rev_sec = 0;
int rev_min = 0;
int rad_sec = 0;
boolean working = false;
const byte interruptPin = 2;
volatile byte state = LOW;
volatile long counter = 0;
unsigned long timeTracker = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  Serial.println("Yo");
  
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), count, FALLING);
  pinMode(13, OUTPUT);
}

void loop() {
  serialRoutine();
  computeRevolutions();
}

void computeRevolutions() {
  if (micros() - timeTracker>= SECOND_US) {
    rev_sec = counter;
    counter = 0;  
  }  
}

void count() { // ISR
  counter++;
}

void serialRoutine() {
  Serial.print("\nCount:\t");
  Serial.println(counter);
  
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
  }           // wait for a second
}

