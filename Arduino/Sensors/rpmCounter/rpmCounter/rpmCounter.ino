boolean working = false;
int analValue = 0, analogPin = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
  Serial.println("Yo");
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  serialRoutine();
 readAnalogValue();
}

void readAnalogValue(){
  
  analValue = analogRead(analogPin);
}

void serialRoutine() {
  Serial.print("\nValue:\t");
  Serial.println(analValue);
  
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

