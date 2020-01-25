#include <DualMC33926MotorShield.h>

DualMC33926MotorShield md;
float alpha = 0.20;
int referencePin = 0, referenceSignal, referenceSignalFilt = 0;
float STEP = 0.0048;
float MIN_REF_VOL = 0.077;
float MAX_REF_VOL = 4.2;
float ref_last = 0.0f;

//pinout
const int btn_more = 13;
const int btn_less = 12;

int buttonMoreState;             // the current reading from the input pin
int lastButtonMoreState = LOW;   // the previous reading from the input pin

unsigned long lastDebounceMoreTime = 0;  // the last time the output pin was toggled
unsigned long debounceMoreDelay = 50;    // the debounce time; increase if the output flickers

int buttonLessState;             // the current reading from the input pin
int lastButtonLessState = LOW;   // the previous reading from the input pin

unsigned long lastDebounceLessTime = 0;  // the last time the output pin was toggled
unsigned long debounceLessDelay = 50;    // the debounce time; increase if the output flickers
void setup() {
  // Sets the two pins as Outputs
  pinMode(btn_more, INPUT);
  pinMode(btn_less, INPUT);
  md.init();
  Serial.begin(115200);          //  setup serial
}
void loop() {
  //pressione bottone più
  int readingMore = digitalRead(btn_more);

  referenceSignal = analogRead(0);
  
  if (readingMore != lastButtonMoreState) {
    lastDebounceMoreTime = millis();
  }
  if ((millis() - lastDebounceMoreTime) > debounceMoreDelay) {
    if (readingMore != buttonMoreState) {
      buttonMoreState = readingMore;
      if (buttonMoreState == HIGH) {
        md.setM1Speed(400);
        delay(50);
        md.setM1Speed(0);
        Serial.print("+ gas");
      }
    }
  }

  Serial.print("Reference: ");
  Serial.println(referenceSignal);

  filterVal(referenceSignal);
  
  Serial.print("Reference: ");
  Serial.println(referenceSignal);
  
  //referenceSignal = map(referenceSignal, MIN_REF_VOL/STEP, MAX_REF_VOL/STEP, 0, 100);

  /*
  Serial.print("\tReference (%): ");
  Serial.println(referenceSignal);

  */

  lastButtonMoreState = readingMore;

  //pressione bottone meno
  int readingLess = digitalRead(btn_less);
  if (readingLess != lastButtonLessState) {
    lastDebounceLessTime = millis();
  }
  if ((millis() - lastDebounceLessTime) > debounceLessDelay) {
    if (readingLess != buttonLessState) {
      buttonLessState = readingLess;
      if (buttonLessState == HIGH) {
        md.setM1Speed(-400);
        delay(50);
        md.setM1Speed(0);
        Serial.print("- gas");
      }
    }
  }

  lastButtonLessState = readingLess;


}
//  tastiera = Serial.read();
//  Serial.flush();
//  while (tastiera == '+') {
//    md.setM1Speed(400);
//    delay(50);
//    md.setM1Speed(0);
//    Serial.print("+ gas");
//    tastiera = 0;
//  }
//  while (tastiera == '-') {
//    md.setM1Speed(-400);
//    delay(50);
//    md.setM1Speed(0);
//    Serial.print("- gas");
//    tastiera = 0;
//  }
//}


void filterVal(float current) {
  referenceSignalFilt = alpha * ref_last + (1 - alpha ) * current;
  ref_last = referenceSignalFilt;  
}
