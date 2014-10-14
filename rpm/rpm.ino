int ledPin = 13;                // IR LED connected to digital pin 13
volatile byte rpmcount;
unsigned int rpm;
unsigned long timeold;

void rpm_fun()
 {
  //Update count
  rpmcount++;

 }

void setup()
 {
   Serial.begin(9600);
   //Interrupt 0 is digital pin 2, so that is where the IR detector is connected
   //Triggers on FALLING (change from HIGH to LOW)
   attachInterrupt(0, rpm_fun, RISING);

   //Turn on IR LED
   pinMode(ledPin, OUTPUT);
   digitalWrite(ledPin, HIGH);

   rpmcount = 0;
   rpm = 0;
   timeold = 0;
 }

 void loop()
 {
   //Update RPM every second
   delay(1000);
   //Don't process interrupts during calculations
   //detachInterrupt(0);
    
//   rpm = 30*1000/(millis() - timeold)*rpmcount;
//   timeold = millis();
   //rpmcount = 0;
//   Serial.println("RPM: ");
   Serial.println(rpmcount);
   //Restart the interrupt processing
   //attachInterrupt(0, rpm_fun, CHANGE);
}
