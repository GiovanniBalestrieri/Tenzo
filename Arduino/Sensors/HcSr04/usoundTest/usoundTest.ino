
#define trigPin 7 
#define echoPin 8

long duration, distance;

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200); 
pinMode( trigPin, OUTPUT );
pinMode( echoPin, INPUT );
}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(20); // Added this line
  digitalWrite(trigPin, HIGH);
  delay(2); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  long r = 0.034 * duration / 2;
  Serial.print( "durata: " );
Serial.print( duration );
Serial.print( " , " );
Serial.print( "distanza: " );
 
//dopo 38ms è fuori dalla portata del sensore
if( duration > 38000 ) 
  Serial.println( "fuori portata");
else { Serial.print( r ); 
  Serial.println( "cm" );}
 
//aspetta 1.5 secondi
delay( 20 );

}
