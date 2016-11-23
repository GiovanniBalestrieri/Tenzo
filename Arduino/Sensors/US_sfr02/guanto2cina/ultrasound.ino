
void usRoutineCina()
{
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(20); // Added this line
  digitalWrite(trigPin, HIGH);
  delay(2); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  reading = 0.034 * duration / 2;
  Serial.print( "durata: " );
  Serial.print( duration );
  Serial.print( " , " );
  Serial.print( "distanza: " );
   
  //dopo 38ms è fuori dalla portata del sensore
  if( duration > 38000 ) 
    Serial.println( "fuori portata");
  else { Serial.print( reading ); 
    Serial.println( "cm" );}
   
  //aspetta 1.5 secondi
  delay( 20 );

  if (FILTERING)
    filter();

  //Serial.print(sonarTimer);   // print the reading
}



void usRoutine()
{
  Wire.beginTransmission(112);
  Wire.write(byte(0x00));
  Wire.write(byte(0x51));

  Wire.endTransmission();
  delay(15);

  Wire.beginTransmission(112);
  Wire.write(byte(0x02));
  Wire.endTransmission();
  sonarTimer = micros();
  Wire.requestFrom(112, 2);

  if (2 <= Wire.available()) {
    reading = Wire.read();
    reading = reading << 8;
    reading |= Wire.read();
  }

  if (FILTERING)
    filter();

  contSonarRoutine++;
  sonarTimer = micros() - sonarTimer;
  if (maxsonarTimer <= sonarTimer)
    maxsonarTimer = sonarTimer;
  sonarTimeTot = sonarTimeTot + sonarTimer;
  //Serial.print(sonarTimer);   // print the reading
}
