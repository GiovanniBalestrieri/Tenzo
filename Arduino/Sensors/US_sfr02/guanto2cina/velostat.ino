
void readVelostat() {

  pushValue = analogRead(pushPin);
  //Serial.println(pushValue);

  if (pushValue < pushTreshold) {
    statoPush = 1;
    delay(50);
  }
}
