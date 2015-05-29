
//Analog read pins
const int xPin = 0;
const int yPin = 1;
const int zPin = 2;

int xRead,yRead,zRead;

void setup()
{
  Serial.begin(9600);
  Serial.println("Ready!");
}

void loop()
{
  readAcc();
  printVals();

  delay(500);
}
  
// Read analog values from the Acc
void readAcc()
{
  xRead = analogRead(xPin);
  yRead = analogRead(yPin);
  zRead = analogRead(zPin); 
}

void printVals()
{
    Serial.print("      X: ");
    Serial.print(xRead);
    Serial.print(" | Y: ");
    Serial.print(yRead);
    Serial.print(" | Z: ");
    Serial.print(zRead);
    Serial.println();
}
