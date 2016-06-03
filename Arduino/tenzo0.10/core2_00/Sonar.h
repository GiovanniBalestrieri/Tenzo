#ifndef SONAR_H
#define SONAR_H

//extern int LOW;
//extern int HIGH;

#define trigPin 7 
#define echoPin 8


class Sonar
{
  public:
    Sonar(int,int);
    float getDistance();
    unsigned long getDuration();
    void printAltitude();

    unsigned long duration;
  private:
    float _k1;
    unsigned long _threshold;
    int _trigger;
    int _echo;
    float _distance;
};
/*

Sonar::Sonar(int trigger, int echo)
{
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  Serial.println("[ OK ] Sonar");
  
  _echo = echo;
  _trigger = trigger;
  _k1 = 0.034;
  _threshold = 38000;
  duration = 0;  
}

float Sonar::getDistance()
{
  return _k1 * this->getDuration() / 2;
}


unsigned long Sonar::getDuration()
{
  digitalWrite(_trigger, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(_trigger, HIGH);
  delay(1); // Added this line
  digitalWrite(_trigger, LOW);
  duration = pulseIn(_echo, HIGH);
  return duration;
}

*/
#endif
