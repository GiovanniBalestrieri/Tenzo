#ifndef SONAR_H
#define SONAR_H

extern volatile int contSonarRoutine;
extern volatile unsigned long sonarTimer;
extern volatile unsigned long maxsonarTimer;
extern volatile unsigned long sonarTimeTot;


class Sonar
{
  public:
    Sonar();
    float getDistance();
    void printAltitude();
    
    const int cmReg = 0x51; 
    const int deviceAdd = 112;

    int distance;
    
};

#endif
