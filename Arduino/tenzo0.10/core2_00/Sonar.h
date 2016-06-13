#ifndef SONAR_H
#define SONAR_H

class Sonar
{
  public:
    Sonar();
    void init();
    int getDistance();
    void printAltitude();
    
    int distance;
    
};

#endif
