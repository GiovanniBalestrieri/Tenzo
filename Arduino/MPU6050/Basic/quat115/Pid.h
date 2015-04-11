#ifndef Pid_h
#define Pid_h
#include "Arduino.h"

class Pid
{
  private:
    volatile float  kp;
    volatile float  ki;
    volatile float  kd;
    volatile float  proporzionale;
    volatile float  integrale;
    volatile float  derivativo;
    volatile float  error_old;
    volatile bool   on;
 
  public:
    Pid(float kp,float ki,float kd);
    int get_u(float riferimento, float out);
    void change_kp(float kp);
    void change_kd(float kd);
    void change_ki(float ki);
    float get_prop();
    float get_deriv();
    float get_integ();
    float get_error();
    void reset();
    void pause();
    void restart();
};

#endif
