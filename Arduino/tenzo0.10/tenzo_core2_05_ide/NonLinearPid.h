/*
  NonLinearPid.h - Library implementing a model free approach based on 
  pid control. 
  Created by Giovanni Balestrieri - UserK, August 28, 2015.
*/
#ifndef NonLinearPid_h
  #define NonLinearPid_h

#include "Propulsion.h"
#include "Arduino.h"

class NonLinearPid
{
  private:
    volatile float  kp;
    volatile float  ki;
    volatile float  kd;
    volatile float  proporzionale;
    volatile float  integrale;
    volatile float  derivativo;
    volatile float  error_old;
    volatile float  outMax;
    volatile float  outMin;
    volatile bool   on;
 
  public:
    NonLinearPid(volatile float kp,volatile float ki,volatile float kd);
    int Compute(volatile float riferimento,volatile float out);
    void SetTunings(volatile float kp,volatile float ki,volatile float kd);
    void SetOutputLimits(volatile float Min,volatile float Max);
    void change_kp(volatile float kp);
    void change_kd(volatile float kd);
    void change_ki(volatile float ki);
    float get_prop();
    float get_deriv();
    float get_integ();
    float get_error();
    void reset();
    void pause();
    void restart();
};

#endif
