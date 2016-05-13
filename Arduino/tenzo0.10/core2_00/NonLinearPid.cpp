/*
  NonLinearPid.h - Library implementing a model free approach based on 
  pid control. 
  Created by Giovanni Balestrieri - UserK, August 28, 2015.
*/
#include "NonLinearPid.h"

Pid::Pid(float kp_,float ki_,float kd_)
{
  kp = kp_;
  ki = ki_;
  kd = kd_;
  integrale = 0.0;
  error_old = 0.0;
  on = true;
}

int Pid::get_u(float riferimento, float out)
{
  int u = 0;
  float error = riferimento - out ;
  proporzionale = kp*error;
  integrale = integrale + ki*error;
  derivativo = kd*(error-error_old);
  u = int(proporzionale + integrale + derivativo);
  error_old = error;
  return u;//(on? u:0);
}

float Pid::get_error()
{
  return error_old;
}

void Pid::pause()
{
  reset();
  on = false;
}

void Pid::restart()
{
  reset();
  on = true;
}

void Pid::reset()
{
  integrale = 0.0;
  error_old = 0.0;
}

void Pid::change_kp(float k_prop)
{
  kp = k_prop;
  return;
}

void Pid::change_kd(float k_der)
{
  kd = k_der;
  return;
}

void Pid::change_ki(float k_int)
{
  ki = k_int;
  return;
}

float Pid::get_prop()
{
  return proporzionale;
}

float Pid::get_integ()
{
  return integrale;
}

float Pid::get_deriv()
{
  return derivativo;
}
