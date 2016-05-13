/*
  NonLinearPid.h - Library implementing a model free approach based on 
  pid control. 
  Created by Giovanni Balestrieri - UserK, August 28, 2015.
*/
#include "NonLinearPid.h"

NonLinearPid::NonLinearPid(volatile float kp_,volatile float ki_,volatile float kd_)
{
  kp = kp_;
  ki = ki_;
  kd = kd_;
  integrale = 0.0;
  error_old = 0.0;
  on = true;
}

int NonLinearPid::compute(volatile float riferimento,volatile float out)
{
  volatile int u = 0;
  volatile float error = riferimento - out ;
  proporzionale = kp*error;
  integrale = integrale + ki*error;
  derivativo = kd*(error-error_old);
  u = int(proporzionale + integrale + derivativo);
  error_old = error;
  return u;//(on? u:0);
}

float NonLinearPid::get_error()
{
  return error_old;
}

void NonLinearPid::pause()
{
  reset();
  on = false;
}

void NonLinearPid::restart()
{
  reset();
  on = true;
}

void NonLinearPid::reset()
{
  integrale = 0.0;
  error_old = 0.0;
}

void NonLinearPid::change_kp(volatile float k_prop)
{
  kp = k_prop;
  return;
}

void NonLinearPid::change_kd(volatile float k_der)
{
  kd = k_der;
  return;
}

void NonLinearPid::change_ki(volatile float k_int)
{
  ki = k_int;
  return;
}

float NonLinearPid::get_prop()
{
  return proporzionale;
}

float NonLinearPid::get_integ()
{
  return integrale;
}

float NonLinearPid::get_deriv()
{
  return derivativo;
}
