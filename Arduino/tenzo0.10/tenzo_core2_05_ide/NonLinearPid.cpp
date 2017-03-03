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
  
  outMin = -700;
  outMax = 700;
  
  on = true;
}

int NonLinearPid::Compute(volatile float riferimento,volatile float out)
{
  if (!on)   
    return 0;
  else
  {
    volatile int u = 0;
    volatile float error = riferimento - out ;
    proporzionale = kp*error;
    integrale = integrale + ki*error;
    derivativo = kd*(error-error_old);
    error_old = error;
 
    if(integrale > outMax) 
      integrale= outMax;
    else if(integrale < outMin) 
      integrale= outMin;
    
    u = int(proporzionale + integrale + derivativo);
    // Security Check
    if(u > outMax) 
      u = outMax;
    else if(u < outMin) 
      u = outMin;
      
    return u;
  }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void NonLinearPid::SetOutputLimits(float Min, float Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void NonLinearPid::SetTunings(volatile float kp_,volatile float ki_,volatile float kd_)
{
   if (kp_<0 || ki_<0 || kd_<0) return;
 
   kp = kp_;
   ki = ki_;
   kd = kd_;
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
