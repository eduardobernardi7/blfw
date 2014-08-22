
//pid.c
// Source file for PID control module

#include "pid.h"

// PID Initialization
void PID_Init(PID_T *me, float Kp, float Ki, float Kd)
{
  me->Kp = Kp;
  me->Ki = Ki;
  me->Kd = Kd;
  me->integral = 0;
  me->last_error = 0;
  me->ref_value = 0;   
  me->lock_count = PID_NOT_LOCKED;
  me->locked = 0;
}

// Sets the reference value
void PID_SetRefValue(PID_T *me, int32_t ref_value)
{
  me->ref_value = ref_value;
}

// Testing Lock Status
int PID_LockStatus(PID_T *me)
{
  switch(me->lock_count)
  {
  case PID_UNLOCK:
    if (me->locked == 1) me->locked = 0;
    break;
  case PID_LOCK:
    if (me->locked == 0) me->locked = 1;
    break;
  default:
    break;
  }
  return me->locked;
}


// Process PID output
// Todo: anti-windup for the integral term (needs experimentation first)
int32_t PID_Process(PID_T *me, int32_t measured_value)
{
  int32_t error, derivative, integral, control_sig;
  
  error = me->ref_value - measured_value;
  
  if (error > -PID_ERROR_TOL && error < PID_ERROR_TOL)
    error = 0;
  
  // If error is zero and not fully locked, increases lock count
  if(error == 0 && me->lock_count < PID_FULLY_LOCKED) me->lock_count++;
  // If error is not zero, decrements lock_count until PID_NOT_LOCKED
  else if (error != 0 && me->lock_count > PID_NOT_LOCKED) me->lock_count--;
  
  integral = me->integral + error;
  
  // Anti Windup
  if(integral > PID_MAX_INTEGRAL) integral = PID_MAX_INTEGRAL;
  if(integral < - PID_MAX_INTEGRAL) integral = -PID_MAX_INTEGRAL;
  
  me->integral = integral;
  
  derivative = error - me->last_error;
  me->last_error = error;
  
  control_sig = (int32_t) (me->Kp * error + me->Ki * integral + me->Kd*derivative);
  
  return control_sig;       
}
