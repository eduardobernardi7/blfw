
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
}

// Sets the reference value
void PID_SetRefValue(PID_T *me, int32_t ref_value)
{
  me->ref_value = ref_value;
}

// Process PID output
// Todo: anti-windup for the integral term (needs experimentation first)
int32_t PID_Process(PID_T *me, int32_t measured_value)
{
  int32_t error, derivative, integral, control_sig;
  
  error = me->ref_value - measured_value;
  
  if (error > -PID_ERROR_TOL && error < PID_ERROR_TOL)
    error = 0;
  
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
