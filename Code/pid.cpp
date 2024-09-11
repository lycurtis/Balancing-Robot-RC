#include "pid.h"

//PID scalar constants
float Kp = 1.0;
float Ki = 0.1;
float Kd = 0.01;

const float setPoint = 0.0; //Desired angle (upright default)

float error, proportional, integral, derivative, prevError;

float calculate_PID(float currAngle){
  error = currAngle - setPoint; //or setPoint - currAngle (whichever gives you the correct interpreation of positive/negative)

  proportional = error*Kp;

  integral = integral + (error*Ki);
  //Generally a good idea to limit the integral to some amount so it doesn't keep accumulating too much error which can cause problems
  if(integral > 400){
    integral = 400;
  }
  else if(integral < -400){
    integral = -400;
  }

  derivative = (error - prevError)*Kd;

  prevError = error;

  return proportional + integral + derivative;
}
