#include "pid.h"

void pid_reset(PIDController& pid){
  pid.integrator   = 0.0f;
  pid.previousError = 0.0f;
}

PIDTerms pid_update(PIDController& pid,
                    float setpoint,
                    float measurement,
                    float dt,
                    float integralBand)
{
  PIDTerms terms{};

  float error = setpoint - measurement;

  terms.p = pid.kp * error;

  if (fabsf(error) < integralBand) {
    pid.integrator += error * dt;
  } else {
    pid.integrator = 0.0f;
  }
  terms.i = pid.integrator * pid.ki;

  float derivative = (error - pid.previousError) / dt;
  terms.d = pid.kd * derivative;

  pid.previousError = error;

  terms.output = terms.p + terms.i + terms.d;
  return terms;
}
