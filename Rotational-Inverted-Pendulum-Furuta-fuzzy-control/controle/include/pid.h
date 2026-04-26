#pragma once
#include <Arduino.h>

struct PIDController {
  float kp;
  float ki;
  float kd;

  float integrator;
  float previousError;
};

struct PIDTerms {
  float p;
  float i;
  float d;
  float output;
};

void pid_reset(PIDController& pid);

PIDTerms pid_update(PIDController& pid,
                    float setpoint,
                    float measurement,
                    float dt,
                    float integralBand);
