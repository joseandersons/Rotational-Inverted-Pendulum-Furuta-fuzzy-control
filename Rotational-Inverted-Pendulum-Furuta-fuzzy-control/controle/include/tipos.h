#pragma once
#include <Arduino.h>

struct Obs {
  float theta_counts;
  float omega_counts;
  float motor_counts;
  uint32_t stamp_ms;
};

struct MotorCmd {
  int16_t duty_signed;
};

typedef struct {
  uint32_t t_ms; // Timestamp in milliseconds
  int16_t th_c2; // Pendulum angle in centi-degrees
  int16_t thm_c2; // Motor angular velocity in centi-degrees
  int16_t sp_c2; // Setpoint in centi-degrees
  int16_t e_c2; // Error in centi-degrees
  int16_t duty; // Motor duty cycle
} TelePkt;
