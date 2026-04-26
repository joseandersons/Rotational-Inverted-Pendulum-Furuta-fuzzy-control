#pragma once

struct PIDGainsPendulo {
  float kp;
  float ki;
  float kd;
};

struct PIDGainsMotor {
  float kp;
  float ki;
  float kd;
};

struct ArmFuzzyParams {
  float inicio_acao_deg;
  float acao_maxima_deg;
  float offset_max_setpoint_deg;
  float limiar_vel_alta_deg_s;
};

void params_begin();

void params_pendulo_set(float kp, float ki, float kd);
void params_motor_set(float kp, float ki, float kd);
void params_arm_fuzzy_set(float inicio_acao_deg,
                          float acao_maxima_deg,
                          float offset_max_setpoint_deg,
                          float limiar_vel_alta_deg_s);

PIDGainsMotor params_motor_get();
PIDGainsPendulo params_pendulo_get();
ArmFuzzyParams params_arm_fuzzy_get();