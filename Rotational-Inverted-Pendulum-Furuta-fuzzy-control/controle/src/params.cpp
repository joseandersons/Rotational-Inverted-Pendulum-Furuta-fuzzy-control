#include "params.h"
#include "config.h"
#include <Arduino.h>

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static volatile float g_p_kp;
static volatile float g_p_ki;
static volatile float g_p_kd;

static volatile float g_m_kp;
static volatile float g_m_ki;
static volatile float g_m_kd;

static volatile float g_fz_inicio_acao_deg;
static volatile float g_fz_acao_maxima_deg;
static volatile float g_fz_offset_max_setpoint_deg;
static volatile float g_fz_limiar_vel_alta_deg_s;

void params_begin() {
  portENTER_CRITICAL(&mux);

  g_p_kp = KP_PENDULO;
  g_p_ki = KI_PENDULO;
  g_p_kd = KD_PENDULO;

  g_m_kp = KP_MOTOR;
  g_m_ki = KI_MOTOR;
  g_m_kd = KD_MOTOR;

  g_fz_inicio_acao_deg = 20.0f;
  g_fz_acao_maxima_deg = 30.0f;
  g_fz_offset_max_setpoint_deg = 0.5f;
  g_fz_limiar_vel_alta_deg_s = 500.0f;

  portEXIT_CRITICAL(&mux);
}

void params_pendulo_set(float kp, float ki, float kd) {
  portENTER_CRITICAL(&mux);
  g_p_kp = kp;
  g_p_ki = ki;
  g_p_kd = kd;
  portEXIT_CRITICAL(&mux);
}

void params_motor_set(float kp, float ki, float kd) {
  portENTER_CRITICAL(&mux);
  g_m_kp = kp;
  g_m_ki = ki;
  g_m_kd = kd;
  portEXIT_CRITICAL(&mux);
}

void params_arm_fuzzy_set(float inicio_acao_deg,
                          float acao_maxima_deg,
                          float offset_max_setpoint_deg,
                          float limiar_vel_alta_deg_s) {
  if (inicio_acao_deg < 0.0f) inicio_acao_deg = 0.0f;
  if (acao_maxima_deg < inicio_acao_deg) acao_maxima_deg = inicio_acao_deg;
  if (offset_max_setpoint_deg < 0.0f) offset_max_setpoint_deg = 0.0f;
  if (limiar_vel_alta_deg_s < 0.0f) limiar_vel_alta_deg_s = 0.0f;

  portENTER_CRITICAL(&mux);
  g_fz_inicio_acao_deg = inicio_acao_deg;
  g_fz_acao_maxima_deg = acao_maxima_deg;
  g_fz_offset_max_setpoint_deg = offset_max_setpoint_deg;
  g_fz_limiar_vel_alta_deg_s = limiar_vel_alta_deg_s;
  portEXIT_CRITICAL(&mux);
}

PIDGainsMotor params_motor_get() {
  PIDGainsMotor r;
  portENTER_CRITICAL(&mux);
  r.kp = g_m_kp;
  r.ki = g_m_ki;
  r.kd = g_m_kd;
  portEXIT_CRITICAL(&mux);
  return r;
}

PIDGainsPendulo params_pendulo_get() {
  PIDGainsPendulo r;
  portENTER_CRITICAL(&mux);
  r.kp = g_p_kp;
  r.ki = g_p_ki;
  r.kd = g_p_kd;
  portEXIT_CRITICAL(&mux);
  return r;
}

ArmFuzzyParams params_arm_fuzzy_get() {
  ArmFuzzyParams r;
  portENTER_CRITICAL(&mux);
  r.inicio_acao_deg = g_fz_inicio_acao_deg;
  r.acao_maxima_deg = g_fz_acao_maxima_deg;
  r.offset_max_setpoint_deg = g_fz_offset_max_setpoint_deg;
  r.limiar_vel_alta_deg_s = g_fz_limiar_vel_alta_deg_s;
  portEXIT_CRITICAL(&mux);
  return r;
}