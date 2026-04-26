#include <Arduino.h>
#include "control.h"
#include "config.h"
#include "encoder.h"
#include "tipos.h"
#include "filas.h"
#include "ang.h"
#include "params.h"
#include "calibration.h"
#include "pid.h"
#include "arm_centering_fuzzy.h"

static PIDController pidPendulo{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

static float g_setpoint_deg          = SETPOINT_DEG;
static uint32_t prev_ms              = 0;

static volatile bool g_control_enabled = false;
static volatile bool g_start_requested = false;

static float compute_dt(uint32_t stamp_ms) {
  if (prev_ms == 0) {
    prev_ms = stamp_ms;
    return 1e-3f;
  }
  uint32_t diff_ms = stamp_ms - prev_ms;
  prev_ms = stamp_ms;
  float dt = diff_ms * 1e-3f;
  if (dt <= 0.0f) dt = 1e-3f;
  return dt;
}

static void reset_state_on_start() {
  pid_reset(pidPendulo);
  encoder_reset_motor_count();
  encoder_reset_pendulo_count();
  prev_ms                 = 0;
}

void control_request_start() {
  g_start_requested = true;
}

void control_stop() {
  g_control_enabled = false;
}

static int16_t duty_from_u(float u) {
  float au = fabsf(u);
  if (au < 1e-3f) return 0.0f;
  if (au > U_SAT) au = U_SAT;

  float mag = au / U_SAT;

  if (u > 0.0f) {
    int16_t duty = (int16_t)lroundf(DUTY_DEAD_FWD + mag * (DUTY_MAX - DUTY_DEAD_FWD));
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    return duty;
  } else {
    int16_t duty = (int16_t)lroundf(DUTY_DEAD_REV + mag * (DUTY_MAX - DUTY_DEAD_REV));
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    return -duty;
  }
}

static void control_step(const Obs& o) {
  float dt = compute_dt(o.stamp_ms);
  int32_t zero_offset_counts = get_zero_offset();

  int32_t theta_counts = wrap_counts_pendulo((int32_t)o.theta_counts - zero_offset_counts);
  int32_t omega_counts = wrap_counts_motor((int32_t)o.omega_counts);

  const float contagens_por_volta_pendulo = (CPR_PENDULO * GEAR_PENDULO);
  const float contagens_por_volta_motor   = (CPR_MOTOR   * GEAR_MOTOR);

  float angulo_pendulo_deg = (float)theta_counts * 360.0f / contagens_por_volta_pendulo;
  float angulo_motor_deg   = (float)omega_counts * 360.0f / contagens_por_volta_motor;

  static float arm_prev_deg = 0.0f;
  float arm_angle_deg = angulo_motor_deg;

  float delta_arm = arm_angle_deg - arm_prev_deg;
  if (delta_arm > 180.0f)  delta_arm -= 360.0f;
  if (delta_arm < -180.0f) delta_arm += 360.0f;

  float arm_speed_deg_s = delta_arm / dt;
  arm_prev_deg = arm_angle_deg;

  float base_setpoint_deg = g_setpoint_deg;

  float sp_offset_deg = arm_centering_fuzzy_sp_offset_deg(
    arm_angle_deg,
    arm_speed_deg_s
  );
  float setpoint_deg = base_setpoint_deg + sp_offset_deg;
  float erro = setpoint_deg - angulo_pendulo_deg;

  PIDGainsPendulo ganhos = params_pendulo_get();
  pidPendulo.kp = ganhos.kp;
  pidPendulo.ki = ganhos.ki;
  pidPendulo.kd = ganhos.kd;

  PIDTerms pidOut = pid_update(
    pidPendulo,
    setpoint_deg,
    angulo_pendulo_deg,
    dt,
    I_BAND_DEG
  );

  float u = pidOut.output;

  if (u >  U_SAT) u =  U_SAT;
  if (u < -U_SAT) u = -U_SAT;

  int16_t duty_signed = duty_from_u(u);

  if (fabsf(erro) < 0.8f && fabsf(u) < (0.10f * U_SAT)) {
    duty_signed = 0;
  }

  bool kick = (duty_signed != 0);

  MotorCmd cmd{ duty_signed };
  xQueueSend(qMotor, &cmd, 0);

  static uint32_t last_tx = 0;
  if (millis() - last_tx >= 20) {
    last_tx = millis();

    TelePkt pkt{};
    pkt.t_ms   = o.stamp_ms;
    pkt.th_c2  = (int16_t)lroundf(angulo_pendulo_deg * 100.0f);
    pkt.thm_c2 = (int16_t)lroundf(angulo_motor_deg   * 100.0f);
    pkt.sp_c2  = (int16_t)lroundf(setpoint_deg       * 100.0f);
    pkt.e_c2   = (int16_t)lroundf(erro               * 100.0f);
    pkt.duty   = duty_signed;

    xQueueSend(qTel, &pkt, 0);
  }
}

void control_task_start() {
  xTaskCreatePinnedToCore([](void*){
    Serial.println("[control] start");

    TickType_t last = xTaskGetTickCount();

    for (;;) {
      if (!g_control_enabled) {
        if (g_start_requested) {
          Serial.println("[control] iniciando calibração");
          reset_state_on_start();
          calibrar_zero_por_fundo();
          Serial.println("[control] zero ok, controle habilitado");
          g_control_enabled = true;
          g_start_requested = false;
          last = xTaskGetTickCount();
        } else {
          vTaskDelay(pdMS_TO_TICKS(10));
        }
        continue;
      }

      vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_CONTROL_MS));

      Obs o{};
      encoder_get_obs(&o);
      control_step(o);
    }
  }, "thCONTROL", 4096, nullptr, PRIO_CONTROL, nullptr, CORE_CTRL);
}
