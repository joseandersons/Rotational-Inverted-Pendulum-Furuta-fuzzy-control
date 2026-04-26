#include "motor.h"
#include "config.h"
#include "filas.h"
#include "tipos.h"

static void motor_write_signed(int16_t dutySigned) {
  if (dutySigned == 0) { ledcWrite(PWM_CH, 0); return; }
  if (dutySigned > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  uint16_t mag = abs(dutySigned);
  if (mag > 0 && mag < DEADBAND_DUTY) mag = DEADBAND_DUTY;
  if (dutySigned > 0 && mag > 0 && mag < DUTY_DEAD_FWD) mag = DUTY_DEAD_FWD;
  if (dutySigned < 0 && mag > 0 && mag < DUTY_DEAD_REV) mag = DUTY_DEAD_REV;
  if (mag > DUTY_MAX) mag = DUTY_MAX;
  ledcWrite(PWM_CH, mag);
}
// Ver se isso ta afetando o controle, porque  (É o tempo máximo que a task do motor vai ficar parada, esperando vir um comando.)
static void motor_task(void*) { 
  MotorCmd cmd{};
  for (;;) {
    if (xQueueReceive(qMotor, &cmd, pdMS_TO_TICKS(WATCHDOG_MS)) == pdTRUE) {
      motor_write_signed(cmd.duty_signed);
    } else {
      ledcWrite(PWM_CH, 0);
    }
  }
}

void motor_init() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CH);
  ledcWrite(PWM_CH, 0);
}

void motor_task_start() {
  xTaskCreatePinnedToCore(motor_task, "thMOTOR", 3072, nullptr, PRIO_MOTOR, nullptr, CORE_IO);
}
