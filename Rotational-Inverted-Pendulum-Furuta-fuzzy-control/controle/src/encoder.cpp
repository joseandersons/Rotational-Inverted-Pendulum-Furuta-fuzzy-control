#include "encoder.h"
#include "config.h"

static volatile int32_t enc_count_pendulo = 0;
static volatile int32_t enc_count_motor   = 0;
static portMUX_TYPE mux_pendulo = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE mux_motor   = portMUX_INITIALIZER_UNLOCKED;

static SemaphoreHandle_t mtx = nullptr;
static Obs g_obs{0,0,0};

IRAM_ATTR static void enc_pendulo_isr(){
  uint8_t a = digitalRead(ENCA_PENDULO);
  uint8_t b = digitalRead(ENCB_PENDULO);
  
  if(a == b) {
    portENTER_CRITICAL_ISR(&mux_pendulo);
    enc_count_pendulo++;
    portEXIT_CRITICAL_ISR(&mux_pendulo);
  } else {
    portENTER_CRITICAL_ISR(&mux_pendulo);
    enc_count_pendulo--;
    portEXIT_CRITICAL_ISR(&mux_pendulo);
  }
}

IRAM_ATTR static void enc_motor_isr(){
  uint8_t a = digitalRead(ENCA_MOTOR);
  uint8_t b = digitalRead(ENCB_MOTOR);
  
  if(a == b) {
    portENTER_CRITICAL_ISR(&mux_motor);
    enc_count_motor++;
    portEXIT_CRITICAL_ISR(&mux_motor);
  } else {
    portENTER_CRITICAL_ISR(&mux_motor);
    enc_count_motor--;
    portEXIT_CRITICAL_ISR(&mux_motor);
  }
}

void encoder_pendulo_isr_init(){
  pinMode(ENCA_PENDULO, INPUT);
  pinMode(ENCB_PENDULO, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_PENDULO), enc_pendulo_isr, CHANGE);
}

void encoder_motor_isr_init(){
  pinMode(ENCA_MOTOR, INPUT);
  pinMode(ENCB_MOTOR, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA_MOTOR), enc_motor_isr, CHANGE);
}


int32_t encoder_pendulo_get_count_isr_safe(){
  portENTER_CRITICAL(&mux_pendulo);
  int32_t v = enc_count_pendulo;
  portEXIT_CRITICAL(&mux_pendulo);
  return v;
}

int32_t encoder_motor_get_count_isr_safe(){
  portENTER_CRITICAL(&mux_motor);
  int32_t v = enc_count_motor;
  portEXIT_CRITICAL(&mux_motor);
  return v;
}


bool encoder_get_obs(Obs* out){
  if (xSemaphoreTake(mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
    *out = g_obs;
    xSemaphoreGive(mtx);
    return true;
  }
  return false;
}

static void encoder_task(void*){
  TickType_t ultima_vez_task = xTaskGetTickCount();

  for(;;){
    vTaskDelayUntil(&ultima_vez_task, pdMS_TO_TICKS(PERIOD_ENCODER_MS));

    Obs o;
    o.theta_counts = encoder_pendulo_get_count_isr_safe();
    o.omega_counts = encoder_motor_get_count_isr_safe();
    o.stamp_ms     = millis();


    if (xSemaphoreTake(mtx, 0) == pdTRUE) {
      g_obs = o;
      xSemaphoreGive(mtx);
    }
  }
}

void encoder_reset_motor_count(){
  portENTER_CRITICAL(&mux_motor);
  enc_count_motor = 0;
  portEXIT_CRITICAL(&mux_motor);
}

void encoder_reset_pendulo_count(){
  portENTER_CRITICAL(&mux_pendulo);
  enc_count_pendulo = 0;
  portEXIT_CRITICAL(&mux_pendulo);
}


void encoder_task_start(){
  mtx = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(encoder_task, "thENCODER", 3072, nullptr, PRIO_ENCODER, nullptr, CORE_CTRL);
}
