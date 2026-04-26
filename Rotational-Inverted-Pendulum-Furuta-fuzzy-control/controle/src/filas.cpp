#include "filas.h"

QueueHandle_t qMotor = nullptr;
QueueHandle_t qTel   = nullptr;

void filas_init(){
  qMotor = xQueueCreate(16, sizeof(MotorCmd));
  qTel   = xQueueCreate(32, sizeof(TelePkt));   
}
