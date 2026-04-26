#pragma once
#include <Arduino.h>
#include "tipos.h"

// Encoder do pêndulo
void    encoder_pendulo_isr_init();
int32_t encoder_pendulo_get_count_isr_safe();

// Encoder do motor
void    encoder_motor_isr_init();
int32_t encoder_motor_get_count_isr_safe();

void encoder_reset_pendulo_count();
void encoder_reset_motor_count();

// Task de aquisição do pêndulo
void encoder_task_start();
bool encoder_get_obs(Obs* out);
