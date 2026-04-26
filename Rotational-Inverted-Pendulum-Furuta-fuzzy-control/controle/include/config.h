#pragma once
#include <Arduino.h>

#define CORE_CTRL 0
#define CORE_IO   1

#define PRIO_CONTROL     (configMAX_PRIORITIES-2)
#define PRIO_ENCODER     (configMAX_PRIORITIES-3)
#define PRIO_MOTOR       (configMAX_PRIORITIES-3)

#define PERIOD_CONTROL_MS 2
#define PERIOD_ENCODER_MS 2

#define IN1 19
#define IN2 18
#define ENA 25

#define ENCA_PENDULO 34
#define ENCB_PENDULO 35
#define ENCA_MOTOR 32
#define ENCB_MOTOR 33

#define PWM_FREQ 10000
#define PWM_RES 10
#define PWM_CH 0
#define DUTY_MAX 1023

#define DUTY_DEAD_FWD 550      
#define DUTY_DEAD_REV 550

#define CPR_PENDULO 1200.0f
#define GEAR_PENDULO 1.0f

#define CPR_MOTOR    150.0f
#define GEAR_MOTOR   1.0f

#define KP_PENDULO 0.0f
#define KI_PENDULO 0.0f
#define KD_PENDULO 0.0f
#define KP_MOTOR   0.0f
#define KI_MOTOR   0.0f
#define KD_MOTOR   0.0f


#define U_SAT 60.0f
#define I_BAND_DEG 20.0f // o integrador atua quando o erro está dentro dessa banda (em graus)
#define SETPOINT_DEG 0.0f
#define DEADBAND_DUTY 3

#define WATCHDOG_MS 60