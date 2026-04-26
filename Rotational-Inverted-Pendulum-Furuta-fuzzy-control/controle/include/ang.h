#pragma once
#include <Arduino.h>
#include "config.h"

#ifndef TAU_ANG
#define TAU_ANG (2.0f * (float)PI)
#endif

static inline float wrapPi(float x) {
  while (x >  (float)PI) x -= TAU_ANG;
  while (x < -(float)PI) x += TAU_ANG;
  return x;
}

static inline int32_t wrap_counts_pendulo(int32_t diff){
  const int32_t full = (int32_t)lroundf(CPR_PENDULO * GEAR_PENDULO);
  const int32_t half = full / 2;
  while (diff >  half) diff -= full;
  while (diff < -half) diff += full;
  return diff;
}

static inline int32_t wrap_counts_motor(int32_t c){
  const int32_t full = (int32_t)lroundf(CPR_MOTOR * GEAR_MOTOR);
  const int32_t half = full / 2;
  while (c >  half) c -= full;
  while (c < -half) c += full;
  return c;
}
