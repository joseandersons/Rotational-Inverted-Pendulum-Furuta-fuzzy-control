#include <Arduino.h>
#include "calibration.h"
#include "encoder.h"
#include "config.h"

static int32_t zero_offset_counts = 0;

void calibrar_zero_por_fundo(){
  for (int i = 10; i > 0; i--) {
    Serial.printf("[calib] calibre no FUNDO (180°)... %ds\n", i);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  int64_t acc = 0;
  const int N = 200;

  for (int i = 0; i < N; i++) {
    acc += encoder_pendulo_get_count_isr_safe();
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  int32_t fundo_counts = acc / N;

  const int32_t full_rev_counts = (int32_t)lroundf(CPR_PENDULO * GEAR_PENDULO);
  const int32_t half_rev_counts = full_rev_counts / 2;

  zero_offset_counts = fundo_counts + half_rev_counts;

  Serial.printf("[calib] OFFSET aplicado. fundo=%ld topo_offset=%ld\n",
                (long)fundo_counts, (long)zero_offset_counts);
}

int32_t get_zero_offset(){
  return zero_offset_counts;
}
