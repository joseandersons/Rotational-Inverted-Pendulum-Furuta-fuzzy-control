#include <Arduino.h>
#include "filas.h"
#include "tipos.h"
#include "config.h"

#define TEL        Serial
#define TEL_BEGIN()  do { Serial.begin(115200); Serial.setTxBufferSize(2048); } while(0)

static inline void tx_line(const TelePkt& p){
  if (TEL.availableForWrite() < 96) return;
  char buf[160];
  int n = snprintf(buf, sizeof(buf),
    "T,%u,TH,%d,THM,%d,SP,%d,E,%d,DUTY,%d\n",
    p.t_ms, p.th_c2, p.thm_c2, p.sp_c2, p.e_c2, p.duty
  );
  if (n > 0) TEL.write((const uint8_t*)buf, (size_t)n);
}


static void serial_tx_task(void*){
  TEL_BEGIN();
  TelePkt pkt{};
  for(;;){
    if (xQueueReceive(qTel, &pkt, pdMS_TO_TICKS(10)) == pdTRUE) {
      tx_line(pkt);
    }
  }
}

void serial_tx_task_start(){
  xTaskCreatePinnedToCore(serial_tx_task, "thTEL", 3072, nullptr,
                          tskIDLE_PRIORITY+1, nullptr, CORE_IO);
}
