#include <Arduino.h>
#include <string.h>
#include "serial_cmd.h"
#include "params.h"
#include "config.h"
#include "control.h"

static void serial_cmd_task(void*){
  Serial.println("[serial] cmd task start");
  static char buf[128];
  size_t n = 0;

  for(;;){
    while(Serial.available()){
      char c = (char)Serial.read();

      if (c == '\n' || n >= sizeof(buf)-1){
        buf[n] = 0;

        if (n){
          float kp, ki, kd;
          float sp;
          float inicio_acao_deg, acao_maxima_deg, offset_max_setpoint_deg, limiar_vel_alta_deg_s;

          if (sscanf(buf, "GP %f %f %f", &kp, &ki, &kd) == 3){
            params_pendulo_set(kp, ki, kd);
            auto g = params_pendulo_get();
            Serial.printf("GP %.6f %.6f %.6f\n", g.kp, g.ki, g.kd);

          } else if (sscanf(buf, "GM %f %f %f", &kp, &ki, &kd) == 3){
            params_motor_set(kp, ki, kd);
            auto m = params_motor_get();
            Serial.printf("GM %.6f %.6f %.6f\n", m.kp, m.ki, m.kd);

          } else if (sscanf(buf, "FZ %f %f %f %f",
                            &inicio_acao_deg,
                            &acao_maxima_deg,
                            &offset_max_setpoint_deg,
                            &limiar_vel_alta_deg_s) == 4){
            params_arm_fuzzy_set(
              inicio_acao_deg,
              acao_maxima_deg,
              offset_max_setpoint_deg,
              limiar_vel_alta_deg_s
            );
            auto fz = params_arm_fuzzy_get();
            Serial.printf("FZ %.6f %.6f %.6f %.6f\n",
                          fz.inicio_acao_deg,
                          fz.acao_maxima_deg,
                          fz.offset_max_setpoint_deg,
                          fz.limiar_vel_alta_deg_s);

          } else if (strcmp(buf, "START") == 0){
            control_request_start();
            Serial.println("START");

          } else if (strcmp(buf, "STOP") == 0){
            control_stop();
            Serial.println("STOP");

          } else {
            Serial.println("ERR");
          }
        }

        n = 0;

      } else if (c != '\r'){
        buf[n++] = c;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void serial_cmd_start(){
  xTaskCreatePinnedToCore(serial_cmd_task, "thSER", 3072, nullptr,
                          PRIO_ENCODER, nullptr, CORE_IO);
}