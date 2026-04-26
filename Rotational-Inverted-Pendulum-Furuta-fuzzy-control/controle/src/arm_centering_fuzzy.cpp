#include <math.h>
#include "arm_centering_fuzzy.h"
#include "params.h"

static inline float limitar(float valor, float minimo, float maximo){
  return (valor < minimo) ? minimo : (valor > maximo) ? maximo : valor;
}

static inline float pertinencia_triangular(float x, float esquerda, float pico, float direita){
  if (x <= esquerda || x >= direita) return 0.0f;
  if (x == pico) return 1.0f;
  if (x < pico) return (x - esquerda) / (pico - esquerda);
  return (direita - x) / (direita - pico);
}

static inline float pertinencia_trapezoidal(float x, float esquerda0, float esquerda1, float direita1, float direita0){
  if (x <= esquerda0 || x >= direita0) return 0.0f;
  if (x >= esquerda1 && x <= direita1) return 1.0f;
  if (x < esquerda1) return (x - esquerda0) / (esquerda1 - esquerda0);
  return (direita0 - x) / (direita0 - direita1);
}

float arm_centering_fuzzy_sp_offset_deg(float angulo_braco_deg, float vel_braco_deg_s){
  ArmFuzzyParams fz = params_arm_fuzzy_get();

  float angulo_braco_abs_deg = fabsf(angulo_braco_deg);
  float vel_braco_abs_deg_s  = fabsf(vel_braco_deg_s);

  float braco_esta_seguro = pertinencia_trapezoidal(
    angulo_braco_abs_deg,
    0.0f,
    0.0f,
    fmaxf(0.0f, fz.inicio_acao_deg - 10.0f),
    fz.inicio_acao_deg
  );

  float braco_esta_perto_limite = pertinencia_triangular(
    angulo_braco_abs_deg,
    fz.inicio_acao_deg,
    0.5f * (fz.inicio_acao_deg + fz.acao_maxima_deg),
    fz.acao_maxima_deg
  );

  float braco_esta_critico = pertinencia_trapezoidal(
    angulo_braco_abs_deg,
    fz.acao_maxima_deg - 8.0f,
    fz.acao_maxima_deg - 2.0f,
    120.0f,
    180.0f
  );

  float vel_esta_alta = pertinencia_trapezoidal(
    vel_braco_abs_deg_s,
    0.6f * fz.limiar_vel_alta_deg_s,
    fz.limiar_vel_alta_deg_s,
    2000.0f,
    4000.0f
  );

  float soma_pesos_regras = 0.0f;
  float soma_saidas_ponderadas = 0.0f;

  auto aplicar_regra = [&](float peso_regra, float saida_regra){
    if (peso_regra <= 0.0f) return;
    soma_pesos_regras += peso_regra;
    soma_saidas_ponderadas += peso_regra * saida_regra;
  };

  aplicar_regra(braco_esta_seguro, 0.0f);
  aplicar_regra(braco_esta_perto_limite, 0.35f * fz.offset_max_setpoint_deg);
  aplicar_regra(braco_esta_critico, 1.00f * fz.offset_max_setpoint_deg);
  aplicar_regra(fminf(braco_esta_critico, vel_esta_alta), 1.00f * fz.offset_max_setpoint_deg);
  aplicar_regra(fminf(braco_esta_perto_limite, vel_esta_alta), 0.50f * fz.offset_max_setpoint_deg);

  float modulo_offset_deg =
    (soma_pesos_regras > 1e-6f)
      ? (soma_saidas_ponderadas / soma_pesos_regras)
      : 0.0f;

  modulo_offset_deg = limitar(modulo_offset_deg, 0.0f, fz.offset_max_setpoint_deg);

  float direcao_para_centro = (angulo_braco_deg >= 0.0f) ? +1.0f : -1.0f;

  return direcao_para_centro * modulo_offset_deg;
}