#ifndef FOC_OBSERVER_H
#define FOC_OBSERVER_H

#include <stdbool.h>

typedef struct {
  float x_alpha;
  float x_beta;
  float phase_rad;
  float pll_phase_rad;
  float pll_speed_rad_per_second;
  bool using_encoder;
} foc_observer_state_t;

void foc_observer_reset(foc_observer_state_t *state, float phase_rad);

void foc_observer_update(foc_observer_state_t *state,
                         float voltage_alpha,
                         float voltage_beta,
                         float current_alpha,
                         float current_beta,
                         float duty_abs,
                         float dt_seconds);

void foc_observer_run_pll(foc_observer_state_t *state, float dt_seconds);

float foc_observer_select_phase(foc_observer_state_t *state,
                                float encoder_phase_rad);

#endif
