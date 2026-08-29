#include "foc_observer.h"

#include <math.h>
#include <stddef.h>

#include "motor_control_config.h"

static float wrap_radians(float angle)
{
  while (angle > 3.14159265359F) { angle -= 6.28318530718F; }
  while (angle < -3.14159265359F) { angle += 6.28318530718F; }
  return angle;
}

void foc_observer_reset(foc_observer_state_t *state, float phase_rad)
{
  if (state == NULL) { return; }
  state->x_alpha = 0.0F;
  state->x_beta = 0.0F;
  state->phase_rad = wrap_radians(phase_rad);
  state->pll_phase_rad = state->phase_rad;
  state->pll_speed_rad_per_second = 0.0F;
  state->using_encoder = true;
}

/* Ortega flux observer from the pinned VESC mcpwm_foc.c observer_update().
 * CESC keeps the same 3/2 phase-parameter normalization, six sub-iterations,
 * flux-error clamp and low-duty observer-gain scaling. */
void foc_observer_update(foc_observer_state_t *state,
                         float voltage_alpha,
                         float voltage_beta,
                         float current_alpha,
                         float current_beta,
                         float duty_abs,
                         float dt_seconds)
{
  if (state == NULL || dt_seconds <= 0.0F) { return; }
  if (duty_abs < 0.0F) { duty_abs = -duty_abs; }
  if (duty_abs > 1.0F) { duty_abs = 1.0F; }

  const float inductance =
      1.5F * motor_control_config.motor_inductance_h;
  const float resistance =
      1.5F * motor_control_config.motor_resistance_ohm;
  const float flux = motor_control_config.motor_flux_linkage_wb;
  const float l_i_alpha = inductance * current_alpha;
  const float l_i_beta = inductance * current_beta;
  const float r_i_alpha = resistance * current_alpha;
  const float r_i_beta = resistance * current_beta;
  const float flux_squared = flux * flux;
  const float gain_scale = motor_control_config.observer_gain_slow +
      (1.0F - motor_control_config.observer_gain_slow) * duty_abs;
  const float gain_half =
      motor_control_config.observer_gain * gain_scale * 0.5F;
  const float dt_iteration = dt_seconds / 6.0F;

  for (unsigned int iteration = 0U; iteration < 6U; ++iteration) {
    float error = flux_squared -
        ((state->x_alpha - l_i_alpha) * (state->x_alpha - l_i_alpha) +
         (state->x_beta - l_i_beta) * (state->x_beta - l_i_beta));
    float error_limit = flux_squared * 0.2F;
    float gain = gain_half;
    if (error > error_limit) {
      error = error_limit;
      gain *= 10.0F;
    } else if (error < -error_limit) {
      error = -error_limit;
      gain *= 10.0F;
    }
    state->x_alpha += (-r_i_alpha + voltage_alpha +
        gain * (state->x_alpha - l_i_alpha) * error) * dt_iteration;
    state->x_beta += (-r_i_beta + voltage_beta +
        gain * (state->x_beta - l_i_beta) * error) * dt_iteration;
  }

  if (!isfinite(state->x_alpha)) { state->x_alpha = 0.0F; }
  if (!isfinite(state->x_beta)) { state->x_beta = 0.0F; }
  state->phase_rad = atan2f(state->x_beta - l_i_beta,
                            state->x_alpha - l_i_alpha);
}

/* Direct port of VESC pll_run(). */
void foc_observer_run_pll(foc_observer_state_t *state, float dt_seconds)
{
  if (state == NULL || dt_seconds <= 0.0F) { return; }
  float phase_error = wrap_radians(state->phase_rad - state->pll_phase_rad);
  state->pll_phase_rad = wrap_radians(state->pll_phase_rad +
      (state->pll_speed_rad_per_second +
       motor_control_config.observer_pll_kp * phase_error) * dt_seconds);
  state->pll_speed_rad_per_second +=
      motor_control_config.observer_pll_ki * phase_error * dt_seconds;
  if (!isfinite(state->pll_speed_rad_per_second)) {
    state->pll_speed_rad_per_second = 0.0F;
  }
}

/* Direct port of VESC correct_encoder(): foc_sl_erpm with 5% hysteresis. */
float foc_observer_select_phase(foc_observer_state_t *state,
                                float encoder_phase_rad)
{
  if (state == NULL) { return encoder_phase_rad; }
  const float erpm = fabsf(state->pll_speed_rad_per_second) *
      (60.0F / 6.28318530718F);
  const float hysteresis =
      motor_control_config.observer_transition_erpm * 0.05F;
  if (state->using_encoder) {
    if (erpm > motor_control_config.observer_transition_erpm + hysteresis) {
      state->using_encoder = false;
    }
  } else if (erpm <
             motor_control_config.observer_transition_erpm - hysteresis) {
    state->using_encoder = true;
  }
  return state->using_encoder ? encoder_phase_rad : state->phase_rad;
}
