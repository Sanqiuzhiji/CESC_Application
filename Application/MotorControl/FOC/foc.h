#ifndef FOC_H
#define FOC_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float alpha;
  float beta;
  float d;
  float q;
} foc_dq_sample_t;

typedef struct {
  float integral_d;
  float integral_q;
} foc_current_controller_t;

typedef struct {
  float d;
  float q;
  float alpha;
  float beta;
  uint16_t request_counts;
  bool integral_d_saturated;
  bool integral_q_saturated;
  bool voltage_saturated;
} foc_voltage_output_t;

typedef struct {
  uint16_t a;
  uint16_t b;
  uint16_t c;
} foc_pwm_output_t;

float foc_sine_raw(uint16_t angle_raw);

foc_dq_sample_t foc_clarke_park(float phase_a,
                                float phase_b,
                                float phase_c,
                                bool full_clarke,
                                uint16_t electrical_raw);

foc_voltage_output_t foc_current_control(
    foc_current_controller_t *controller,
    float id,
    float iq,
    float id_target,
    float iq_target,
    float kp_counts_per_amp,
    float ki_counts_per_amp_second,
    float dt_seconds,
    float voltage_limit_counts,
    uint16_t electrical_raw);

foc_pwm_output_t foc_voltage_to_pwm(float voltage_alpha,
                                    float voltage_beta,
                                    uint16_t timer_period);

/** VESC-style six-sector space-vector modulation for alpha/beta voltage. */
foc_pwm_output_t foc_svm_voltage_to_pwm(float voltage_alpha,
                                        float voltage_beta,
                                        uint16_t timer_period,
                                        uint8_t *sector);

foc_pwm_output_t foc_oriented_voltage_to_pwm(uint16_t electrical_raw,
                                             int32_t voltage_q_counts,
                                             uint16_t timer_period);

#endif
