#include "foc.h"

#include <limits.h>
#include <math.h>

static uint16_t clamp_compare(int32_t compare, uint16_t timer_period);

float foc_sine_raw(uint16_t angle_raw)
{
  float angle = (float)(angle_raw & 0x0FFFU) *
                (6.28318530718F / 4096.0F);
  float value;
  if (angle > 3.14159265359F) {
    angle -= 6.28318530718F;
  }
  if (angle < 0.0F) {
    value = 1.27323954F * angle + 0.405284735F * angle * angle;
  } else {
    value = 1.27323954F * angle - 0.405284735F * angle * angle;
  }
  return value < 0.0F ?
      0.225F * (value * -value - value) + value :
      0.225F * (value * value - value) + value;
}

foc_dq_sample_t foc_clarke_park(float phase_a,
                                float phase_b,
                                float phase_c,
                                bool full_clarke,
                                uint16_t electrical_raw)
{
  foc_dq_sample_t result;
  result.alpha = full_clarke ?
      (0.666666667F * phase_a - 0.333333333F * phase_b -
       0.333333333F * phase_c) : phase_a;
  result.beta = full_clarke ?
      (0.577350269F * (phase_b - phase_c)) :
      (0.577350269F * phase_a + 1.154700538F * phase_b);
  const float sine = foc_sine_raw(electrical_raw);
  const float cosine = foc_sine_raw((uint16_t)(electrical_raw + 1024U));
  result.d = cosine * result.alpha + sine * result.beta;
  result.q = cosine * result.beta - sine * result.alpha;
  return result;
}

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
    uint16_t electrical_raw)
{
  foc_voltage_output_t result = {0};
  const float error_d = id_target - id;
  const float error_q = iq_target - iq;
  controller->integral_d += error_d * ki_counts_per_amp_second * dt_seconds;
  controller->integral_q += error_q * ki_counts_per_amp_second * dt_seconds;
  if (controller->integral_d > voltage_limit_counts) {
    controller->integral_d = voltage_limit_counts;
    result.integral_d_saturated = true;
  } else if (controller->integral_d < -voltage_limit_counts) {
    controller->integral_d = -voltage_limit_counts;
    result.integral_d_saturated = true;
  }
  if (controller->integral_q > voltage_limit_counts) {
    controller->integral_q = voltage_limit_counts;
    result.integral_q_saturated = true;
  } else if (controller->integral_q < -voltage_limit_counts) {
    controller->integral_q = -voltage_limit_counts;
    result.integral_q_saturated = true;
  }

  result.d = controller->integral_d + kp_counts_per_amp * error_d;
  result.q = controller->integral_q + kp_counts_per_amp * error_q;
  const float request = sqrtf(result.d * result.d + result.q * result.q);
  result.request_counts = request > (float)UINT16_MAX ?
      UINT16_MAX : (uint16_t)request;
  if (request > voltage_limit_counts) {
    const float scale = voltage_limit_counts / request;
    result.d *= scale;
    result.q *= scale;
    result.voltage_saturated = true;
  }

  const float sine = foc_sine_raw(electrical_raw);
  const float cosine = foc_sine_raw((uint16_t)(electrical_raw + 1024U));
  result.alpha = cosine * result.d - sine * result.q;
  result.beta = sine * result.d + cosine * result.q;
  return result;
}

foc_pwm_output_t foc_svm_voltage_to_pwm(float voltage_alpha,
                                        float voltage_beta,
                                        uint16_t timer_period,
                                        uint8_t *sector_out)
{
  /* mcpwm_foc.c::svm() uses the opposite alpha/beta PWM polarity. */
  const float alpha = -voltage_alpha / (float)timer_period;
  const float beta = -voltage_beta / (float)timer_period;
  const float one_by_sqrt3 = 0.577350269F;
  const float two_by_sqrt3 = 1.154700538F;
  uint8_t sector;
  if (beta >= 0.0F) {
    if (alpha >= 0.0F) {
      sector = one_by_sqrt3 * beta > alpha ? 2U : 1U;
    } else {
      sector = -one_by_sqrt3 * beta > alpha ? 3U : 2U;
    }
  } else if (alpha >= 0.0F) {
    sector = -one_by_sqrt3 * beta > alpha ? 5U : 6U;
  } else {
    sector = one_by_sqrt3 * beta > alpha ? 4U : 5U;
  }

  float t_a;
  float t_b;
  float t_c;
  const float period = (float)timer_period;
  switch (sector) {
  case 1: {
    const float t1 = (alpha - one_by_sqrt3 * beta) * period;
    const float t2 = two_by_sqrt3 * beta * period;
    t_a = (period - t1 - t2) * 0.5F;
    t_b = t_a + t1;
    t_c = t_b + t2;
    break;
  }
  case 2: {
    const float t2 = (alpha + one_by_sqrt3 * beta) * period;
    const float t3 = (-alpha + one_by_sqrt3 * beta) * period;
    t_b = (period - t2 - t3) * 0.5F;
    t_a = t_b + t3;
    t_c = t_a + t2;
    break;
  }
  case 3: {
    const float t3 = two_by_sqrt3 * beta * period;
    const float t4 = (-alpha - one_by_sqrt3 * beta) * period;
    t_b = (period - t3 - t4) * 0.5F;
    t_c = t_b + t3;
    t_a = t_c + t4;
    break;
  }
  case 4: {
    const float t4 = (-alpha + one_by_sqrt3 * beta) * period;
    const float t5 = -two_by_sqrt3 * beta * period;
    t_c = (period - t4 - t5) * 0.5F;
    t_b = t_c + t5;
    t_a = t_b + t4;
    break;
  }
  case 5: {
    const float t5 = (-alpha - one_by_sqrt3 * beta) * period;
    const float t6 = (alpha - one_by_sqrt3 * beta) * period;
    t_c = (period - t5 - t6) * 0.5F;
    t_a = t_c + t5;
    t_b = t_a + t6;
    break;
  }
  default: {
    const float t6 = -two_by_sqrt3 * beta * period;
    const float t1 = (alpha + one_by_sqrt3 * beta) * period;
    t_a = (period - t6 - t1) * 0.5F;
    t_c = t_a + t1;
    t_b = t_c + t6;
    break;
  }
  }

  foc_pwm_output_t result = {
    .a = clamp_compare((int32_t)t_a, timer_period),
    .b = clamp_compare((int32_t)t_b, timer_period),
    .c = clamp_compare((int32_t)t_c, timer_period),
  };
  if (sector_out != NULL) {
    *sector_out = sector;
  }
  return result;
}

static uint16_t clamp_compare(int32_t compare, uint16_t timer_period)
{
  if (compare < 0) {
    return 0U;
  }
  if (compare > (int32_t)timer_period) {
    return timer_period;
  }
  return (uint16_t)compare;
}

foc_pwm_output_t foc_voltage_to_pwm(float voltage_alpha,
                                    float voltage_beta,
                                    uint16_t timer_period)
{
  const int32_t neutral = (int32_t)(timer_period / 2U);
  foc_pwm_output_t result;
  result.a = clamp_compare(neutral + (int32_t)voltage_alpha, timer_period);
  result.b = clamp_compare(neutral + (int32_t)(
      -0.5F * voltage_alpha + 0.866025404F * voltage_beta), timer_period);
  result.c = clamp_compare(neutral + (int32_t)(
      -0.5F * voltage_alpha - 0.866025404F * voltage_beta), timer_period);
  return result;
}

foc_pwm_output_t foc_oriented_voltage_to_pwm(uint16_t electrical_raw,
                                             int32_t voltage_q_counts,
                                             uint16_t timer_period)
{
  const uint16_t voltage_angle = (uint16_t)(electrical_raw +
      (voltage_q_counts < 0 ? 3072U : 1024U));
  const float amplitude = (float)(voltage_q_counts < 0 ?
      -voltage_q_counts : voltage_q_counts);
  const float alpha = amplitude *
      foc_sine_raw((uint16_t)(voltage_angle + 1024U));
  const float beta = amplitude * foc_sine_raw(voltage_angle);
  return foc_voltage_to_pwm(alpha, beta, timer_period);
}
