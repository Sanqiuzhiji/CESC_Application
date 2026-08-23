#include "power_stage.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

#include "adc.h"
#include "angle_sensor.h"
#include "drv8301.h"
#include "main.h"
#include "tim.h"

enum {
  CURRENT_CALIBRATION_SAMPLES = 1024U,
  /* Default commissioning ceiling: +/-10% around center-aligned neutral. */
  COMMISSIONING_MODULATION_DIVISOR = 10U,
  /* Rs identification needs 2.7 V for 0.5 A into the specified 5.4 ohm phase. */
  RESISTANCE_MODULATION_DIVISOR = 2U,
  BUS_VOLTAGE_SAMPLE_PERIOD_MS = 100U,
  ADC_FULL_SCALE = 4095U,
  ADC_REFERENCE_MV = 3300U,
  BUS_DIVIDER_HIGH_OHMS = 39000U,
  BUS_DIVIDER_LOW_OHMS = 2200U,
  COMMISSIONING_TEST_TIMEOUT_MS = 3000U,
  COMMISSIONING_ALIGNMENT_MS = 500U,
  COMMISSIONING_STEP_PERIOD_MS = 50U,
  COMMISSIONING_STEP_COUNT = 30U,
  COMMISSIONING_SETTLE_MS = 200U,
  ENCODER_ALIGNMENT_ZERO_CAPTURE_MS = 1300U,
  ENCODER_ALIGNMENT_QUADRATURE_START_MS = 1500U,
  CURRENT_FOC_ALIGNMENT_RAMP_UP_MS = 600U,
  CURRENT_FOC_ALIGNMENT_RAMP_DOWN_MS = 200U,
  ENCODER_ALIGNMENT_HOLD_MS = 3000U,
  ENCODER_ALIGNMENT_TIMEOUT_MS = 4000U,
  ENCODER_VOLTAGE_ALIGN_MS = 1500U,
  ENCODER_VOLTAGE_ROTATE_MS = 10000U,
  ENCODER_VOLTAGE_TIMEOUT_MS = 13000U,
  ENCODER_VOLTAGE_TARGET_DEGREES_PER_SECOND = 2U,
  ENCODER_VOLTAGE_RAMP_DOWN_MS = 500U,
  ENCODER_VOLTAGE_FULL_OUTPUT_ERROR_COUNTS = 46U,
  CURRENT_FOC_ACTIVE_MS = 10000U,
  CURRENT_FOC_TIMEOUT_MS = 13000U,
  CURRENT_FOC_MODULATION_DIVISOR = 20U,
  CURRENT_FOC_TARGET_DEGREES_PER_SECOND = 10U,
  CURRENT_FOC_OUTER_LOOP_PERIOD_MS = 10U,
  CURRENT_FOC_RAMP_DOWN_MS = 500U,
  RESISTANCE_RAMP_MS = 500U,
  RESISTANCE_SETTLE_MS = 800U,
  RESISTANCE_SAMPLE_MS = 1000U,
  RESISTANCE_ZERO_MS = 300U,
  RESISTANCE_RAMP_DOWN_MS = 500U,
  RESISTANCE_TIMEOUT_MS = 6000U,
  INDUCTANCE_TIMEOUT_MS = 3000U,
  INDUCTANCE_HALF_PERIOD_TICKS = 20U,
  INDUCTANCE_ENDPOINT_AVERAGE_TICKS = 8U,
  INDUCTANCE_SETTLE_HALF_PERIODS = 40U,
  INDUCTANCE_TARGET_SAMPLES_PER_DIRECTION = 250U,
  FLUX_SAMPLE_START_MS = 4000U,
  FLUX_SAMPLE_END_MS = 9000U,
  ENCODER_CONTROL_MAX_SAMPLE_AGE_MS = 50U,
  ENCODER_CONTROL_INVALID_LIMIT = 5U,
  CURRENT_TRANSFORM_DIAGNOSTIC_DIVIDER = 20U,
  COMMISSIONING_MIN_BUS_MV = 6000U,
  COMMISSIONING_MAX_BUS_MV = 10000U,
  CURRENT_TRIP_ADC_COUNTS = 20U,
  CURRENT_TRIP_CONSECUTIVE_SAMPLES = 3U,
  SOFTWARE_OVERCURRENT_FAULT = 1U << 15,
  COMMISSIONING_TEST_MODULATION_DIVISOR = 12U,
  ADC_SAMPLE_TOP_MARGIN_COUNTS = 800U
};

/*
 * AD8418 gain = 20 V/V and phase shunt = 0.5 mOhm, giving
 * 0.08058608 A/count at VDDA = 3.3 V. The schematic connects IN+ to SH_x
 * and IN- to motor terminal P_x, so a positive ADC delta represents current
 * from the motor back into the bridge. FOC phase current is defined in the
 * opposite direction (bridge to motor), hence the negative sign.
 */
static const float CURRENT_ADC_TO_AMPS = -0.08058608F;

static volatile power_stage_state_t stage_state = POWER_STAGE_UNINITIALIZED;
static volatile uint32_t current_sequence;
static volatile uint16_t current_raw[3];
static volatile uint16_t current_offset[3];
static volatile uint32_t calibration_sum[3];
static volatile uint16_t calibration_count;
static volatile uint16_t latched_faults;
static volatile bool fault_status_pending;
static uint16_t bus_voltage_raw;
static uint32_t bus_voltage_mv;
static uint32_t bus_voltage_sample_time;
static bool bus_voltage_valid;
static volatile power_stage_test_state_t test_state = POWER_STAGE_TEST_IDLE;
static uint32_t test_started_at;
static uint32_t test_last_step_at;
static uint8_t test_step;
static volatile uint8_t test_steps_completed;
static uint32_t test_settle_started_at;
static int8_t test_direction;
static volatile uint8_t overcurrent_count;
typedef enum {
  TEST_KIND_NONE = 0,
  TEST_KIND_COMMUTATION,
  TEST_KIND_ENCODER_ALIGNMENT,
  TEST_KIND_ENCODER_VOLTAGE,
  TEST_KIND_CURRENT_FOC,
  TEST_KIND_RESISTANCE,
  TEST_KIND_INDUCTANCE,
  TEST_KIND_FLUX
} test_kind_t;
static test_kind_t test_kind;
static bool test_alignment_calibrated;
static bool test_alignment_quadrature_started;
static uint32_t test_rotation_started_at;
static uint16_t test_command_electrical_raw;
static int32_t test_start_position_counts;
static uint8_t test_sensor_invalid_count;
static volatile uint32_t test_current_samples;
static volatile int64_t test_current_sum[3];
static volatile int16_t test_current_min[3];
static volatile int16_t test_current_max[3];
static volatile uint64_t test_current_balance_abs_sum;
static volatile uint16_t test_current_balance_abs_max;
static volatile uint32_t test_v0_samples;
static volatile uint32_t test_v7_samples;
static volatile uint32_t test_reconstructed_samples[3];
static volatile uint32_t test_transform_samples;
static volatile int64_t test_id_sum_ma;
static volatile int64_t test_iq_sum_ma;
static volatile int32_t test_id_min_ma;
static volatile int32_t test_id_max_ma;
static volatile int32_t test_iq_min_ma;
static volatile int32_t test_iq_max_ma;
static volatile int64_t test_iq_target_sum_ma;
static volatile int32_t test_iq_target_min_ma;
static volatile int32_t test_iq_target_max_ma;
static volatile uint32_t test_voltage_saturated_samples;
static volatile uint32_t test_integral_d_saturated_samples;
static volatile uint32_t test_integral_q_saturated_samples;
static volatile uint64_t test_voltage_request_sum_counts;
static volatile uint16_t test_voltage_request_max_counts;
static volatile uint32_t resistance_measurement_samples;
static volatile int64_t resistance_iq_sum_ma;
static volatile int64_t resistance_vq_sum_mv;
typedef enum {
  RESISTANCE_PHASE_IDLE = 0,
  RESISTANCE_PHASE_OFFSET_CALIBRATION,
  RESISTANCE_PHASE_ENABLE_PENDING,
  RESISTANCE_PHASE_FORWARD_RAMP,
  RESISTANCE_PHASE_FORWARD_SETTLE,
  RESISTANCE_PHASE_FORWARD_SAMPLE,
  RESISTANCE_PHASE_ZERO_SETTLE,
  RESISTANCE_PHASE_REVERSE_RAMP,
  RESISTANCE_PHASE_REVERSE_SETTLE,
  RESISTANCE_PHASE_REVERSE_SAMPLE,
  RESISTANCE_PHASE_RAMP_DOWN,
  RESISTANCE_PHASE_COMPLETE
} resistance_phase_t;
static volatile resistance_phase_t resistance_phase;
static uint32_t resistance_phase_started_at;
static volatile uint32_t resistance_forward_samples;
static volatile uint32_t resistance_reverse_samples;
static volatile int64_t resistance_forward_id_sum_ma;
static volatile int64_t resistance_forward_vd_sum_mv;
static volatile int64_t resistance_reverse_id_sum_ma;
static volatile int64_t resistance_reverse_vd_sum_mv;
static volatile int32_t resistance_id_ma;
static volatile int32_t resistance_iq_ma;
static volatile int32_t resistance_vd_mv;
static volatile int32_t resistance_vq_mv;
static volatile int32_t resistance_live_milliohms;
static volatile int32_t resistance_forward_milliohms;
static volatile int32_t resistance_reverse_milliohms;
static volatile int32_t resistance_average_milliohms;
static volatile bool resistance_valid;
typedef enum {
  INDUCTANCE_PHASE_IDLE = 0,
  INDUCTANCE_PHASE_OFFSET_CALIBRATION,
  INDUCTANCE_PHASE_ENABLE_PENDING,
  INDUCTANCE_PHASE_PULSING,
  INDUCTANCE_PHASE_COMPLETE
} inductance_phase_t;
static volatile inductance_phase_t inductance_phase;
static volatile uint16_t inductance_tick;
static volatile uint16_t inductance_half_periods;
static volatile int8_t inductance_direction;
static volatile int64_t inductance_baseline_sum_ma;
static volatile uint8_t inductance_baseline_samples;
static volatile uint32_t inductance_forward_samples;
static volatile uint32_t inductance_reverse_samples;
static volatile int64_t inductance_forward_delta_sum_ma;
static volatile int64_t inductance_reverse_delta_sum_ma;
static volatile int64_t inductance_forward_voltage_sum_mv;
static volatile int64_t inductance_reverse_voltage_sum_mv;
static volatile int32_t inductance_delta_current_ma;
static volatile int32_t inductance_voltage_mv;
static volatile uint32_t inductance_forward_uh;
static volatile uint32_t inductance_reverse_uh;
static volatile uint32_t inductance_average_uh;
static volatile bool inductance_valid;
static volatile uint32_t flux_samples;
static volatile int64_t flux_speed_sum_mdps;
static volatile int64_t flux_iq_sum_ma;
static volatile int64_t flux_vq_sum_mv;
static volatile int32_t flux_speed_mdps;
static volatile int32_t flux_iq_ma;
static volatile int32_t flux_vq_mv;
static volatile uint32_t flux_linkage_uwb;
static volatile uint32_t flux_ke_uv_per_rad_s;
static volatile uint32_t flux_kv_millirpm_per_volt;
static volatile bool flux_valid;
static volatile uint8_t test_transform_divider;
static volatile bool current_foc_active;
static volatile float current_foc_integral_d;
static volatile float current_foc_integral_q;
static volatile float current_foc_id_target;
static volatile float current_foc_iq_target;
static uint32_t current_foc_outer_last_ms;
static float current_foc_pll_position_counts;
static float current_foc_pll_speed_counts_per_second;
static float current_foc_speed_reference_dps;
static float current_foc_target_position_counts;
static float current_foc_speed_integral_amps;

static const float CURRENT_FOC_MAX_TARGET_AMPS = 0.30F;
static const float RESISTANCE_MEASUREMENT_TARGET_AMPS = 0.50F;
static const float CURRENT_FOC_TORQUE_FEEDFORWARD_AMPS = 0.12F;
static const float CURRENT_FOC_KP_COUNTS_PER_AMP = 40.0F;
static const float CURRENT_FOC_KI_COUNTS_PER_AMP_SECOND = 800.0F;
static const float RESISTANCE_FOC_KI_COUNTS_PER_AMP_SECOND = 5000.0F;
static const float CURRENT_FOC_DT_SECONDS = 0.00005F;
static const float CURRENT_FOC_POSITION_TO_SPEED_GAIN = 1.0F;
static const float CURRENT_FOC_MAX_SPEED_TARGET_DPS = 15.0F;
static const float CURRENT_FOC_ACCELERATION_DPS2 = 20.0F;
static const float CURRENT_FOC_PLL_KP_PER_SECOND = 40.0F;
static const float CURRENT_FOC_PLL_KI_PER_SECOND2 = 400.0F;
static const float CURRENT_FOC_SPEED_KP_AMPS_PER_DPS = 0.010F;
static const float CURRENT_FOC_SPEED_KI_AMPS_PER_DEGREE = 0.040F;

static void reset_test_current_statistics(void)
{
  test_current_samples = 0U;
  test_current_balance_abs_sum = 0U;
  test_current_balance_abs_max = 0U;
  test_v0_samples = 0U;
  test_v7_samples = 0U;
  test_transform_samples = 0U;
  test_id_sum_ma = 0;
  test_iq_sum_ma = 0;
  test_id_min_ma = INT32_MAX;
  test_id_max_ma = INT32_MIN;
  test_iq_min_ma = INT32_MAX;
  test_iq_max_ma = INT32_MIN;
  test_iq_target_sum_ma = 0;
  test_iq_target_min_ma = INT32_MAX;
  test_iq_target_max_ma = INT32_MIN;
  test_voltage_saturated_samples = 0U;
  test_integral_d_saturated_samples = 0U;
  test_integral_q_saturated_samples = 0U;
  test_voltage_request_sum_counts = 0U;
  test_voltage_request_max_counts = 0U;
  resistance_measurement_samples = 0U;
  resistance_iq_sum_ma = 0;
  resistance_vq_sum_mv = 0;
  resistance_phase = RESISTANCE_PHASE_IDLE;
  resistance_phase_started_at = 0U;
  resistance_forward_samples = 0U;
  resistance_reverse_samples = 0U;
  resistance_forward_id_sum_ma = 0;
  resistance_forward_vd_sum_mv = 0;
  resistance_reverse_id_sum_ma = 0;
  resistance_reverse_vd_sum_mv = 0;
  resistance_id_ma = 0;
  resistance_iq_ma = 0;
  resistance_vd_mv = 0;
  resistance_vq_mv = 0;
  resistance_live_milliohms = 0;
  resistance_forward_milliohms = 0;
  resistance_reverse_milliohms = 0;
  resistance_average_milliohms = 0;
  resistance_valid = false;
  test_transform_divider = 0U;
  current_foc_integral_d = 0.0F;
  current_foc_integral_q = 0.0F;
  current_foc_id_target = 0.0F;
  current_foc_iq_target = 0.0F;
  current_foc_outer_last_ms = 0U;
  current_foc_pll_position_counts = 0.0F;
  current_foc_pll_speed_counts_per_second = 0.0F;
  current_foc_speed_reference_dps = 0.0F;
  current_foc_target_position_counts = 0.0F;
  current_foc_speed_integral_amps = 0.0F;
  for (uint32_t phase = 0U; phase < 3U; ++phase) {
    test_current_sum[phase] = 0;
    test_current_min[phase] = INT16_MAX;
    test_current_max[phase] = INT16_MIN;
    test_reconstructed_samples[phase] = 0U;
  }
}

static void set_compare_values(uint16_t a, uint16_t b, uint16_t c);

static int32_t resistance_from_sums_milliohms(int64_t voltage_sum_mv,
                                              int64_t current_sum_ma)
{
  if ((current_sum_ma > -20) && (current_sum_ma < 20)) {
    return 0;
  }
  return (int32_t)((voltage_sum_mv * 1000) / current_sum_ma);
}

static void finish_resistance_measurement(void)
{
  resistance_forward_milliohms = resistance_from_sums_milliohms(
      resistance_forward_vd_sum_mv, resistance_forward_id_sum_ma);
  resistance_reverse_milliohms = resistance_from_sums_milliohms(
      resistance_reverse_vd_sum_mv, resistance_reverse_id_sum_ma);
  resistance_average_milliohms =
      (resistance_forward_milliohms + resistance_reverse_milliohms) / 2;
  const int32_t resistance_difference =
      resistance_forward_milliohms > resistance_reverse_milliohms ?
          resistance_forward_milliohms - resistance_reverse_milliohms :
          resistance_reverse_milliohms - resistance_forward_milliohms;
  resistance_valid =
      (resistance_forward_samples >= 1000U) &&
      (resistance_reverse_samples >= 1000U) &&
      (resistance_forward_id_sum_ma >
       (int64_t)resistance_forward_samples * 350) &&
      (resistance_reverse_id_sum_ma <
       -(int64_t)resistance_reverse_samples * 350) &&
      (resistance_forward_milliohms > 0) &&
      (resistance_reverse_milliohms > 0) &&
      ((int64_t)resistance_difference * 10 <=
       (int64_t)resistance_forward_milliohms +
       resistance_reverse_milliohms);
}

static void reset_inductance_statistics(void)
{
  inductance_phase = INDUCTANCE_PHASE_IDLE;
  inductance_tick = 0U;
  inductance_half_periods = 0U;
  inductance_direction = 1;
  inductance_baseline_sum_ma = 0;
  inductance_baseline_samples = 0U;
  inductance_forward_samples = 0U;
  inductance_reverse_samples = 0U;
  inductance_forward_delta_sum_ma = 0;
  inductance_reverse_delta_sum_ma = 0;
  inductance_forward_voltage_sum_mv = 0;
  inductance_reverse_voltage_sum_mv = 0;
  inductance_delta_current_ma = 0;
  inductance_voltage_mv = 0;
  inductance_forward_uh = 0U;
  inductance_reverse_uh = 0U;
  inductance_average_uh = 0U;
  inductance_valid = false;
}

static void reset_flux_statistics(void)
{
  flux_samples = 0U;
  flux_speed_sum_mdps = 0;
  flux_iq_sum_ma = 0;
  flux_vq_sum_mv = 0;
  flux_speed_mdps = 0;
  flux_iq_ma = 0;
  flux_vq_mv = 0;
  flux_linkage_uwb = 0U;
  flux_ke_uv_per_rad_s = 0U;
  flux_kv_millirpm_per_volt = 0U;
  flux_valid = false;
}

static void finish_flux_measurement(void)
{
  if (flux_samples < 1000U || !resistance_valid) {
    return;
  }
  flux_speed_mdps = (int32_t)(flux_speed_sum_mdps / flux_samples);
  flux_iq_ma = (int32_t)(flux_iq_sum_ma / flux_samples);
  flux_vq_mv = (int32_t)(flux_vq_sum_mv / flux_samples);
  const float omega_e = (float)flux_speed_mdps * 0.001F *
      (3.14159265359F / 180.0F) * 11.0F;
  const float bemf_mv = (float)flux_vq_mv -
      (float)resistance_average_milliohms * (float)flux_iq_ma * 0.001F;
  if (fabsf(omega_e) < 0.5F || bemf_mv * omega_e <= 0.0F) {
    return;
  }
  const float linkage_wb = (bemf_mv * 0.001F) / omega_e;
  if (linkage_wb < 0.001F || linkage_wb > 2.0F) {
    return;
  }
  flux_linkage_uwb = (uint32_t)(linkage_wb * 1000000.0F);
  /* Datasheet Ke uses mechanical rad/s; dq flux linkage uses electrical rad/s. */
  flux_ke_uv_per_rad_s = flux_linkage_uwb * 11U;
  flux_kv_millirpm_per_volt = (uint32_t)(
      60000.0F / (2.0F * 3.14159265359F * 11.0F * linkage_wb));
  flux_valid = true;
}

static void finish_inductance_measurement(void)
{
  const float half_period_us =
      (float)(INDUCTANCE_HALF_PERIOD_TICKS * 50U);
  const float resistance_milliohms =
      (float)resistance_average_milliohms;
  const float forward_current_ma = (float)inductance_forward_delta_sum_ma /
      (float)inductance_forward_samples;
  const float reverse_current_ma = (float)inductance_reverse_delta_sum_ma /
      (float)inductance_reverse_samples;
  const float forward_voltage_mv = (float)inductance_forward_voltage_sum_mv /
      (float)inductance_forward_samples;
  const float reverse_voltage_mv = (float)inductance_reverse_voltage_sum_mv /
      (float)inductance_reverse_samples;
  const float forward_ratio = forward_current_ma * resistance_milliohms /
      (forward_voltage_mv * 1000.0F);
  const float reverse_ratio = reverse_current_ma * resistance_milliohms /
      (reverse_voltage_mv * 1000.0F);
  if ((forward_ratio > 0.02F) && (forward_ratio < 0.98F)) {
    inductance_forward_uh = (uint32_t)(resistance_milliohms * half_period_us /
        (2000.0F * atanhf(forward_ratio)));
  }
  if ((reverse_ratio > 0.02F) && (reverse_ratio < 0.98F)) {
    inductance_reverse_uh = (uint32_t)(resistance_milliohms * half_period_us /
        (2000.0F * atanhf(reverse_ratio)));
  }
  const float differential_current_ma =
      (forward_current_ma + reverse_current_ma) * 0.5F;
  const float differential_voltage_mv =
      (forward_voltage_mv + reverse_voltage_mv) * 0.5F;
  const float differential_ratio = differential_current_ma *
      resistance_milliohms / (differential_voltage_mv * 1000.0F);
  if ((differential_ratio > 0.02F) && (differential_ratio < 0.98F)) {
    inductance_average_uh = (uint32_t)(
        resistance_milliohms * half_period_us /
        (2000.0F * atanhf(differential_ratio)));
  }
  inductance_valid =
      (inductance_forward_samples >=
       INDUCTANCE_TARGET_SAMPLES_PER_DIRECTION) &&
      (inductance_reverse_samples >=
       INDUCTANCE_TARGET_SAMPLES_PER_DIRECTION) &&
      (forward_current_ma > 100.0F) &&
      (reverse_current_ma > 100.0F) &&
      (inductance_average_uh >= 100U) &&
      (inductance_average_uh <= 100000U);
}

static void set_inductance_voltage(int8_t direction)
{
  const int32_t neutral = (int32_t)(TIM1->ARR / 2U);
  const int32_t voltage_d = direction == 0 ? 0 :
      (int32_t)(TIM1->ARR / 6U) * direction;
  set_compare_values((uint16_t)(neutral + voltage_d),
                     (uint16_t)(neutral - voltage_d / 2),
                     (uint16_t)(neutral - voltage_d / 2));
}

static float fast_sine_raw(uint16_t angle_raw)
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

static void set_voltage_vector(uint16_t electrical_raw, uint16_t amplitude_counts)
{
  const int32_t neutral = (int32_t)(TIM1->ARR / 2U);
  const float amplitude = (float)amplitude_counts;
  const int32_t a = neutral + (int32_t)(amplitude *
      fast_sine_raw((uint16_t)(electrical_raw + 1024U)));
  const int32_t b = neutral + (int32_t)(amplitude *
      fast_sine_raw((uint16_t)(electrical_raw + 3755U)));
  const int32_t c = neutral + (int32_t)(amplitude *
      fast_sine_raw((uint16_t)(electrical_raw + 2389U)));
  set_compare_values((uint16_t)a, (uint16_t)b, (uint16_t)c);
}

static void set_commutation_step(uint8_t step)
{
  const uint16_t neutral = (uint16_t)(TIM1->ARR / 2U);
  const uint16_t delta =
      (uint16_t)(TIM1->ARR / COMMISSIONING_TEST_MODULATION_DIVISOR);
  const uint16_t high = (uint16_t)(neutral + delta);
  const uint16_t low = (uint16_t)(neutral - delta);

  switch (step % 6U) {
  case 0U: set_compare_values(high, low, neutral); break;
  case 1U: set_compare_values(high, neutral, low); break;
  case 2U: set_compare_values(neutral, high, low); break;
  case 3U: set_compare_values(low, high, neutral); break;
  case 4U: set_compare_values(low, neutral, high); break;
  default: set_compare_values(neutral, low, high); break;
  }
}

static uint16_t clamp_compare(uint16_t compare)
{
  const uint16_t neutral = (uint16_t)(TIM1->ARR / 2U);
  const uint16_t divisor =
      ((test_kind == TEST_KIND_RESISTANCE) ||
       (test_kind == TEST_KIND_INDUCTANCE)) ?
      RESISTANCE_MODULATION_DIVISOR : COMMISSIONING_MODULATION_DIVISOR;
  const uint16_t deviation = (uint16_t)(TIM1->ARR / divisor);
  const uint16_t minimum = (uint16_t)(neutral - deviation);
  const uint16_t maximum = (uint16_t)(neutral + deviation);
  if (compare < minimum) {
    return minimum;
  }
  return compare > maximum ? maximum : compare;
}

static void set_compare_values(uint16_t a, uint16_t b, uint16_t c)
{
  TIM1->CR1 |= TIM_CR1_UDIS;
  TIM1->CCR1 = clamp_compare(a);
  TIM1->CCR2 = clamp_compare(b);
  TIM1->CCR3 = clamp_compare(c);
  TIM1->CR1 &= ~TIM_CR1_UDIS;
}

void power_stage_disable(void)
{
  current_foc_active = false;
  current_foc_id_target = 0.0F;
  current_foc_iq_target = 0.0F;
  /* Gate all timer outputs before changing the individual channel enables. */
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                  TIM_CCER_CC2E | TIM_CCER_CC2NE |
                  TIM_CCER_CC3E | TIM_CCER_CC3NE);
  HAL_GPIO_WritePin(DRV_EN_GATE_GPIO_Port, DRV_EN_GATE_Pin, GPIO_PIN_RESET);

  /*
   * TIM1_CC4 has no GPIO output and is used only as the ADC trigger. On this
   * STM32 advanced timer, MOE must remain set for the CC4 trigger waveform to
   * run. CH1..CH3 remain electrically disconnected because all six CCER bits
   * above are clear and EN_GATE is low.
   */
  if ((TIM1->CCER & TIM_CCER_CC4E) != 0U) {
    TIM1->BDTR |= TIM_BDTR_MOE;
  }

  if (stage_state == POWER_STAGE_RUNNING) {
    stage_state = POWER_STAGE_READY;
  }
}

bool power_stage_init(void)
{
  power_stage_disable();
  stage_state = POWER_STAGE_UNINITIALIZED;
  current_sequence = 0U;
  calibration_count = 0U;
  latched_faults = 0U;
  fault_status_pending = false;
  bus_voltage_raw = 0U;
  bus_voltage_mv = 0U;
  bus_voltage_sample_time = HAL_GetTick();
  bus_voltage_valid = false;
  test_state = POWER_STAGE_TEST_IDLE;
  test_kind = TEST_KIND_NONE;
  test_alignment_calibrated = false;
  test_alignment_quadrature_started = false;
  test_rotation_started_at = 0U;
  test_command_electrical_raw = 0U;
  test_start_position_counts = 0;
  test_sensor_invalid_count = 0U;
  current_foc_active = false;
  reset_test_current_statistics();
  reset_inductance_statistics();
  test_steps_completed = 0U;
  test_settle_started_at = 0U;
  overcurrent_count = 0U;
  for (uint32_t index = 0U; index < 3U; ++index) {
    current_raw[index] = 0U;
    current_offset[index] = 0U;
    calibration_sum[index] = 0U;
  }

  if (drv8301_init() != DRV8301_RESULT_OK) {
    stage_state = POWER_STAGE_FAULT;
    return false;
  }
  if (HAL_GPIO_ReadPin(DRV_FAULT_N_GPIO_Port, DRV_FAULT_N_Pin) ==
      GPIO_PIN_RESET) {
    fault_status_pending = true;
    stage_state = POWER_STAGE_FAULT;
    return false;
  }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
                        TIM1->ARR - ADC_SAMPLE_TOP_MARGIN_COUNTS);
  if ((HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK) ||
      (HAL_ADCEx_InjectedStart(&hadc3) != HAL_OK) ||
      (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)) {
    power_stage_disable();
    stage_state = POWER_STAGE_FAULT;
    return false;
  }

  /* CH4 keeps MOE set, while all CH1..CH3 output-enable bits remain clear. */
  stage_state = POWER_STAGE_CALIBRATING;
  return true;
}

void power_stage_process(void)
{
  uint16_t faults;
  const uint32_t now = HAL_GetTick();

  if (fault_status_pending &&
      (drv8301_read_faults(&faults) == DRV8301_RESULT_OK)) {
    latched_faults |= faults;
    fault_status_pending = false;
  }

  if ((uint32_t)(now - bus_voltage_sample_time) >=
      BUS_VOLTAGE_SAMPLE_PERIOD_MS) {
    bus_voltage_sample_time = now;
    if ((HAL_ADC_Start(&hadc1) == HAL_OK) &&
        (HAL_ADC_PollForConversion(&hadc1, 2U) == HAL_OK)) {
      bus_voltage_raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
      bus_voltage_mv = (uint32_t)(
          ((uint64_t)bus_voltage_raw * ADC_REFERENCE_MV *
           (BUS_DIVIDER_HIGH_OHMS + BUS_DIVIDER_LOW_OHMS)) /
          ((uint64_t)ADC_FULL_SCALE * BUS_DIVIDER_LOW_OHMS));
      bus_voltage_valid = true;
    }
  }

  if ((test_state == POWER_STAGE_TEST_RUNNING) &&
      (test_kind == TEST_KIND_RESISTANCE)) {
    const uint32_t phase_elapsed =
        (uint32_t)(now - resistance_phase_started_at);
    if ((uint32_t)(now - test_started_at) >= RESISTANCE_TIMEOUT_MS) {
      power_stage_disable();
      test_state = POWER_STAGE_TEST_ABORTED;
      test_kind = TEST_KIND_NONE;
    } else if ((resistance_phase != RESISTANCE_PHASE_OFFSET_CALIBRATION) &&
        (resistance_phase != RESISTANCE_PHASE_ENABLE_PENDING) &&
        (stage_state != POWER_STAGE_RUNNING)) {
      power_stage_disable();
      test_state = POWER_STAGE_TEST_ABORTED;
      test_kind = TEST_KIND_NONE;
    } else {
      switch (resistance_phase) {
      case RESISTANCE_PHASE_OFFSET_CALIBRATION:
        break;
      case RESISTANCE_PHASE_ENABLE_PENDING: {
        const uint16_t neutral = (uint16_t)(TIM1->ARR / 2U);
        if (!power_stage_enable(neutral, neutral, neutral)) {
          test_state = POWER_STAGE_TEST_ABORTED;
          test_kind = TEST_KIND_NONE;
          break;
        }
        current_foc_active = true;
        resistance_phase = RESISTANCE_PHASE_FORWARD_RAMP;
        resistance_phase_started_at = now;
        break;
      }
      case RESISTANCE_PHASE_FORWARD_RAMP:
        current_foc_id_target = RESISTANCE_MEASUREMENT_TARGET_AMPS *
            (float)phase_elapsed / (float)RESISTANCE_RAMP_MS;
        if (phase_elapsed >= RESISTANCE_RAMP_MS) {
          current_foc_id_target = RESISTANCE_MEASUREMENT_TARGET_AMPS;
          resistance_phase = RESISTANCE_PHASE_FORWARD_SETTLE;
          resistance_phase_started_at = now;
        }
        break;
      case RESISTANCE_PHASE_FORWARD_SETTLE:
        current_foc_id_target = RESISTANCE_MEASUREMENT_TARGET_AMPS;
        if (phase_elapsed >= RESISTANCE_SETTLE_MS) {
          resistance_phase = RESISTANCE_PHASE_FORWARD_SAMPLE;
          resistance_phase_started_at = now;
        }
        break;
      case RESISTANCE_PHASE_FORWARD_SAMPLE:
        current_foc_id_target = RESISTANCE_MEASUREMENT_TARGET_AMPS;
        if (phase_elapsed >= RESISTANCE_SAMPLE_MS) {
          current_foc_id_target = 0.0F;
          resistance_phase = RESISTANCE_PHASE_ZERO_SETTLE;
          resistance_phase_started_at = now;
        }
        break;
      case RESISTANCE_PHASE_ZERO_SETTLE:
        current_foc_id_target = 0.0F;
        if (phase_elapsed >= RESISTANCE_ZERO_MS) {
          current_foc_integral_d = 0.0F;
          current_foc_integral_q = 0.0F;
          resistance_phase = RESISTANCE_PHASE_REVERSE_RAMP;
          resistance_phase_started_at = now;
        }
        break;
      case RESISTANCE_PHASE_REVERSE_RAMP:
        current_foc_id_target = -RESISTANCE_MEASUREMENT_TARGET_AMPS *
            (float)phase_elapsed / (float)RESISTANCE_RAMP_MS;
        if (phase_elapsed >= RESISTANCE_RAMP_MS) {
          current_foc_id_target = -RESISTANCE_MEASUREMENT_TARGET_AMPS;
          resistance_phase = RESISTANCE_PHASE_REVERSE_SETTLE;
          resistance_phase_started_at = now;
        }
        break;
      case RESISTANCE_PHASE_REVERSE_SETTLE:
        current_foc_id_target = -RESISTANCE_MEASUREMENT_TARGET_AMPS;
        if (phase_elapsed >= RESISTANCE_SETTLE_MS) {
          resistance_phase = RESISTANCE_PHASE_REVERSE_SAMPLE;
          resistance_phase_started_at = now;
        }
        break;
      case RESISTANCE_PHASE_REVERSE_SAMPLE:
        current_foc_id_target = -RESISTANCE_MEASUREMENT_TARGET_AMPS;
        if (phase_elapsed >= RESISTANCE_SAMPLE_MS) {
          resistance_phase = RESISTANCE_PHASE_RAMP_DOWN;
          resistance_phase_started_at = now;
        }
        break;
      case RESISTANCE_PHASE_RAMP_DOWN:
        current_foc_id_target = -RESISTANCE_MEASUREMENT_TARGET_AMPS *
            (1.0F - (float)phase_elapsed / (float)RESISTANCE_RAMP_DOWN_MS);
        if (phase_elapsed >= RESISTANCE_RAMP_DOWN_MS) {
          current_foc_id_target = 0.0F;
          finish_resistance_measurement();
          resistance_phase = RESISTANCE_PHASE_COMPLETE;
          power_stage_disable();
          test_state = resistance_valid ? POWER_STAGE_TEST_COMPLETED :
                                          POWER_STAGE_TEST_ABORTED;
          test_kind = TEST_KIND_NONE;
        }
        break;
      default:
        break;
      }
    }
  } else if ((test_state == POWER_STAGE_TEST_RUNNING) &&
      (test_kind == TEST_KIND_INDUCTANCE)) {
    if ((uint32_t)(now - test_started_at) >= INDUCTANCE_TIMEOUT_MS) {
      power_stage_disable();
      test_state = POWER_STAGE_TEST_ABORTED;
      test_kind = TEST_KIND_NONE;
    } else if (inductance_phase == INDUCTANCE_PHASE_ENABLE_PENDING) {
      const uint16_t neutral = (uint16_t)(TIM1->ARR / 2U);
      if (!power_stage_enable(neutral, neutral, neutral)) {
        test_state = POWER_STAGE_TEST_ABORTED;
        test_kind = TEST_KIND_NONE;
      } else {
        inductance_phase = INDUCTANCE_PHASE_PULSING;
        inductance_tick = 0U;
        inductance_half_periods = 0U;
        inductance_direction = 1;
        set_inductance_voltage(inductance_direction);
      }
    } else if (inductance_phase == INDUCTANCE_PHASE_COMPLETE) {
      power_stage_disable();
      test_state = inductance_valid ? POWER_STAGE_TEST_COMPLETED :
                                      POWER_STAGE_TEST_ABORTED;
      test_kind = TEST_KIND_NONE;
    } else if ((inductance_phase !=
                INDUCTANCE_PHASE_OFFSET_CALIBRATION) &&
               (stage_state != POWER_STAGE_RUNNING)) {
      power_stage_disable();
      test_state = POWER_STAGE_TEST_ABORTED;
      test_kind = TEST_KIND_NONE;
    }
  } else if ((test_state == POWER_STAGE_TEST_RUNNING) &&
      (test_kind == TEST_KIND_CURRENT_FOC)) {
    const uint32_t elapsed = (uint32_t)(now - test_started_at);
    if ((stage_state != POWER_STAGE_RUNNING) ||
        (elapsed >= CURRENT_FOC_TIMEOUT_MS)) {
      power_stage_disable();
      test_state = POWER_STAGE_TEST_ABORTED;
      test_kind = TEST_KIND_NONE;
    } else if (!test_alignment_calibrated &&
               (elapsed >= ENCODER_ALIGNMENT_ZERO_CAPTURE_MS)) {
      test_alignment_calibrated = angle_sensor_calibrate_electrical_zero(0U);
      if (!test_alignment_calibrated) {
        power_stage_disable();
        test_state = POWER_STAGE_TEST_ABORTED;
        test_kind = TEST_KIND_NONE;
      }
    } else if (!current_foc_active &&
               (elapsed >= ENCODER_VOLTAGE_ALIGN_MS)) {
      angle_sensor_sample_t sample;
      if (!angle_sensor_get_sample(&sample) ||
          !sample.electrical_zero_calibrated ||
          ((uint32_t)(now - sample.timestamp_ms) >
           ENCODER_CONTROL_MAX_SAMPLE_AGE_MS)) {
        ++test_sensor_invalid_count;
        if (test_sensor_invalid_count >= ENCODER_CONTROL_INVALID_LIMIT) {
          power_stage_disable();
          test_state = POWER_STAGE_TEST_ABORTED;
          test_kind = TEST_KIND_NONE;
        }
      } else {
        test_sensor_invalid_count = 0U;
        test_start_position_counts = sample.position_counts;
        current_foc_integral_d = 0.0F;
        current_foc_integral_q = 0.0F;
        current_foc_iq_target = 0.0F;
        current_foc_outer_last_ms = now;
        current_foc_pll_position_counts = (float)sample.position_counts;
        current_foc_pll_speed_counts_per_second = 0.0F;
        current_foc_speed_reference_dps = 0.0F;
        current_foc_target_position_counts = (float)sample.position_counts;
        current_foc_speed_integral_amps = 0.0F;
        test_transform_divider = 0U;
        test_rotation_started_at = now;
        current_foc_active = true;
      }
    } else if (current_foc_active && (test_rotation_started_at != 0U) &&
               ((uint32_t)(now - test_rotation_started_at) >=
                CURRENT_FOC_ACTIVE_MS)) {
      power_stage_disable();
      test_state = POWER_STAGE_TEST_COMPLETED;
      test_kind = TEST_KIND_NONE;
    } else if (current_foc_active) {
      angle_sensor_sample_t sample;
      if (!angle_sensor_get_sample(&sample) ||
          ((uint32_t)(now - sample.timestamp_ms) >
           ENCODER_CONTROL_MAX_SAMPLE_AGE_MS)) {
        ++test_sensor_invalid_count;
        if (test_sensor_invalid_count >= ENCODER_CONTROL_INVALID_LIMIT) {
          power_stage_disable();
          test_state = POWER_STAGE_TEST_ABORTED;
          test_kind = TEST_KIND_NONE;
        }
      } else if ((uint32_t)(now - current_foc_outer_last_ms) >=
                 CURRENT_FOC_OUTER_LOOP_PERIOD_MS) {
        test_sensor_invalid_count = 0U;
        const uint32_t rotation_elapsed =
            (uint32_t)(now - test_rotation_started_at);
        const uint32_t outer_elapsed =
            (uint32_t)(now - current_foc_outer_last_ms);
        const float outer_dt = (float)outer_elapsed * 0.001F;

        /* Encoder PLL: interpolate quantized AS5600 position into a smooth
         * velocity estimate without waiting for a long finite-difference
         * window. Position is the multi-turn count, so no wrap handling is
         * needed here. */
        const float pll_error_counts =
            (float)sample.position_counts - current_foc_pll_position_counts;
        current_foc_pll_position_counts +=
            (current_foc_pll_speed_counts_per_second +
             CURRENT_FOC_PLL_KP_PER_SECOND * pll_error_counts) * outer_dt;
        current_foc_pll_speed_counts_per_second +=
            CURRENT_FOC_PLL_KI_PER_SECOND2 * pll_error_counts * outer_dt;

        /* Acceleration-limited reference. Integrating this reference produces
         * a trajectory that is consistent with the requested speed ramp. */
        const float requested_speed_dps = test_direction > 0 ?
            (float)CURRENT_FOC_TARGET_DEGREES_PER_SECOND :
           -(float)CURRENT_FOC_TARGET_DEGREES_PER_SECOND;
        const float speed_reference_step =
            CURRENT_FOC_ACCELERATION_DPS2 * outer_dt;
        if (current_foc_speed_reference_dps < requested_speed_dps) {
          current_foc_speed_reference_dps += speed_reference_step;
          if (current_foc_speed_reference_dps > requested_speed_dps) {
            current_foc_speed_reference_dps = requested_speed_dps;
          }
        } else if (current_foc_speed_reference_dps > requested_speed_dps) {
          current_foc_speed_reference_dps -= speed_reference_step;
          if (current_foc_speed_reference_dps < requested_speed_dps) {
            current_foc_speed_reference_dps = requested_speed_dps;
          }
        }
        current_foc_target_position_counts +=
            current_foc_speed_reference_dps * (4096.0F / 360.0F) * outer_dt;
        const float position_error_counts =
            current_foc_target_position_counts - (float)sample.position_counts;

        const float commanded_speed_dps =
            current_foc_speed_reference_dps +
            CURRENT_FOC_POSITION_TO_SPEED_GAIN *
                (position_error_counts * 360.0F / 4096.0F);
        float limited_speed_dps = commanded_speed_dps;
        if (limited_speed_dps > CURRENT_FOC_MAX_SPEED_TARGET_DPS) {
          limited_speed_dps = CURRENT_FOC_MAX_SPEED_TARGET_DPS;
        } else if (limited_speed_dps < -CURRENT_FOC_MAX_SPEED_TARGET_DPS) {
          limited_speed_dps = -CURRENT_FOC_MAX_SPEED_TARGET_DPS;
        }
        const float measured_speed_dps =
            current_foc_pll_speed_counts_per_second * (360.0F / 4096.0F);
        const float speed_error_dps =
            limited_speed_dps - measured_speed_dps;
        float maximum_iq = CURRENT_FOC_MAX_TARGET_AMPS;
        if (rotation_elapsed >
            (CURRENT_FOC_ACTIVE_MS - CURRENT_FOC_RAMP_DOWN_MS)) {
          maximum_iq *= (float)(CURRENT_FOC_ACTIVE_MS - rotation_elapsed) /
                        (float)CURRENT_FOC_RAMP_DOWN_MS;
        }
        current_foc_speed_integral_amps +=
            CURRENT_FOC_SPEED_KI_AMPS_PER_DEGREE * speed_error_dps *
            outer_dt;
        if (current_foc_speed_integral_amps > maximum_iq) {
          current_foc_speed_integral_amps = maximum_iq;
        } else if (current_foc_speed_integral_amps < -maximum_iq) {
          current_foc_speed_integral_amps = -maximum_iq;
        }
        /* Smooth Coulomb/cogging feed-forward. Scaling it with the ramped
         * speed reference avoids a torque step at startup while maintaining
         * enough directional torque to cross low-speed detent positions. */
        const float feedforward_ratio = current_foc_speed_reference_dps /
            (float)CURRENT_FOC_TARGET_DEGREES_PER_SECOND;
        const float torque_feedforward =
            CURRENT_FOC_TORQUE_FEEDFORWARD_AMPS * feedforward_ratio;
        float iq_target = torque_feedforward +
            current_foc_speed_integral_amps +
            CURRENT_FOC_SPEED_KP_AMPS_PER_DPS * speed_error_dps;
        if (iq_target > maximum_iq) {
          iq_target = maximum_iq;
        } else if (iq_target < -maximum_iq) {
          iq_target = -maximum_iq;
        }
        current_foc_iq_target = iq_target;
        current_foc_outer_last_ms = now;
      }
    } else if (!current_foc_active) {
      const uint16_t maximum_alignment_amplitude =
          (uint16_t)(TIM1->ARR / 10U);
      uint32_t alignment_amplitude;
      if (elapsed < CURRENT_FOC_ALIGNMENT_RAMP_UP_MS) {
        alignment_amplitude =
            ((uint32_t)maximum_alignment_amplitude * elapsed) /
            CURRENT_FOC_ALIGNMENT_RAMP_UP_MS;
      } else if (elapsed < ENCODER_ALIGNMENT_ZERO_CAPTURE_MS) {
        alignment_amplitude = maximum_alignment_amplitude;
      } else {
        const uint32_t ramp_elapsed =
            elapsed - ENCODER_ALIGNMENT_ZERO_CAPTURE_MS;
        alignment_amplitude = ramp_elapsed >= CURRENT_FOC_ALIGNMENT_RAMP_DOWN_MS ?
            0U : ((uint32_t)maximum_alignment_amplitude *
                  (CURRENT_FOC_ALIGNMENT_RAMP_DOWN_MS - ramp_elapsed)) /
                  CURRENT_FOC_ALIGNMENT_RAMP_DOWN_MS;
      }
      set_voltage_vector(0U, (uint16_t)alignment_amplitude);
    }
  } else if ((test_state == POWER_STAGE_TEST_RUNNING) &&
      ((test_kind == TEST_KIND_ENCODER_VOLTAGE) ||
       (test_kind == TEST_KIND_FLUX))) {
    const uint32_t elapsed = (uint32_t)(now - test_started_at);
    if ((stage_state != POWER_STAGE_RUNNING) ||
        (elapsed >= ENCODER_VOLTAGE_TIMEOUT_MS)) {
      power_stage_disable();
      test_state = POWER_STAGE_TEST_ABORTED;
      test_kind = TEST_KIND_NONE;
    } else if (!test_alignment_calibrated &&
               (elapsed >= ENCODER_ALIGNMENT_ZERO_CAPTURE_MS)) {
      test_alignment_calibrated = angle_sensor_calibrate_electrical_zero(0U);
      if (!test_alignment_calibrated) {
        power_stage_disable();
        test_state = POWER_STAGE_TEST_ABORTED;
        test_kind = TEST_KIND_NONE;
      }
    } else if ((test_rotation_started_at == 0U) &&
               (elapsed >= ENCODER_VOLTAGE_ALIGN_MS)) {
      angle_sensor_sample_t sample;
      if (!angle_sensor_get_sample(&sample) ||
          !sample.electrical_zero_calibrated ||
          ((uint32_t)(now - sample.timestamp_ms) >
           ENCODER_CONTROL_MAX_SAMPLE_AGE_MS)) {
        ++test_sensor_invalid_count;
        if (test_sensor_invalid_count >= ENCODER_CONTROL_INVALID_LIMIT) {
          power_stage_disable();
          test_state = POWER_STAGE_TEST_ABORTED;
          test_kind = TEST_KIND_NONE;
        }
      } else {
        test_sensor_invalid_count = 0U;
        test_start_position_counts = sample.position_counts;
        test_rotation_started_at = now;
      }
    } else if (test_rotation_started_at != 0U) {
      const uint32_t rotation_elapsed =
          (uint32_t)(now - test_rotation_started_at);
      if (rotation_elapsed >= ENCODER_VOLTAGE_ROTATE_MS) {
        power_stage_disable();
        if (test_kind == TEST_KIND_FLUX) {
          finish_flux_measurement();
        }
        test_state = (test_kind != TEST_KIND_FLUX || flux_valid) ?
            POWER_STAGE_TEST_COMPLETED : POWER_STAGE_TEST_ABORTED;
        test_kind = TEST_KIND_NONE;
      } else {
        angle_sensor_sample_t sample;
        if (!angle_sensor_get_sample(&sample) ||
            !sample.electrical_zero_calibrated ||
            ((uint32_t)(now - sample.timestamp_ms) >
             ENCODER_CONTROL_MAX_SAMPLE_AGE_MS)) {
          ++test_sensor_invalid_count;
          if (test_sensor_invalid_count >= ENCODER_CONTROL_INVALID_LIMIT) {
            power_stage_disable();
            test_state = POWER_STAGE_TEST_ABORTED;
            test_kind = TEST_KIND_NONE;
          }
        } else {
          test_sensor_invalid_count = 0U;
          const uint32_t target_dps = test_kind == TEST_KIND_FLUX ? 360U :
              ENCODER_VOLTAGE_TARGET_DEGREES_PER_SECOND;
          const int32_t target_advance_counts = (int32_t)(
              ((uint64_t)rotation_elapsed * 4096U *
               target_dps) / 360000U);
          const int32_t target_counts = test_start_position_counts +
              (test_direction > 0 ? target_advance_counts :
                                    -target_advance_counts);
          const int32_t error_counts = target_counts - sample.position_counts;
          const uint32_t absolute_error = (uint32_t)(
              error_counts < 0 ? -error_counts : error_counts);
          uint32_t maximum_amplitude = test_kind == TEST_KIND_FLUX ?
              TIM1->ARR / 3U : TIM1->ARR / 12U;
          if (rotation_elapsed >
              (ENCODER_VOLTAGE_ROTATE_MS - ENCODER_VOLTAGE_RAMP_DOWN_MS)) {
            maximum_amplitude = (maximum_amplitude *
                (ENCODER_VOLTAGE_ROTATE_MS - rotation_elapsed)) /
                ENCODER_VOLTAGE_RAMP_DOWN_MS;
          }
          uint32_t amplitude =
              (absolute_error * maximum_amplitude) /
              ENCODER_VOLTAGE_FULL_OUTPUT_ERROR_COUNTS;
          if (amplitude > maximum_amplitude) {
            amplitude = maximum_amplitude;
          }
          if (error_counts > 0) {
            test_command_electrical_raw =
                (uint16_t)((sample.electrical_raw + 1024U) & 0x0FFFU);
          } else if (error_counts < 0) {
            test_command_electrical_raw =
                (uint16_t)((sample.electrical_raw + 3072U) & 0x0FFFU);
          } else {
            test_command_electrical_raw = sample.electrical_raw;
          }
          set_voltage_vector(test_command_electrical_raw,
                             (uint16_t)amplitude);
          if (test_kind == TEST_KIND_FLUX) {
            const uint32_t outer_elapsed = current_foc_outer_last_ms == 0U ?
                10U : (uint32_t)(now - current_foc_outer_last_ms);
            if (outer_elapsed >= 5U) {
              const float outer_dt = (float)outer_elapsed * 0.001F;
              const float pll_error = (float)sample.position_counts -
                  current_foc_pll_position_counts;
              current_foc_pll_position_counts +=
                  (current_foc_pll_speed_counts_per_second +
                   CURRENT_FOC_PLL_KP_PER_SECOND * pll_error) * outer_dt;
              current_foc_pll_speed_counts_per_second +=
                  CURRENT_FOC_PLL_KI_PER_SECOND2 * pll_error * outer_dt;
              current_foc_outer_last_ms = now;
            }
            flux_vq_mv = (int32_t)((int64_t)(error_counts >= 0 ?
                (int32_t)amplitude : -(int32_t)amplitude) * bus_voltage_mv /
                TIM1->ARR);
          }
        }
      }
    }
  } else if ((test_state == POWER_STAGE_TEST_RUNNING) &&
      (test_kind == TEST_KIND_ENCODER_ALIGNMENT)) {
    if ((stage_state != POWER_STAGE_RUNNING) ||
        ((uint32_t)(now - test_started_at) >= ENCODER_ALIGNMENT_TIMEOUT_MS)) {
      power_stage_disable();
      test_state = POWER_STAGE_TEST_ABORTED;
      test_kind = TEST_KIND_NONE;
    } else if ((uint32_t)(now - test_started_at) >=
               ENCODER_ALIGNMENT_HOLD_MS) {
      power_stage_disable();
      test_state = test_alignment_calibrated ? POWER_STAGE_TEST_COMPLETED :
                                               POWER_STAGE_TEST_ABORTED;
      test_kind = TEST_KIND_NONE;
    } else if (test_alignment_calibrated &&
               !test_alignment_quadrature_started &&
               ((uint32_t)(now - test_started_at) >=
                ENCODER_ALIGNMENT_QUADRATURE_START_MS)) {
      const uint16_t neutral = (uint16_t)(TIM1->ARR / 2U);
      const uint16_t delta = (uint16_t)(TIM1->ARR / 10U);
      const uint16_t quadrature_delta =
          (uint16_t)(((uint32_t)delta * 866U) / 1000U);
      set_compare_values(neutral,
                         (uint16_t)(neutral + quadrature_delta),
                         (uint16_t)(neutral - quadrature_delta));
      test_alignment_quadrature_started = true;
    } else if (!test_alignment_calibrated &&
               ((uint32_t)(now - test_started_at) >=
                ENCODER_ALIGNMENT_ZERO_CAPTURE_MS)) {
      test_alignment_calibrated = angle_sensor_calibrate_electrical_zero(
          0U);
      if (!test_alignment_calibrated) {
        power_stage_disable();
        test_state = POWER_STAGE_TEST_ABORTED;
        test_kind = TEST_KIND_NONE;
      }
    }
  } else if (test_state == POWER_STAGE_TEST_RUNNING) {
    if ((stage_state != POWER_STAGE_RUNNING) ||
        ((uint32_t)(now - test_started_at) >= COMMISSIONING_TEST_TIMEOUT_MS)) {
      power_stage_disable();
      test_state = POWER_STAGE_TEST_ABORTED;
      test_kind = TEST_KIND_NONE;
    } else if (((uint32_t)(now - test_started_at) >=
                COMMISSIONING_ALIGNMENT_MS) &&
               (test_steps_completed < COMMISSIONING_STEP_COUNT) &&
               ((uint32_t)(now - test_last_step_at) >=
                COMMISSIONING_STEP_PERIOD_MS)) {
      test_last_step_at = now;
      test_step = (uint8_t)((test_step +
          (test_direction > 0 ? 1U : 5U)) % 6U);
      set_commutation_step(test_step);
      ++test_steps_completed;
      if (test_steps_completed == COMMISSIONING_STEP_COUNT) {
        test_settle_started_at = now;
      }
    } else if ((test_steps_completed == COMMISSIONING_STEP_COUNT) &&
               ((uint32_t)(now - test_settle_started_at) >=
                COMMISSIONING_SETTLE_MS)) {
      power_stage_disable();
      test_state = POWER_STAGE_TEST_COMPLETED;
      test_kind = TEST_KIND_NONE;
    }
  }
}

bool power_stage_enable(uint16_t duty_a, uint16_t duty_b, uint16_t duty_c)
{
  if ((stage_state != POWER_STAGE_READY) ||
      (HAL_GPIO_ReadPin(DRV_FAULT_N_GPIO_Port, DRV_FAULT_N_Pin) ==
       GPIO_PIN_RESET)) {
    return false;
  }

  set_compare_values(duty_a, duty_b, duty_c);
  HAL_GPIO_WritePin(DRV_EN_GATE_GPIO_Port, DRV_EN_GATE_Pin, GPIO_PIN_SET);
  /* DRV8301 holds nFAULT low while its supplies and charge pump wake up. */
  HAL_Delay(100U);

  if (HAL_GPIO_ReadPin(DRV_FAULT_N_GPIO_Port, DRV_FAULT_N_Pin) ==
      GPIO_PIN_RESET) {
    uint16_t faults;
    if (drv8301_read_faults(&faults) == DRV8301_RESULT_OK) {
      latched_faults |= faults;
    } else {
      fault_status_pending = true;
    }
    power_stage_disable();
    stage_state = POWER_STAGE_FAULT;
    return false;
  }

  TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE |
                TIM_CCER_CC2E | TIM_CCER_CC2NE |
                TIM_CCER_CC3E | TIM_CCER_CC3NE;
  TIM1->BDTR |= TIM_BDTR_MOE;
  stage_state = POWER_STAGE_RUNNING;
  return true;
}

bool power_stage_set_duty(uint16_t duty_a, uint16_t duty_b, uint16_t duty_c)
{
  if (stage_state != POWER_STAGE_RUNNING) {
    return false;
  }
  set_compare_values(duty_a, duty_b, duty_c);
  return true;
}

bool power_stage_start_commissioning_test(int8_t direction)
{
  const uint16_t neutral = (uint16_t)(TIM1->ARR / 2U);

  if ((direction != 1) && (direction != -1)) {
    return false;
  }
  if ((stage_state != POWER_STAGE_READY) || !bus_voltage_valid ||
      (bus_voltage_mv < COMMISSIONING_MIN_BUS_MV) ||
      (bus_voltage_mv > COMMISSIONING_MAX_BUS_MV) ||
      (latched_faults != 0U)) {
    return false;
  }

  test_direction = direction;
  test_kind = TEST_KIND_COMMUTATION;
  test_step = 0U;
  test_steps_completed = 0U;
  test_settle_started_at = 0U;
  test_started_at = HAL_GetTick();
  test_last_step_at = test_started_at + COMMISSIONING_ALIGNMENT_MS;
  overcurrent_count = 0U;
  if (!power_stage_enable(neutral, neutral, neutral)) {
    test_state = POWER_STAGE_TEST_ABORTED;
    return false;
  }
  set_commutation_step(test_step);
  test_state = POWER_STAGE_TEST_RUNNING;
  return true;
}

bool power_stage_start_encoder_alignment(void)
{
  const uint16_t neutral = (uint16_t)(TIM1->ARR / 2U);
  const uint16_t delta = (uint16_t)(TIM1->ARR / 10U);

  if ((stage_state != POWER_STAGE_READY) || !bus_voltage_valid ||
      (bus_voltage_mv < COMMISSIONING_MIN_BUS_MV) ||
      (bus_voltage_mv > COMMISSIONING_MAX_BUS_MV) ||
      (latched_faults != 0U)) {
    return false;
  }

  test_steps_completed = 0U;
  overcurrent_count = 0U;
  test_started_at = HAL_GetTick();
  test_kind = TEST_KIND_ENCODER_ALIGNMENT;
  test_alignment_calibrated = false;
  test_alignment_quadrature_started = false;
  /* Initial 0-degree vector: cos(0), cos(-120), cos(120). */
  if (!power_stage_enable((uint16_t)(neutral + delta),
                          (uint16_t)(neutral - delta / 2U),
                          (uint16_t)(neutral - delta / 2U))) {
    test_state = POWER_STAGE_TEST_ABORTED;
    test_kind = TEST_KIND_NONE;
    return false;
  }
  test_state = POWER_STAGE_TEST_RUNNING;
  return true;
}

bool power_stage_start_encoder_voltage_test(int8_t direction)
{
  const uint16_t neutral = (uint16_t)(TIM1->ARR / 2U);
  const uint16_t delta = (uint16_t)(TIM1->ARR / 10U);

  if (((direction != 1) && (direction != -1)) ||
      (stage_state != POWER_STAGE_READY) || !bus_voltage_valid ||
      (bus_voltage_mv < COMMISSIONING_MIN_BUS_MV) ||
      (bus_voltage_mv > COMMISSIONING_MAX_BUS_MV) ||
      (latched_faults != 0U)) {
    return false;
  }
  test_direction = direction;
  test_steps_completed = 0U;
  overcurrent_count = 0U;
  test_started_at = HAL_GetTick();
  test_rotation_started_at = 0U;
  test_command_electrical_raw = 0U;
  test_start_position_counts = 0;
  test_sensor_invalid_count = 0U;
  reset_test_current_statistics();
  test_kind = TEST_KIND_ENCODER_VOLTAGE;
  test_alignment_calibrated = false;
  if (!power_stage_enable((uint16_t)(neutral + delta),
                          (uint16_t)(neutral - delta / 2U),
                          (uint16_t)(neutral - delta / 2U))) {
    test_state = POWER_STAGE_TEST_ABORTED;
    test_kind = TEST_KIND_NONE;
    return false;
  }
  test_state = POWER_STAGE_TEST_RUNNING;
  return true;
}

bool power_stage_start_current_foc_test(int8_t direction)
{
  const uint16_t neutral = (uint16_t)(TIM1->ARR / 2U);
  angle_sensor_sample_t sample;

  if (((direction != 1) && (direction != -1)) ||
      (stage_state != POWER_STAGE_READY) || !bus_voltage_valid ||
      (bus_voltage_mv < COMMISSIONING_MIN_BUS_MV) ||
      (bus_voltage_mv > COMMISSIONING_MAX_BUS_MV) ||
      (latched_faults != 0U)) {
    return false;
  }

  test_direction = direction;
  test_steps_completed = 0U;
  overcurrent_count = 0U;
  test_started_at = HAL_GetTick();
  test_rotation_started_at = 0U;
  test_sensor_invalid_count = 0U;
  reset_test_current_statistics();
  current_foc_active = false;
  test_kind = TEST_KIND_CURRENT_FOC;
  test_alignment_calibrated = angle_sensor_get_sample(&sample) &&
      sample.electrical_zero_calibrated &&
      ((uint32_t)(test_started_at - sample.timestamp_ms) <=
       ENCODER_CONTROL_MAX_SAMPLE_AGE_MS);
  if (!power_stage_enable(neutral, neutral, neutral)) {
    test_state = POWER_STAGE_TEST_ABORTED;
    test_kind = TEST_KIND_NONE;
    return false;
  }
  if (test_alignment_calibrated) {
    test_start_position_counts = sample.position_counts;
    test_rotation_started_at = HAL_GetTick();
    current_foc_outer_last_ms = test_rotation_started_at;
    current_foc_pll_position_counts = (float)sample.position_counts;
    current_foc_pll_speed_counts_per_second = 0.0F;
    current_foc_speed_reference_dps = 0.0F;
    current_foc_target_position_counts = (float)sample.position_counts;
    current_foc_speed_integral_amps = 0.0F;
    current_foc_active = true;
  }
  test_state = POWER_STAGE_TEST_RUNNING;
  return true;
}

bool power_stage_start_resistance_measurement(void)
{
  if ((stage_state != POWER_STAGE_READY) || !bus_voltage_valid ||
      (bus_voltage_mv < COMMISSIONING_MIN_BUS_MV) ||
      (bus_voltage_mv > COMMISSIONING_MAX_BUS_MV) ||
      (latched_faults != 0U)) {
    return false;
  }

  overcurrent_count = 0U;
  test_started_at = HAL_GetTick();
  reset_test_current_statistics();
  current_foc_integral_d = 0.0F;
  current_foc_integral_q = 0.0F;
  current_foc_id_target = 0.0F;
  current_foc_iq_target = 0.0F;
  current_foc_active = false;
  test_kind = TEST_KIND_RESISTANCE;
  resistance_phase = RESISTANCE_PHASE_OFFSET_CALIBRATION;
  resistance_phase_started_at = test_started_at;
  calibration_count = 0U;
  for (uint32_t phase = 0U; phase < 3U; ++phase) {
    calibration_sum[phase] = 0U;
  }
  test_state = POWER_STAGE_TEST_RUNNING;
  return true;
}

bool power_stage_start_inductance_measurement(void)
{
  if ((stage_state != POWER_STAGE_READY) || !bus_voltage_valid ||
      (bus_voltage_mv < COMMISSIONING_MIN_BUS_MV) ||
      (bus_voltage_mv > COMMISSIONING_MAX_BUS_MV) ||
      (latched_faults != 0U) || !resistance_valid) {
    return false;
  }

  reset_inductance_statistics();
  overcurrent_count = 0U;
  test_started_at = HAL_GetTick();
  test_kind = TEST_KIND_INDUCTANCE;
  inductance_phase = INDUCTANCE_PHASE_OFFSET_CALIBRATION;
  calibration_count = 0U;
  for (uint32_t phase = 0U; phase < 3U; ++phase) {
    calibration_sum[phase] = 0U;
  }
  test_state = POWER_STAGE_TEST_RUNNING;
  return true;
}

bool power_stage_start_flux_measurement(int8_t direction)
{
  const int32_t saved_resistance = resistance_average_milliohms;
  const bool saved_resistance_valid = resistance_valid;
  if (!saved_resistance_valid ||
      !power_stage_start_encoder_voltage_test(direction)) {
    return false;
  }
  /* The generic test reset clears prior diagnostics; retain the prerequisite Rs. */
  resistance_average_milliohms = saved_resistance;
  resistance_valid = true;
  reset_flux_statistics();
  angle_sensor_sample_t sample;
  if (angle_sensor_get_sample(&sample)) {
    current_foc_pll_position_counts = (float)sample.position_counts;
  }
  current_foc_pll_speed_counts_per_second = 0.0F;
  current_foc_outer_last_ms = HAL_GetTick();
  test_kind = TEST_KIND_FLUX;
  return true;
}

void power_stage_stop_commissioning_test(void)
{
  if (test_state == POWER_STAGE_TEST_RUNNING) {
    power_stage_disable();
    test_state = POWER_STAGE_TEST_ABORTED;
    test_kind = TEST_KIND_NONE;
  }
}

power_stage_test_state_t power_stage_get_test_state(void)
{
  return test_state;
}

uint8_t power_stage_get_test_steps_completed(void)
{
  return test_steps_completed;
}

power_stage_state_t power_stage_get_state(void)
{
  return stage_state;
}

bool power_stage_get_current_sample(power_stage_current_sample_t *sample)
{
  uint32_t sequence_before;
  uint32_t sequence_after;

  if (sample == NULL) {
    return false;
  }

  do {
    sequence_before = current_sequence;
    if ((sequence_before & 1U) != 0U) {
      continue;
    }
    for (uint32_t index = 0U; index < 3U; ++index) {
      sample->raw[index] = current_raw[index];
      sample->offset[index] = current_offset[index];
      sample->centered[index] =
          (int32_t)sample->raw[index] - (int32_t)sample->offset[index];
    }
    sequence_after = current_sequence;
  } while ((sequence_before != sequence_after) ||
           ((sequence_after & 1U) != 0U));

  sample->sequence = sequence_after >> 1U;
  return true;
}

uint16_t power_stage_get_latched_faults(void)
{
  return latched_faults;
}

bool power_stage_get_diagnostics(power_stage_diagnostics_t *diagnostics)
{
  if (diagnostics == NULL) {
    return false;
  }

  diagnostics->state = stage_state;
  diagnostics->drv_faults = latched_faults;
  diagnostics->bus_voltage_raw = bus_voltage_raw;
  diagnostics->bus_voltage_mv = bus_voltage_mv;
  diagnostics->gate_enabled =
      HAL_GPIO_ReadPin(DRV_EN_GATE_GPIO_Port, DRV_EN_GATE_Pin) == GPIO_PIN_SET;
  diagnostics->pwm_outputs_enabled =
      (TIM1->CCER & (TIM_CCER_CC1E | TIM_CCER_CC1NE |
                     TIM_CCER_CC2E | TIM_CCER_CC2NE |
                     TIM_CCER_CC3E | TIM_CCER_CC3NE)) != 0U;
  diagnostics->fault_pin_active =
      HAL_GPIO_ReadPin(DRV_FAULT_N_GPIO_Port, DRV_FAULT_N_Pin) ==
      GPIO_PIN_RESET;
  diagnostics->bus_voltage_valid = bus_voltage_valid;
  diagnostics->test_current_samples = test_current_samples;
  diagnostics->test_current_balance_abs_sum = test_current_balance_abs_sum;
  diagnostics->test_current_balance_abs_max = test_current_balance_abs_max;
  diagnostics->test_v0_samples = test_v0_samples;
  diagnostics->test_v7_samples = test_v7_samples;
  diagnostics->test_transform_samples = test_transform_samples;
  diagnostics->test_id_sum_ma = test_id_sum_ma;
  diagnostics->test_iq_sum_ma = test_iq_sum_ma;
  diagnostics->test_id_min_ma = test_id_min_ma;
  diagnostics->test_id_max_ma = test_id_max_ma;
  diagnostics->test_iq_min_ma = test_iq_min_ma;
  diagnostics->test_iq_max_ma = test_iq_max_ma;
  diagnostics->test_iq_target_sum_ma = test_iq_target_sum_ma;
  diagnostics->test_iq_target_min_ma = test_iq_target_min_ma;
  diagnostics->test_iq_target_max_ma = test_iq_target_max_ma;
  diagnostics->test_voltage_saturated_samples = test_voltage_saturated_samples;
  diagnostics->test_integral_d_saturated_samples =
      test_integral_d_saturated_samples;
  diagnostics->test_integral_q_saturated_samples =
      test_integral_q_saturated_samples;
  diagnostics->test_voltage_request_sum_counts =
      test_voltage_request_sum_counts;
  diagnostics->test_voltage_request_max_counts =
      test_voltage_request_max_counts;
  diagnostics->resistance_measurement_samples =
      resistance_measurement_samples;
  diagnostics->resistance_iq_sum_ma = resistance_iq_sum_ma;
  diagnostics->resistance_vq_sum_mv = resistance_vq_sum_mv;
  diagnostics->resistance_phase = (uint8_t)resistance_phase;
  diagnostics->resistance_valid = resistance_valid ? 1U : 0U;
  diagnostics->resistance_forward_samples = resistance_forward_samples;
  diagnostics->resistance_reverse_samples = resistance_reverse_samples;
  diagnostics->resistance_id_ma = resistance_id_ma;
  diagnostics->resistance_iq_ma = resistance_iq_ma;
  diagnostics->resistance_vd_mv = resistance_vd_mv;
  diagnostics->resistance_vq_mv = resistance_vq_mv;
  diagnostics->resistance_live_milliohms = resistance_live_milliohms;
  diagnostics->resistance_forward_milliohms =
      resistance_forward_milliohms;
  diagnostics->resistance_reverse_milliohms =
      resistance_reverse_milliohms;
  diagnostics->resistance_average_milliohms =
      resistance_average_milliohms;
  diagnostics->inductance_phase = (uint8_t)inductance_phase;
  diagnostics->inductance_valid = inductance_valid ? 1U : 0U;
  diagnostics->inductance_forward_samples = inductance_forward_samples;
  diagnostics->inductance_reverse_samples = inductance_reverse_samples;
  diagnostics->inductance_delta_current_ma = inductance_delta_current_ma;
  diagnostics->inductance_voltage_mv = inductance_voltage_mv;
  diagnostics->inductance_forward_uh = inductance_forward_uh;
  diagnostics->inductance_reverse_uh = inductance_reverse_uh;
  diagnostics->inductance_average_uh = inductance_average_uh;
  diagnostics->flux_valid = flux_valid ? 1U : 0U;
  diagnostics->flux_samples = flux_samples;
  diagnostics->flux_speed_millidegrees_per_second = flux_speed_mdps;
  diagnostics->flux_iq_ma = flux_iq_ma;
  diagnostics->flux_vq_mv = flux_vq_mv;
  diagnostics->flux_linkage_uwb = flux_linkage_uwb;
  diagnostics->back_emf_constant_uv_per_rad_s = flux_ke_uv_per_rad_s;
  diagnostics->kv_millirpm_per_volt = flux_kv_millirpm_per_volt;
  for (uint32_t phase = 0U; phase < 3U; ++phase) {
    diagnostics->test_current_sum[phase] = test_current_sum[phase];
    diagnostics->test_current_min[phase] = test_current_min[phase];
    diagnostics->test_current_max[phase] = test_current_max[phase];
    diagnostics->test_reconstructed_samples[phase] =
        test_reconstructed_samples[phase];
  }
  return power_stage_get_current_sample(&diagnostics->current);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance != ADC1) {
    return;
  }

  const uint16_t samples[3] = {
      (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1),
      (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1),
      (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1)
  };

  ++current_sequence;
  for (uint32_t index = 0U; index < 3U; ++index) {
    current_raw[index] = samples[index];
  }
  ++current_sequence;

  if ((test_state == POWER_STAGE_TEST_RUNNING) &&
      (((test_kind == TEST_KIND_RESISTANCE) &&
        (resistance_phase == RESISTANCE_PHASE_OFFSET_CALIBRATION)) ||
       ((test_kind == TEST_KIND_INDUCTANCE) &&
        (inductance_phase == INDUCTANCE_PHASE_OFFSET_CALIBRATION)))) {
    for (uint32_t index = 0U; index < 3U; ++index) {
      calibration_sum[index] += samples[index];
    }
    ++calibration_count;
    if (calibration_count >= CURRENT_CALIBRATION_SAMPLES) {
      for (uint32_t index = 0U; index < 3U; ++index) {
        current_offset[index] = (uint16_t)(
            calibration_sum[index] / CURRENT_CALIBRATION_SAMPLES);
      }
      if (test_kind == TEST_KIND_RESISTANCE) {
        resistance_phase = RESISTANCE_PHASE_ENABLE_PENDING;
        resistance_phase_started_at = HAL_GetTick();
      } else {
        inductance_phase = INDUCTANCE_PHASE_ENABLE_PENDING;
      }
    }
  } else if (stage_state == POWER_STAGE_CALIBRATING) {
    for (uint32_t index = 0U; index < 3U; ++index) {
      calibration_sum[index] += samples[index];
    }
    ++calibration_count;
    if (calibration_count >= CURRENT_CALIBRATION_SAMPLES) {
      for (uint32_t index = 0U; index < 3U; ++index) {
        current_offset[index] =
            (uint16_t)(calibration_sum[index] /
                       CURRENT_CALIBRATION_SAMPLES);
      }
      stage_state = POWER_STAGE_READY;
    }
  } else if (stage_state == POWER_STAGE_RUNNING) {
    bool overcurrent = false;
    const int32_t current_trip_counts =
        test_kind == TEST_KIND_INDUCTANCE ? 40 :
        (int32_t)CURRENT_TRIP_ADC_COUNTS;
    int32_t balance = 0;
    int32_t centered_samples[3];
    for (uint32_t index = 0U; index < 3U; ++index) {
      const int32_t centered =
          (int32_t)samples[index] - (int32_t)current_offset[index];
      centered_samples[index] = centered;
      balance += centered;
      test_current_sum[index] += centered;
      if (centered < test_current_min[index]) {
        test_current_min[index] = (int16_t)centered;
      }
      if (centered > test_current_max[index]) {
        test_current_max[index] = (int16_t)centered;
      }
      if ((centered > current_trip_counts) ||
          (centered < -current_trip_counts)) {
        overcurrent = true;
      }
    }
    ++test_current_samples;
    const uint16_t balance_abs = (uint16_t)(balance < 0 ? -balance : balance);
    test_current_balance_abs_sum += balance_abs;
    if (balance_abs > test_current_balance_abs_max) {
      test_current_balance_abs_max = balance_abs;
    }

    const uint8_t transform_divider_limit =
        ((((test_kind == TEST_KIND_CURRENT_FOC) ||
           (test_kind == TEST_KIND_RESISTANCE)) && current_foc_active) ||
         (test_kind == TEST_KIND_FLUX) ||
         (test_kind == TEST_KIND_INDUCTANCE)) ?
        1U : CURRENT_TRANSFORM_DIAGNOSTIC_DIVIDER;
    if ((((test_kind == TEST_KIND_ENCODER_VOLTAGE) &&
          (test_rotation_started_at != 0U)) ||
         (((test_kind == TEST_KIND_CURRENT_FOC) ||
           (test_kind == TEST_KIND_RESISTANCE)) && current_foc_active) ||
         (test_kind == TEST_KIND_FLUX) ||
         (test_kind == TEST_KIND_INDUCTANCE)) &&
        (++test_transform_divider >= transform_divider_limit)) {
      test_transform_divider = 0U;
      /*
       * The resistance vector is stationary and ADC sampling is near ARR.
       * With low-side shunts, the phase with the largest compare is the one
       * that can be unobservable at that instant, on both counter directions.
       */
      const bool is_v7 = (test_kind != TEST_KIND_RESISTANCE) &&
          (test_kind != TEST_KIND_INDUCTANCE) &&
          ((TIM1->CR1 & TIM_CR1_DIR) == 0U);
      uint32_t reconstructed_phase = 3U;
      int32_t corrected[3] = {
          centered_samples[0], centered_samples[1], centered_samples[2]};

      if (is_v7) {
        ++test_v7_samples;
        if ((TIM1->CCR1 < TIM1->CCR2) && (TIM1->CCR1 < TIM1->CCR3)) {
          reconstructed_phase = 0U;
        } else if ((TIM1->CCR2 < TIM1->CCR1) &&
                   (TIM1->CCR2 < TIM1->CCR3)) {
          reconstructed_phase = 1U;
        } else if ((TIM1->CCR3 < TIM1->CCR1) &&
                   (TIM1->CCR3 < TIM1->CCR2)) {
          reconstructed_phase = 2U;
        }
      } else {
        ++test_v0_samples;
        if ((TIM1->CCR1 > TIM1->CCR2) && (TIM1->CCR1 > TIM1->CCR3)) {
          reconstructed_phase = 0U;
        } else if ((TIM1->CCR2 > TIM1->CCR1) &&
                   (TIM1->CCR2 > TIM1->CCR3)) {
          reconstructed_phase = 1U;
        } else if ((TIM1->CCR3 > TIM1->CCR1) &&
                   (TIM1->CCR3 > TIM1->CCR2)) {
          reconstructed_phase = 2U;
        }
      }

      if (reconstructed_phase < 3U) {
        const uint32_t other_1 = (reconstructed_phase + 1U) % 3U;
        const uint32_t other_2 = (reconstructed_phase + 2U) % 3U;
        corrected[reconstructed_phase] =
            -(corrected[other_1] + corrected[other_2]);
        ++test_reconstructed_samples[reconstructed_phase];
      }

      uint16_t electrical_raw;
      const bool fixed_identification_angle =
          (test_kind == TEST_KIND_RESISTANCE) ||
          (test_kind == TEST_KIND_INDUCTANCE);
      const bool angle_valid = fixed_identification_angle ?
          ((electrical_raw = 0U), true) :
          angle_sensor_get_electrical_raw_fast(&electrical_raw);
      if (angle_valid) {
        const bool stationary_identification =
            (test_kind == TEST_KIND_RESISTANCE) ||
            (test_kind == TEST_KIND_INDUCTANCE);
        const bool inline_identification = stationary_identification ||
            (test_kind == TEST_KIND_FLUX);
        const float current_scale = inline_identification ?
            -CURRENT_ADC_TO_AMPS : CURRENT_ADC_TO_AMPS;
        const int32_t *transform_samples = inline_identification ?
            centered_samples : corrected;
        const float ia = (float)transform_samples[0] * current_scale;
        const float ib = (float)transform_samples[1] * current_scale;
        const float ic = (float)transform_samples[2] * current_scale;
        /* Full Clarke rejects ADC common-mode error on the three inline shunts. */
        const float i_alpha = inline_identification ?
            (0.666666667F * ia - 0.333333333F * ib - 0.333333333F * ic) :
            ia;
        const float i_beta = inline_identification ?
            (0.577350269F * (ib - ic)) :
            (0.577350269F * ia + 1.154700538F * ib);
        const float sine = fast_sine_raw(electrical_raw);
        const float cosine = fast_sine_raw((uint16_t)(electrical_raw + 1024U));
        const float id = cosine * i_alpha + sine * i_beta;
        const float iq = cosine * i_beta - sine * i_alpha;
        const int32_t id_ma = (int32_t)(1000.0F * id);
        const int32_t iq_ma = (int32_t)(1000.0F * iq);
        ++test_transform_samples;
        test_id_sum_ma += id_ma;
        test_iq_sum_ma += iq_ma;
        if (id_ma < test_id_min_ma) { test_id_min_ma = id_ma; }
        if (id_ma > test_id_max_ma) { test_id_max_ma = id_ma; }
        if (iq_ma < test_iq_min_ma) { test_iq_min_ma = iq_ma; }
        if (iq_ma > test_iq_max_ma) { test_iq_max_ma = iq_ma; }

        if ((test_kind == TEST_KIND_INDUCTANCE) &&
            (inductance_phase == INDUCTANCE_PHASE_PULSING)) {
          if (inductance_tick >= INDUCTANCE_HALF_PERIOD_TICKS -
                                 INDUCTANCE_ENDPOINT_AVERAGE_TICKS) {
            inductance_baseline_sum_ma += id_ma;
            ++inductance_baseline_samples;
          }
          if (inductance_tick >= INDUCTANCE_HALF_PERIOD_TICKS - 1U) {
            const int32_t endpoint_ma = inductance_baseline_samples == 0U ? 0 :
                (int32_t)(inductance_baseline_sum_ma /
                          inductance_baseline_samples);
            const int32_t normalized_current_ma =
                endpoint_ma * inductance_direction;
            const int32_t voltage_mv = (int32_t)(
                ((uint64_t)(TIM1->ARR / 6U) * bus_voltage_mv) / TIM1->ARR);
            inductance_delta_current_ma = normalized_current_ma;
            inductance_voltage_mv = voltage_mv;
            if ((inductance_half_periods >=
                 INDUCTANCE_SETTLE_HALF_PERIODS) &&
                (normalized_current_ma > 20)) {
              if (inductance_direction > 0) {
                ++inductance_forward_samples;
                inductance_forward_delta_sum_ma += normalized_current_ma;
                inductance_forward_voltage_sum_mv += voltage_mv;
              } else {
                ++inductance_reverse_samples;
                inductance_reverse_delta_sum_ma += normalized_current_ma;
                inductance_reverse_voltage_sum_mv += voltage_mv;
              }
            }
            if ((inductance_forward_samples >=
                 INDUCTANCE_TARGET_SAMPLES_PER_DIRECTION) &&
                (inductance_reverse_samples >=
                 INDUCTANCE_TARGET_SAMPLES_PER_DIRECTION)) {
              finish_inductance_measurement();
              inductance_phase = INDUCTANCE_PHASE_COMPLETE;
              set_inductance_voltage(0);
            } else {
              inductance_tick = 0U;
              ++inductance_half_periods;
              inductance_direction = (int8_t)-inductance_direction;
              inductance_baseline_sum_ma = 0;
              inductance_baseline_samples = 0U;
              set_inductance_voltage(inductance_direction);
            }
          } else {
            ++inductance_tick;
          }
        }

        if (test_kind == TEST_KIND_FLUX) {
          const int32_t speed_mdps = (int32_t)(
              current_foc_pll_speed_counts_per_second *
              (360000.0F / 4096.0F));
          flux_speed_mdps = speed_mdps;
          flux_iq_ma = iq_ma;
          const uint32_t rotation_elapsed =
              (uint32_t)(HAL_GetTick() - test_rotation_started_at);
          const bool speed_valid = test_direction > 0 ?
              (speed_mdps > 300000 && speed_mdps < 420000) :
              (speed_mdps < -300000 && speed_mdps > -420000);
          if ((rotation_elapsed >= FLUX_SAMPLE_START_MS) &&
              (rotation_elapsed < FLUX_SAMPLE_END_MS) && speed_valid) {
            ++flux_samples;
            flux_speed_sum_mdps += speed_mdps;
            flux_iq_sum_ma += iq_ma;
            flux_vq_sum_mv += flux_vq_mv;
          }
        }

        if (((test_kind == TEST_KIND_CURRENT_FOC) ||
             (test_kind == TEST_KIND_RESISTANCE)) && current_foc_active) {
          const float id_target = current_foc_id_target;
          const float iq_target = current_foc_iq_target;
          const int32_t iq_target_ma = (int32_t)(1000.0F * iq_target);
          test_iq_target_sum_ma += iq_target_ma;
          if (iq_target_ma < test_iq_target_min_ma) {
            test_iq_target_min_ma = iq_target_ma;
          }
          if (iq_target_ma > test_iq_target_max_ma) {
            test_iq_target_max_ma = iq_target_ma;
          }
          const float error_d = id_target - id;
          const float error_q = iq_target - iq;
          const float voltage_limit_counts =
              (float)(TIM1->ARR /
                  (test_kind == TEST_KIND_RESISTANCE ?
                       RESISTANCE_MODULATION_DIVISOR :
                       CURRENT_FOC_MODULATION_DIVISOR));

          const float current_ki = test_kind == TEST_KIND_RESISTANCE ?
              RESISTANCE_FOC_KI_COUNTS_PER_AMP_SECOND :
              CURRENT_FOC_KI_COUNTS_PER_AMP_SECOND;
          current_foc_integral_d += error_d * current_ki *
              CURRENT_FOC_DT_SECONDS;
          current_foc_integral_q += error_q * current_ki *
              CURRENT_FOC_DT_SECONDS;
          if (current_foc_integral_d > voltage_limit_counts) {
            ++test_integral_d_saturated_samples;
            current_foc_integral_d = voltage_limit_counts;
          } else if (current_foc_integral_d < -voltage_limit_counts) {
            ++test_integral_d_saturated_samples;
            current_foc_integral_d = -voltage_limit_counts;
          }
          if (current_foc_integral_q > voltage_limit_counts) {
            ++test_integral_q_saturated_samples;
            current_foc_integral_q = voltage_limit_counts;
          } else if (current_foc_integral_q < -voltage_limit_counts) {
            ++test_integral_q_saturated_samples;
            current_foc_integral_q = -voltage_limit_counts;
          }

          float voltage_d = current_foc_integral_d +
              CURRENT_FOC_KP_COUNTS_PER_AMP * error_d;
          float voltage_q = current_foc_integral_q +
              CURRENT_FOC_KP_COUNTS_PER_AMP * error_q;
          const float voltage_abs_sum =
              (voltage_d < 0.0F ? -voltage_d : voltage_d) +
              (voltage_q < 0.0F ? -voltage_q : voltage_q);
          const uint16_t voltage_request_counts = voltage_abs_sum > 65535.0F ?
              UINT16_MAX : (uint16_t)voltage_abs_sum;
          test_voltage_request_sum_counts += voltage_request_counts;
          if (voltage_request_counts > test_voltage_request_max_counts) {
            test_voltage_request_max_counts = voltage_request_counts;
          }
          if (voltage_abs_sum > voltage_limit_counts) {
            ++test_voltage_saturated_samples;
            const float scale = voltage_limit_counts / voltage_abs_sum;
            voltage_d *= scale;
            voltage_q *= scale;
          }

          if (test_kind == TEST_KIND_RESISTANCE) {
            const int32_t applied_vd_mv = (int32_t)(
                voltage_d * (float)bus_voltage_mv / (float)TIM1->ARR);
            const int32_t applied_vq_mv = (int32_t)(
                voltage_q * (float)bus_voltage_mv / (float)TIM1->ARR);
            resistance_id_ma = id_ma;
            resistance_iq_ma = iq_ma;
            resistance_vd_mv = applied_vd_mv;
            resistance_vq_mv = applied_vq_mv;
            resistance_live_milliohms =
                ((id_ma > 20) || (id_ma < -20)) ?
                    (applied_vd_mv * 1000) / id_ma : 0;
            if (resistance_phase == RESISTANCE_PHASE_FORWARD_SAMPLE) {
              ++resistance_forward_samples;
              resistance_forward_id_sum_ma += id_ma;
              resistance_forward_vd_sum_mv += applied_vd_mv;
              /* Preserve the old fields and offsets for protocol compatibility. */
              ++resistance_measurement_samples;
              resistance_iq_sum_ma += id_ma;
              resistance_vq_sum_mv += applied_vd_mv;
            } else if (resistance_phase == RESISTANCE_PHASE_REVERSE_SAMPLE) {
              ++resistance_reverse_samples;
              resistance_reverse_id_sum_ma += id_ma;
              resistance_reverse_vd_sum_mv += applied_vd_mv;
            }
          }

          const float voltage_alpha = cosine * voltage_d - sine * voltage_q;
          const float voltage_beta = sine * voltage_d + cosine * voltage_q;
          const int32_t neutral = (int32_t)(TIM1->ARR / 2U);
          const int32_t phase_a = neutral + (int32_t)voltage_alpha;
          const int32_t phase_b = neutral + (int32_t)(
              -0.5F * voltage_alpha + 0.866025404F * voltage_beta);
          const int32_t phase_c = neutral + (int32_t)(
              -0.5F * voltage_alpha - 0.866025404F * voltage_beta);
          set_compare_values((uint16_t)phase_a,
                             (uint16_t)phase_b,
                             (uint16_t)phase_c);
        }
      }
    }

    overcurrent_count = overcurrent ? (uint8_t)(overcurrent_count + 1U) : 0U;
    if (overcurrent_count >= CURRENT_TRIP_CONSECUTIVE_SAMPLES) {
      power_stage_disable();
      latched_faults |= SOFTWARE_OVERCURRENT_FAULT;
      stage_state = POWER_STAGE_FAULT;
      test_state = POWER_STAGE_TEST_ABORTED;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
  if (gpio_pin != DRV_FAULT_N_Pin) {
    return;
  }

  power_stage_disable();
  stage_state = POWER_STAGE_FAULT;
  test_state = POWER_STAGE_TEST_ABORTED;
  /* SPI is intentionally not used in the ISR. Status is read later. */
  latched_faults |= DRV8301_FAULT_FAULT;
  fault_status_pending = true;
}
