#include "power_stage.h"

#include <limits.h>
#include <stddef.h>

#include "adc.h"
#include "angle_sensor.h"
#include "drv8301.h"
#include "main.h"
#include "tim.h"

enum {
  CURRENT_CALIBRATION_SAMPLES = 1024U,
  /* Absolute commissioning ceiling: +/-10% around center-aligned neutral. */
  COMMISSIONING_MODULATION_DIVISOR = 10U,
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
  RESISTANCE_RAMP_UP_MS = 500U,
  RESISTANCE_SETTLE_END_MS = 1000U,
  RESISTANCE_SAMPLE_END_MS = 2000U,
  RESISTANCE_TEST_END_MS = 2500U,
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
  TEST_KIND_RESISTANCE
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
static volatile uint8_t test_transform_divider;
static volatile bool current_foc_active;
static volatile float current_foc_integral_d;
static volatile float current_foc_integral_q;
static volatile float current_foc_iq_target;
static uint32_t current_foc_outer_last_ms;
static float current_foc_pll_position_counts;
static float current_foc_pll_speed_counts_per_second;
static float current_foc_speed_reference_dps;
static float current_foc_target_position_counts;
static float current_foc_speed_integral_amps;

static const float CURRENT_FOC_MAX_TARGET_AMPS = 0.30F;
static const float RESISTANCE_MEASUREMENT_TARGET_AMPS = 0.25F;
static const float CURRENT_FOC_TORQUE_FEEDFORWARD_AMPS = 0.12F;
static const float CURRENT_FOC_KP_COUNTS_PER_AMP = 40.0F;
static const float CURRENT_FOC_KI_COUNTS_PER_AMP_SECOND = 800.0F;
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
  test_transform_divider = 0U;
  current_foc_integral_d = 0.0F;
  current_foc_integral_q = 0.0F;
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
  const uint16_t deviation =
      (uint16_t)(TIM1->ARR / COMMISSIONING_MODULATION_DIVISOR);
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
    const uint32_t elapsed = (uint32_t)(now - test_started_at);
    if ((stage_state != POWER_STAGE_RUNNING) ||
        (elapsed >= RESISTANCE_TEST_END_MS)) {
      power_stage_disable();
      test_state = elapsed >= RESISTANCE_TEST_END_MS ?
          POWER_STAGE_TEST_COMPLETED : POWER_STAGE_TEST_ABORTED;
      test_kind = TEST_KIND_NONE;
    } else if (elapsed < RESISTANCE_RAMP_UP_MS) {
      current_foc_iq_target = RESISTANCE_MEASUREMENT_TARGET_AMPS *
          (float)elapsed / (float)RESISTANCE_RAMP_UP_MS;
    } else if (elapsed < RESISTANCE_SAMPLE_END_MS) {
      current_foc_iq_target = RESISTANCE_MEASUREMENT_TARGET_AMPS;
    } else {
      current_foc_iq_target = RESISTANCE_MEASUREMENT_TARGET_AMPS *
          (float)(RESISTANCE_TEST_END_MS - elapsed) /
          (float)(RESISTANCE_TEST_END_MS - RESISTANCE_SAMPLE_END_MS);
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
      (test_kind == TEST_KIND_ENCODER_VOLTAGE)) {
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
        test_state = POWER_STAGE_TEST_COMPLETED;
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
          const int32_t target_advance_counts = (int32_t)(
              ((uint64_t)rotation_elapsed * 4096U *
               ENCODER_VOLTAGE_TARGET_DEGREES_PER_SECOND) / 360000U);
          const int32_t target_counts = test_start_position_counts +
              (test_direction > 0 ? target_advance_counts :
                                    -target_advance_counts);
          const int32_t error_counts = target_counts - sample.position_counts;
          const uint32_t absolute_error = (uint32_t)(
              error_counts < 0 ? -error_counts : error_counts);
          uint32_t maximum_amplitude = TIM1->ARR / 12U;
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
  const uint16_t neutral = (uint16_t)(TIM1->ARR / 2U);
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
  current_foc_iq_target = 0.0F;
  current_foc_active = true;
  test_kind = TEST_KIND_RESISTANCE;
  if (!power_stage_enable(neutral, neutral, neutral)) {
    current_foc_active = false;
    test_state = POWER_STAGE_TEST_ABORTED;
    test_kind = TEST_KIND_NONE;
    return false;
  }
  test_state = POWER_STAGE_TEST_RUNNING;
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

  if (stage_state == POWER_STAGE_CALIBRATING) {
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
      if ((centered > (int32_t)CURRENT_TRIP_ADC_COUNTS) ||
          (centered < -(int32_t)CURRENT_TRIP_ADC_COUNTS)) {
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
        (((test_kind == TEST_KIND_CURRENT_FOC) ||
          (test_kind == TEST_KIND_RESISTANCE)) && current_foc_active) ?
        1U : CURRENT_TRANSFORM_DIAGNOSTIC_DIVIDER;
    if ((((test_kind == TEST_KIND_ENCODER_VOLTAGE) &&
          (test_rotation_started_at != 0U)) ||
         (((test_kind == TEST_KIND_CURRENT_FOC) ||
           (test_kind == TEST_KIND_RESISTANCE)) && current_foc_active)) &&
        (++test_transform_divider >= transform_divider_limit)) {
      test_transform_divider = 0U;
      const bool is_v7 = (TIM1->CR1 & TIM_CR1_DIR) == 0U;
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
      const bool angle_valid = test_kind == TEST_KIND_RESISTANCE ?
          ((electrical_raw = 0U), true) :
          angle_sensor_get_electrical_raw_fast(&electrical_raw);
      if (angle_valid) {
        const float ia = (float)corrected[0] * CURRENT_ADC_TO_AMPS;
        const float ib = (float)corrected[1] * CURRENT_ADC_TO_AMPS;
        const float i_alpha = ia;
        const float i_beta = 0.577350269F * ia + 1.154700538F * ib;
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

        if (((test_kind == TEST_KIND_CURRENT_FOC) ||
             (test_kind == TEST_KIND_RESISTANCE)) && current_foc_active) {
          const float iq_target = current_foc_iq_target;
          const int32_t iq_target_ma = (int32_t)(1000.0F * iq_target);
          test_iq_target_sum_ma += iq_target_ma;
          if (iq_target_ma < test_iq_target_min_ma) {
            test_iq_target_min_ma = iq_target_ma;
          }
          if (iq_target_ma > test_iq_target_max_ma) {
            test_iq_target_max_ma = iq_target_ma;
          }
          const float error_d = -id;
          const float error_q = iq_target - iq;
          const float voltage_limit_counts =
              (float)(TIM1->ARR /
                  (test_kind == TEST_KIND_RESISTANCE ?
                       COMMISSIONING_MODULATION_DIVISOR :
                       CURRENT_FOC_MODULATION_DIVISOR));

          current_foc_integral_d += error_d *
              CURRENT_FOC_KI_COUNTS_PER_AMP_SECOND * CURRENT_FOC_DT_SECONDS;
          current_foc_integral_q += error_q *
              CURRENT_FOC_KI_COUNTS_PER_AMP_SECOND * CURRENT_FOC_DT_SECONDS;
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
            const uint32_t measurement_elapsed =
                (uint32_t)(HAL_GetTick() - test_started_at);
            if ((measurement_elapsed >= RESISTANCE_SETTLE_END_MS) &&
                (measurement_elapsed < RESISTANCE_SAMPLE_END_MS)) {
              const int32_t applied_vq_mv = (int32_t)(
                  voltage_q * (float)bus_voltage_mv / (float)TIM1->ARR);
              ++resistance_measurement_samples;
              resistance_iq_sum_ma += iq_ma;
              resistance_vq_sum_mv += applied_vq_mv;
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
