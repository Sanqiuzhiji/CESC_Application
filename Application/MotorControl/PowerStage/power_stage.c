#include "power_stage.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include "adc.h"
#include "angle_sensor.h"
#include "drv8301.h"
#include "foc.h"
#include "foc_observer.h"
#include "motor_control_config.h"
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
  ENCODER_ALIGNMENT_SWEEP_START_MS = 1000U,
  ENCODER_ALIGNMENT_FORWARD_END_MS = 5000U,
  ENCODER_ALIGNMENT_REVERSE_END_MS = 9000U,
  ENCODER_ALIGNMENT_ZERO_CAPTURE_MS = 1300U,
  CURRENT_FOC_ALIGNMENT_RAMP_UP_MS = 600U,
  CURRENT_FOC_ALIGNMENT_RAMP_DOWN_MS = 200U,
  ENCODER_ALIGNMENT_TIMEOUT_MS = 10000U,
  ENCODER_VOLTAGE_ALIGN_MS = 1500U,
  ENCODER_VOLTAGE_ROTATE_MS = 10000U,
  ENCODER_VOLTAGE_TIMEOUT_MS = 13000U,
  ENCODER_VOLTAGE_TARGET_DEGREES_PER_SECOND = 2U,
  ENCODER_VOLTAGE_RAMP_DOWN_MS = 500U,
  ENCODER_VOLTAGE_FULL_OUTPUT_ERROR_COUNTS = 46U,
  CURRENT_FOC_ACTIVE_MS = 10000U,
  CURRENT_FOC_TIMEOUT_MS = 13000U,
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
  VESC_INDUCTANCE_TIMER_ARR = 55999U,
  VESC_INDUCTANCE_SAMPLE_OFFSET_COUNTS = 10U,
  VESC_INDUCTANCE_RISE_COMP_COUNTS = 50U,
  VESC_INDUCTANCE_SEARCH_SEQUENCES = 10U,
  VESC_INDUCTANCE_MEASURE_SEQUENCES = 200U,
  INDUCTANCE_HALF_PERIOD_TICKS = 20U,
  INDUCTANCE_ENDPOINT_AVERAGE_TICKS = 8U,
  INDUCTANCE_SETTLE_HALF_PERIODS = 40U,
  INDUCTANCE_TARGET_SAMPLES_PER_DIRECTION = 250U,
  FLUX_SAMPLE_START_MS = 4000U,
  FLUX_SAMPLE_END_MS = 9000U,
  CURRENT_TRANSFORM_DIAGNOSTIC_DIVIDER = 20U,
  /* 40 counts is about 3.2 A. The measured 5.4-ohm phase and 8.1 V bus cannot
   * sustain that current; this threshold rejects switching transients while
   * DRV8301 hardware protection remains the final asynchronous safeguard. */
  CURRENT_TRIP_ADC_COUNTS = 40U,
  CURRENT_TRIP_CONSECUTIVE_SAMPLES = 3U,
  SOFTWARE_OVERSPEED_FAULT = 1U << 14,
  SOFTWARE_OVERCURRENT_FAULT = 1U << 15,
  COMMISSIONING_TEST_MODULATION_DIVISOR = 12U,
  /* VESC uses a second timer reset by every TIM1 update so the current ADCs
   * sample shortly after both zero vectors (V0 and V7). At 168 MHz, 200
   * counts gives the gate driver and current amplifiers about 1.19 us to
   * settle after the PWM boundary. */
  ADC_ZERO_VECTOR_SETTLE_COUNTS = 200U
};

#define ENCODER_CONTROL_MAX_SAMPLE_AGE_MS \
  (motor_control_config.encoder_max_sample_age_ms)
#define ENCODER_CONTROL_INVALID_LIMIT \
  (motor_control_config.encoder_invalid_sample_limit)
#define COMMISSIONING_MIN_BUS_MV \
  (motor_control_config.minimum_bus_voltage_mv)
#define COMMISSIONING_MAX_BUS_MV \
  (motor_control_config.maximum_bus_voltage_mv)
#define CONTROL_COMMAND_TIMEOUT_MS \
  (motor_control_config.command_timeout_ms)
#define CONTROL_SENSOR_STARTUP_GRACE_MS \
  (motor_control_config.sensor_startup_grace_ms)
#define CONTROL_MAX_IQ_MA (motor_control_config.maximum_iq_ma)
#define CONTROL_MIN_ACTIVE_IQ_MA \
  (motor_control_config.minimum_active_iq_ma)
#define CURRENT_FOC_MODULATION_DIVISOR \
  (motor_control_config.current_modulation_divisor)
#define HIGH_SPEED_MODULATION_DIVISOR \
  (motor_control_config.high_speed_modulation_divisor)
#define DIRECT_VOLTAGE_MODULATION_DIVISOR \
  (motor_control_config.direct_voltage_modulation_divisor)

/*
 * AD8418 gain = 20 V/V and phase shunt = 0.5 mOhm, giving
 * 0.08058608 A/count at VDDA = 3.3 V. The schematic connects IN+ to SH_x
 * and IN- to motor terminal P_x, so a positive ADC delta represents current
 * from the motor back into the bridge. FOC phase current is defined in the
 * opposite direction (bridge to motor), hence the negative sign.
 */
#define CURRENT_ADC_TO_AMPS (motor_control_config.current_adc_amps_per_count)

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
static volatile uint8_t speed_voltage_phase_limit_count;
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
static float test_alignment_offset_sin_sum;
static float test_alignment_offset_cos_sum;
static uint32_t test_alignment_offset_samples;
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
static volatile uint8_t inductance_pulse_state;
static volatile uint32_t inductance_pulse_duty_counts;
static volatile uint32_t inductance_pulse_sequences;
static volatile uint32_t inductance_pulse_current_samples;
static volatile int64_t inductance_pulse_current_ma_sum;
static volatile uint64_t inductance_pulse_bus_mv_sum;
static volatile bool inductance_pulse_searching;
static volatile int32_t inductance_pulse_baseline_counts[3];
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
static volatile power_stage_control_mode_t control_mode;
static volatile int32_t control_id_ma;
static volatile int32_t control_iq_ma;
static volatile int32_t control_iq_target_ma;
static volatile uint32_t control_command_at_ms;
static volatile bool control_command_timeout_latched;
static uint8_t control_sensor_invalid_count;
static volatile int32_t control_speed_target_mdps;
static volatile int32_t control_speed_actual_mdps;
static volatile int32_t control_speed_voltage_q_counts;
static float control_speed_voltage_integral_counts;
static volatile int32_t control_speed_voltage_limit_counts;
static volatile bool control_speed_voltage_current_limited;
static volatile int32_t control_speed_voltage_fast_ceiling_counts;
static volatile uint16_t control_speed_voltage_fast_limit_hold_cycles;
static volatile bool control_speed_current_foc;
static int32_t control_speed_sample_position_counts;
static uint32_t control_speed_sample_timestamp_ms;
static volatile int32_t control_position_target_mdeg;
static volatile int32_t control_position_actual_mdeg;
static volatile int32_t control_position_profile_mdeg;
static volatile int32_t control_position_profile_speed_mdps;
static volatile int32_t control_position_profile_acceleration_mdps2;
static volatile int32_t control_position_profile_deceleration_mdps2;
static float control_position_profile_velocity_mdps;
static volatile int32_t control_haptic_spacing_mdeg;
static volatile int32_t control_haptic_strength_ma;
static volatile int32_t control_haptic_damping_ma_per_dps;
static volatile int32_t control_haptic_minimum_mdeg;
static volatile int32_t control_haptic_maximum_mdeg;
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
static float current_foc_speed_iq_command_amps;
static volatile float control_pll_phase_rad;
static volatile float control_pll_speed_electrical_rad_per_second;
static volatile bool control_pll_initialized;
static uint32_t control_pll_sensor_sequence;
static uint32_t control_pll_sensor_timestamp_ms;
static volatile uint16_t control_pll_predicted_electrical_raw;
static foc_observer_state_t speed_observer;
static volatile uint16_t speed_observer_phase_raw;
static volatile int16_t speed_observer_encoder_error_raw;
static volatile int32_t speed_observer_erpm;
static volatile bool speed_observer_using_encoder;
static float precise_encoder_phase_raw;
static uint32_t precise_encoder_last_cycles;
static uint32_t precise_encoder_sensor_sequence;
static bool precise_encoder_initialized;
static float precise_encoder_blend;
static bool precise_encoder_requested;

static const float CURRENT_FOC_MAX_TARGET_AMPS = 0.30F;
/*
 * The reference VESC auto-detection raises the locking current until the
 * measurement has useful voltage and current headroom.  CESC cannot use the
 * VESC's 4.15 A bench current with its present 20-count software trip. Bench
 * comparison also showed modulation-dependent bias above 0.5 A, so retain the
 * proven-safe 0.5 A locking current until the ADC/PWM sampling is redesigned.
 */
static const float RESISTANCE_MEASUREMENT_TARGET_AMPS = 0.40F;
static const float CURRENT_FOC_TORQUE_FEEDFORWARD_AMPS = 0.12F;
/*
 * VESC tunes the current loop with Kp = L / tc and Ki = R / tc. Using the
 * repeatable bench values R=2.20 ohm and L=1.16 mH, converted from volts to
 * TIM1 counts at the nominal 8 V bus, and a conservative tc=4 ms gives these
 * initial gains. VESC's automatic configuration uses tc=1 ms; the slower
 * starting point leaves margin for the CESC ADC's coarse current resolution.
 */
static const float RESISTANCE_FOC_KI_COUNTS_PER_AMP_SECOND = 5000.0F;
#define CURRENT_FOC_DT_SECONDS \
  (motor_control_config.current_loop_period_seconds)
static const float CURRENT_FOC_POSITION_TO_SPEED_GAIN = 1.0F;
static const float CURRENT_FOC_MAX_SPEED_TARGET_DPS = 15.0F;
static const float CURRENT_FOC_ACCELERATION_DPS2 = 20.0F;
static const float CURRENT_FOC_PLL_KP_PER_SECOND = 20.0F;
static const float CURRENT_FOC_PLL_KI_PER_SECOND2 = 100.0F;
static const float CURRENT_FOC_SPEED_KP_AMPS_PER_DPS = 0.002F;
static const float CURRENT_FOC_SPEED_KI_AMPS_PER_DEGREE = 0.005F;
#define CONTROL_SPEED_MAX_DPS \
  ((float)motor_control_config.maximum_speed_mdps * 0.001F)
#define CONTROL_SPEED_ACCELERATION_DPS2 \
  ((float)motor_control_config.default_speed_acceleration_mdps2 * 0.001F)
/* Benjamin Vedder VESC defaults and normalization used by pll_run() and
 * run_pid_control_speed(). Speed is electrical RPM in the PID. */
/* VESC exposes these as motor-configuration parameters. AS5600 updates much
 * slower than the ABI/SPI encoders behind VESC's defaults, so retain
 * pll_run() but reduce its bandwidth by one decade. */
#define CONTROL_FOC_PLL_KP (motor_control_config.encoder_pll_kp)
#define CONTROL_FOC_PLL_KI (motor_control_config.encoder_pll_ki)
/* This application runs at 55--82.5 eRPM, far below VESC's default
 * s_pid_min_erpm=900. Keep VESC's controller equation, but scale its
 * configurable gains for this low-eRPM operating range. */
#define CONTROL_MOTOR_POLE_PAIRS ((float)motor_control_config.pole_pairs)
/* Match VESC's position-control structure: position error produces Iq
 * directly.  A small breakaway term compensates this motor's measured
 * stiction, but is removed close to the target to avoid a limit cycle. */
/*
 * VESC bounds and clamps its speed PID before feeding the Iq current loop.
 * This board resolves about 80.6 mA per ADC count, so speed mode retains that
 * bounded PI structure but drives an encoder-oriented Vq vector directly.
 * ARR/12 is the already bench-proven encoder-voltage-test limit.
 */
#define CONTROL_SPEED_VOLTAGE_FULL_OUTPUT_ERROR_COUNTS \
  (motor_control_config.speed_full_output_error_counts)
#define CONTROL_POSITION_VOLTAGE_FULL_OUTPUT_ERROR_COUNTS \
  (motor_control_config.position_full_output_error_counts)

static void reset_foc_control_statistics(void)
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
  test_transform_divider = 0U;
  for (uint32_t phase = 0U; phase < 3U; ++phase) {
    test_current_sum[phase] = 0;
    test_current_min[phase] = INT16_MAX;
    test_current_max[phase] = INT16_MIN;
    test_reconstructed_samples[phase] = 0U;
  }
}

static void reset_test_current_statistics(void)
{
  reset_foc_control_statistics();
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
  current_foc_speed_iq_command_amps = 0.0F;
  control_pll_phase_rad = 0.0F;
  control_pll_speed_electrical_rad_per_second = 0.0F;
  control_pll_initialized = false;
  control_pll_sensor_sequence = 0U;
  control_pll_sensor_timestamp_ms = 0U;
  control_pll_predicted_electrical_raw = 0U;
  foc_observer_reset(&speed_observer, 0.0F);
  speed_observer_phase_raw = 0U;
  speed_observer_encoder_error_raw = 0;
  speed_observer_erpm = 0;
  speed_observer_using_encoder = true;
  precise_encoder_phase_raw = 0.0F;
  precise_encoder_last_cycles = 0U;
  precise_encoder_sensor_sequence = 0U;
  precise_encoder_initialized = false;
  precise_encoder_blend = 0.0F;
  precise_encoder_requested = false;
}

static void set_compare_values(uint16_t a, uint16_t b, uint16_t c);

static int32_t resistance_from_sums_milliohms(int64_t voltage_sum_mv,
                                              int64_t current_sum_ma)
{
  if ((current_sum_ma > -20) && (current_sum_ma < 20)) {
    return 0;
  }
  /* Match the VESC phase-parameter convention. The locked-vector voltage to
   * current ratio is converted to the star-equivalent FOC resistance by 2/3.
   * See mcpwm_foc_measure_resistance() in the pinned VESC reference project.
   */
  return (int32_t)((voltage_sum_mv * 2000) / (current_sum_ma * 3));
}

static void finish_resistance_measurement(void)
{
  resistance_forward_milliohms = resistance_from_sums_milliohms(
      resistance_forward_vd_sum_mv, resistance_forward_id_sum_ma);
  /* VESC db6ba047 performs one positive locked-current measurement. Keep the
   * legacy reverse fields zero rather than manufacturing a second result. */
  resistance_reverse_milliohms = 0;
  resistance_average_milliohms = resistance_forward_milliohms;
  resistance_valid =
      (resistance_forward_samples >= 1000U) &&
      (resistance_forward_id_sum_ma >
       (int64_t)resistance_forward_samples * 350) &&
      (resistance_forward_milliohms > 0);
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
  inductance_pulse_state = 0U;
  inductance_pulse_duty_counts =
      (VESC_INDUCTANCE_TIMER_ARR + 1U) / 50U;
  inductance_pulse_sequences = 0U;
  inductance_pulse_current_samples = 0U;
  inductance_pulse_current_ma_sum = 0;
  inductance_pulse_bus_mv_sum = 0U;
  inductance_pulse_searching = true;
  for (uint32_t phase = 0U; phase < 3U; ++phase) {
    inductance_pulse_baseline_counts[phase] = 0;
  }
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
      (3.14159265359F / 180.0F) * (float)motor_control_config.pole_pairs;
  const float bemf_mv = (float)flux_vq_mv -
      (float)resistance_average_milliohms * (float)flux_iq_ma * 0.001F;
  if (fabsf(omega_e) < 0.5F || bemf_mv * omega_e <= 0.0F) {
    return;
  }
  /* set_voltage_vector() records the raw modulation-vector voltage. Match the
   * VESC FOC phase-voltage normalization before publishing motor flux linkage.
   */
  const float linkage_wb = (bemf_mv * 0.001F) / omega_e * (2.0F / 3.0F);
  if (linkage_wb < 0.001F || linkage_wb > 2.0F) {
    return;
  }
  flux_linkage_uwb = (uint32_t)(linkage_wb * 1000000.0F);
  /* Datasheet Ke uses mechanical rad/s; dq flux linkage uses electrical rad/s. */
  flux_ke_uv_per_rad_s =
      flux_linkage_uwb * (uint32_t)motor_control_config.pole_pairs;
  flux_kv_millirpm_per_volt = (uint32_t)(
      60000.0F /
      (2.0F * 3.14159265359F *
       (float)motor_control_config.pole_pairs * linkage_wb));
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
        (2000.0F * atanhf(forward_ratio)) * (2.0F / 3.0F));
  }
  if ((reverse_ratio > 0.02F) && (reverse_ratio < 0.98F)) {
    inductance_reverse_uh = (uint32_t)(resistance_milliohms * half_period_us /
        (2000.0F * atanhf(reverse_ratio)) * (2.0F / 3.0F));
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
        (2000.0F * atanhf(differential_ratio)) * (2.0F / 3.0F));
  }
  const uint32_t inductance_difference =
      inductance_forward_uh > inductance_reverse_uh ?
          inductance_forward_uh - inductance_reverse_uh :
          inductance_reverse_uh - inductance_forward_uh;
  inductance_valid =
      (inductance_forward_samples >=
       INDUCTANCE_TARGET_SAMPLES_PER_DIRECTION) &&
      (inductance_reverse_samples >=
       INDUCTANCE_TARGET_SAMPLES_PER_DIRECTION) &&
      (forward_current_ma > 100.0F) &&
      (reverse_current_ma > 100.0F) &&
      (inductance_forward_uh > 0U) &&
      (inductance_reverse_uh > 0U) &&
      ((uint64_t)inductance_difference * 10U <=
       (uint64_t)inductance_forward_uh + inductance_reverse_uh) &&
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

static void select_current_adc_trigger(uint32_t trigger)
{
  MODIFY_REG(ADC1->CR2, ADC_CR2_JEXTSEL, trigger);
  MODIFY_REG(ADC2->CR2, ADC_CR2_JEXTSEL, trigger);
  MODIFY_REG(ADC3->CR2, ADC_CR2_JEXTSEL, trigger);
}

/* Match the sampling timer arrangement used by VESC: TIM1 emits an update at
 * both ends of its center-aligned count. That update resets TIM8, whose CC2
 * event starts all three injected ADC conversions after a short settling
 * delay. No TIM8 pin or interrupt is used. */
static void configure_zero_vector_adc_sampler(void)
{
  __HAL_RCC_TIM8_CLK_ENABLE();

  TIM8->CR1 = 0U;
  TIM8->CR2 = 0U;
  TIM8->SMCR = 0U;
  TIM8->DIER = 0U;
  TIM8->CCER = 0U;
  TIM8->PSC = 0U;
  TIM8->ARR = 0xFFFFU;
  TIM8->CCR2 = ADC_ZERO_VECTOR_SETTLE_COUNTS;
  TIM8->CCMR1 = TIM_CCMR1_OC2PE | (6U << TIM_CCMR1_OC2M_Pos);
  TIM8->CCER = TIM_CCER_CC2E;
  TIM8->BDTR = TIM_BDTR_MOE;
  TIM8->EGR = TIM_EGR_UG;

  /* On STM32F405, TIM8 ITR0 is TIM1 TRGO. Reset mode restarts the delay on
   * both the top and bottom update events of center-aligned TIM1. */
  MODIFY_REG(TIM1->CR2, TIM_CR2_MMS, TIM_TRGO_UPDATE);
  TIM8->SMCR = TIM_SMCR_SMS_2;
  TIM8->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
  select_current_adc_trigger(ADC_EXTERNALTRIGINJECCONV_T8_CC2);
}

static void configure_vesc_inductance_timer(void)
{
  select_current_adc_trigger(ADC_EXTERNALTRIGINJECCONV_T1_CC4);
  TIM1->CR1 |= TIM_CR1_UDIS;
  TIM1->ARR = VESC_INDUCTANCE_TIMER_ARR;
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;
  TIM1->CCR4 = inductance_pulse_duty_counts -
      VESC_INDUCTANCE_SAMPLE_OFFSET_COUNTS;
  TIM1->CNT = 0U;
  TIM1->CR1 &= ~TIM_CR1_UDIS;
  TIM1->EGR = TIM_EGR_UG;
}

static void restore_control_timer(void)
{
  TIM1->CR1 |= TIM_CR1_UDIS;
  TIM1->ARR = 4199U;
  TIM1->CCR1 = 2099U;
  TIM1->CCR2 = 2099U;
  TIM1->CCR3 = 2099U;
  TIM1->CCR4 = 4199U - ADC_ZERO_VECTOR_SETTLE_COUNTS;
  TIM1->CNT = 0U;
  TIM1->CR1 &= ~TIM_CR1_UDIS;
  TIM1->EGR = TIM_EGR_UG;
  select_current_adc_trigger(ADC_EXTERNALTRIGINJECCONV_T8_CC2);
}

static void finish_vesc_inductance_measurement(void)
{
  if (inductance_pulse_current_samples == 0U ||
      inductance_pulse_duty_counts <=
          VESC_INDUCTANCE_RISE_COMP_COUNTS) {
    return;
  }
  const float current_a =
      fabsf((float)inductance_pulse_current_ma_sum) /
      (float)inductance_pulse_current_samples * 0.001F;
  const float voltage_v =
      (float)inductance_pulse_bus_mv_sum /
      (float)inductance_pulse_current_samples * 0.001F;
  const float pulse_s =
      (float)(inductance_pulse_duty_counts -
              VESC_INDUCTANCE_RISE_COMP_COUNTS) / 168000000.0F;
  if (current_a <= 0.02F || voltage_v <= 0.0F || pulse_s <= 0.0F) {
    return;
  }
  inductance_average_uh = (uint32_t)(
      voltage_v * pulse_s / current_a * 1000000.0F * (2.0F / 3.0F));
  inductance_forward_uh = inductance_average_uh;
  inductance_reverse_uh = 0U;
  inductance_forward_samples = inductance_pulse_current_samples;
  inductance_reverse_samples = 0U;
  inductance_valid = inductance_average_uh >= 10U &&
      inductance_average_uh <= 100000U;
}

static void set_voltage_vector(uint16_t electrical_raw, uint16_t amplitude_counts)
{
  const float amplitude = (float)amplitude_counts;
  const float alpha = amplitude *
      foc_sine_raw((uint16_t)(electrical_raw + 1024U));
  const float beta = amplitude * foc_sine_raw(electrical_raw);
  const foc_pwm_output_t pwm = foc_voltage_to_pwm(
      alpha, beta, (uint16_t)TIM1->ARR);
  set_compare_values(pwm.a, pwm.b, pwm.c);
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
  uint16_t deviation;
  if ((control_mode == POWER_STAGE_CONTROL_SPEED) &&
      !control_speed_current_foc &&
      (control_speed_voltage_limit_counts > 0)) {
    deviation = control_speed_voltage_limit_counts > (int32_t)neutral ?
        neutral : (uint16_t)control_speed_voltage_limit_counts;
  } else {
    const uint16_t divisor =
        ((test_kind == TEST_KIND_RESISTANCE) ||
         (test_kind == TEST_KIND_INDUCTANCE)) ?
        RESISTANCE_MODULATION_DIVISOR :
        (control_speed_current_foc ? HIGH_SPEED_MODULATION_DIVISOR :
                                    COMMISSIONING_MODULATION_DIVISOR);
    deviation = (uint16_t)(TIM1->ARR / divisor);
  }
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
  control_mode = POWER_STAGE_CONTROL_DISABLED;
  control_iq_target_ma = 0;
  control_speed_target_mdps = 0;
  control_speed_actual_mdps = 0;
  control_speed_voltage_q_counts = 0;
  control_speed_voltage_integral_counts = 0.0F;
  control_speed_voltage_limit_counts = 0;
  control_speed_voltage_current_limited = false;
  control_speed_voltage_fast_ceiling_counts = 0;
  control_speed_voltage_fast_limit_hold_cycles = 0U;
  control_speed_current_foc = false;
  control_speed_sample_position_counts = 0;
  control_speed_sample_timestamp_ms = 0U;
  control_position_target_mdeg = 0;
  control_position_actual_mdeg = 0;
  control_position_profile_mdeg = 0;
  control_position_profile_speed_mdps = 0;
  control_position_profile_acceleration_mdps2 =
      motor_control_config.default_profile_acceleration_mdps2;
  control_position_profile_deceleration_mdps2 =
      motor_control_config.default_profile_acceleration_mdps2;
  control_position_profile_velocity_mdps = 0.0F;
  control_haptic_spacing_mdeg = 0;
  control_haptic_strength_ma = 0;
  control_haptic_damping_ma_per_dps = 0;
  control_haptic_minimum_mdeg = 0;
  control_haptic_maximum_mdeg = 0;
  control_sensor_invalid_count = 0U;
  current_foc_active = false;
  current_foc_id_target = 0.0F;
  current_foc_iq_target = 0.0F;
  current_foc_speed_reference_dps = 0.0F;
  current_foc_speed_integral_amps = 0.0F;
  current_foc_speed_iq_command_amps = 0.0F;
  control_pll_initialized = false;
  control_pll_sensor_sequence = 0U;
  control_pll_sensor_timestamp_ms = 0U;
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
  test_alignment_offset_sin_sum = 0.0F;
  test_alignment_offset_cos_sum = 0.0F;
  test_alignment_offset_samples = 0U;
  test_rotation_started_at = 0U;
  test_command_electrical_raw = 0U;
  test_start_position_counts = 0;
  test_sensor_invalid_count = 0U;
  current_foc_active = false;
  control_mode = POWER_STAGE_CONTROL_DISABLED;
  control_id_ma = 0;
  control_iq_ma = 0;
  control_iq_target_ma = 0;
  control_command_at_ms = 0U;
  control_command_timeout_latched = false;
  control_sensor_invalid_count = 0U;
  control_speed_target_mdps = 0;
  control_speed_actual_mdps = 0;
  control_speed_sample_position_counts = 0;
  control_speed_sample_timestamp_ms = 0U;
  control_position_target_mdeg = 0;
  control_position_actual_mdeg = 0;
  control_position_profile_mdeg = 0;
  control_position_profile_speed_mdps = 0;
  control_haptic_spacing_mdeg = 0;
  control_haptic_strength_ma = 0;
  control_haptic_damping_ma_per_dps = 0;
  control_haptic_minimum_mdeg = 0;
  control_haptic_maximum_mdeg = 0;
  reset_test_current_statistics();
  reset_inductance_statistics();
  test_steps_completed = 0U;
  test_settle_started_at = 0U;
  overcurrent_count = 0U;
  speed_voltage_phase_limit_count = 0U;
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
                        TIM1->ARR - ADC_ZERO_VECTOR_SETTLE_COUNTS);
  configure_zero_vector_adc_sampler();
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

  if (control_mode != POWER_STAGE_CONTROL_DISABLED) {
    angle_sensor_sample_t sample;
    const uint32_t command_elapsed =
        (uint32_t)(now - control_command_at_ms);
    const bool sensor_valid = angle_sensor_get_sample(&sample) &&
        sample.electrical_zero_calibrated &&
        ((uint32_t)(now - sample.timestamp_ms) <=
         ENCODER_CONTROL_MAX_SAMPLE_AGE_MS);
    bool disable_control =
        (command_elapsed >= CONTROL_COMMAND_TIMEOUT_MS) ||
        (stage_state != POWER_STAGE_RUNNING);
    if (!sensor_valid &&
        (command_elapsed >= CONTROL_SENSOR_STARTUP_GRACE_MS)) {
      if (control_sensor_invalid_count < UINT8_MAX) {
        ++control_sensor_invalid_count;
      }
      if (control_sensor_invalid_count >= ENCODER_CONTROL_INVALID_LIMIT) {
        disable_control = true;
      }
    } else if (sensor_valid) {
      control_sensor_invalid_count = 0U;
    }
    if (disable_control) {
      power_stage_disable();
    }
  }

  if (((control_mode == POWER_STAGE_CONTROL_SPEED) ||
       (control_mode == POWER_STAGE_CONTROL_POSITION_PROFILE)) &&
      (llabs((long long)control_speed_actual_mdps) >
       (long long)motor_control_config.maximum_speed_mdps * 11LL / 10LL)) {
    power_stage_disable();
    latched_faults |= SOFTWARE_OVERSPEED_FAULT;
    stage_state = POWER_STAGE_FAULT;
  }

  if (((control_mode == POWER_STAGE_CONTROL_SPEED) ||
       (control_mode == POWER_STAGE_CONTROL_POSITION) ||
       (control_mode == POWER_STAGE_CONTROL_POSITION_PROFILE) ||
       (control_mode == POWER_STAGE_CONTROL_HAPTIC)) &&
      ((uint32_t)(now - current_foc_outer_last_ms) >=
       CURRENT_FOC_OUTER_LOOP_PERIOD_MS)) {
    angle_sensor_sample_t sample;
    if (angle_sensor_get_sample(&sample)) {
      const uint32_t elapsed_ms_raw =
          (uint32_t)(now - current_foc_outer_last_ms);
      const uint32_t elapsed_ms =
          elapsed_ms_raw > 20U ? 20U : elapsed_ms_raw;
      const float dt = (float)elapsed_ms * 0.001F;
      control_position_actual_mdeg = (int32_t)(
          (int64_t)sample.position_counts * 360000LL / 4096LL);
      if (((control_mode == POWER_STAGE_CONTROL_SPEED) ||
           (control_mode == POWER_STAGE_CONTROL_POSITION_PROFILE)) &&
          (!control_speed_current_foc ||
           !motor_control_config.high_speed_pll_prediction_enabled)) {
        /* VESC-style position PLL for the outer speed loop. Unlike a
         * one-sample finite difference, this does not turn AS5600 count
         * quantization or an occasional delayed sample into an Iq spike. */
        const float pll_error_counts =
            (float)sample.position_counts - current_foc_pll_position_counts;
        current_foc_pll_position_counts +=
            (current_foc_pll_speed_counts_per_second +
             CURRENT_FOC_PLL_KP_PER_SECOND * pll_error_counts) * dt;
        current_foc_pll_speed_counts_per_second +=
            CURRENT_FOC_PLL_KI_PER_SECOND2 * pll_error_counts * dt;
        const float maximum_speed_counts_per_second =
            CONTROL_SPEED_MAX_DPS * (4096.0F / 360.0F) * 1.2F;
        if (current_foc_pll_speed_counts_per_second >
            maximum_speed_counts_per_second) {
          current_foc_pll_speed_counts_per_second =
              maximum_speed_counts_per_second;
        } else if (current_foc_pll_speed_counts_per_second <
                   -maximum_speed_counts_per_second) {
          current_foc_pll_speed_counts_per_second =
              -maximum_speed_counts_per_second;
        }
        control_speed_actual_mdps = (int32_t)(
            current_foc_pll_speed_counts_per_second *
            (360000.0F / 4096.0F));
        control_speed_sample_position_counts = sample.position_counts;
        control_speed_sample_timestamp_ms = sample.timestamp_ms;
      }
      if (control_mode == POWER_STAGE_CONTROL_POSITION_PROFILE) {
        const float remaining_mdeg = (float)(control_position_target_mdeg -
            control_position_profile_mdeg);
        const float remaining_abs_mdeg = fabsf(remaining_mdeg);
        const float braking_speed_mdps = sqrtf(2.0F *
            (float)control_position_profile_deceleration_mdps2 *
            remaining_abs_mdeg);
        float desired_speed_mdps = braking_speed_mdps <
            (float)control_position_profile_speed_mdps ?
                braking_speed_mdps :
                (float)control_position_profile_speed_mdps;
        if (remaining_mdeg < 0.0F) {
          desired_speed_mdps = -desired_speed_mdps;
        }
        const bool same_direction =
            control_position_profile_velocity_mdps * desired_speed_mdps >= 0.0F;
        const bool increasing_speed = same_direction &&
            fabsf(desired_speed_mdps) >
            fabsf(control_position_profile_velocity_mdps);
        const float velocity_step = (float)(increasing_speed ?
            control_position_profile_acceleration_mdps2 :
            control_position_profile_deceleration_mdps2) * dt;
        if (control_position_profile_velocity_mdps < desired_speed_mdps) {
          control_position_profile_velocity_mdps += velocity_step;
          if (control_position_profile_velocity_mdps > desired_speed_mdps) {
            control_position_profile_velocity_mdps = desired_speed_mdps;
          }
        } else if (control_position_profile_velocity_mdps >
                   desired_speed_mdps) {
          control_position_profile_velocity_mdps -= velocity_step;
          if (control_position_profile_velocity_mdps < desired_speed_mdps) {
            control_position_profile_velocity_mdps = desired_speed_mdps;
          }
        }
        const float profile_step_mdeg =
            control_position_profile_velocity_mdps * dt;
        if ((remaining_abs_mdeg <= fabsf(profile_step_mdeg)) ||
            (remaining_abs_mdeg < 1.0F)) {
          control_position_profile_mdeg = control_position_target_mdeg;
          control_position_profile_velocity_mdps = 0.0F;
        } else {
          control_position_profile_mdeg += (int32_t)profile_step_mdeg;
        }
        float profile_speed_correction_mdps =
            (float)(control_position_profile_mdeg -
                    control_position_actual_mdeg) *
            motor_control_config.position_profile_following_kp_per_second;
        const float profile_speed_limit =
            (float)control_position_profile_speed_mdps;
        const float profile_correction_limit = profile_speed_limit *
            motor_control_config.position_profile_following_speed_fraction;
        if (profile_speed_correction_mdps > profile_correction_limit) {
          profile_speed_correction_mdps = profile_correction_limit;
        } else if (profile_speed_correction_mdps <
                   -profile_correction_limit) {
          profile_speed_correction_mdps = -profile_correction_limit;
        }
        float profile_speed_command_mdps =
            control_position_profile_velocity_mdps +
            profile_speed_correction_mdps;
        if (profile_speed_command_mdps > profile_speed_limit) {
          profile_speed_command_mdps = profile_speed_limit;
        } else if (profile_speed_command_mdps < -profile_speed_limit) {
          profile_speed_command_mdps = -profile_speed_limit;
        }
        control_speed_target_mdps =
            (int32_t)lroundf(profile_speed_command_mdps);
      }
      if (control_mode == POWER_STAGE_CONTROL_HAPTIC) {
        const int32_t spacing = control_haptic_spacing_mdeg;
        int32_t detent = control_position_actual_mdeg;
        if (spacing > 0) {
          detent = (int32_t)lroundf(
              (float)control_position_actual_mdeg / (float)spacing) * spacing;
        }
        int32_t error_mdeg = detent - control_position_actual_mdeg;
        int32_t iq_ma = spacing > 0 ?
            (int32_t)((int64_t)error_mdeg *
                      control_haptic_strength_ma / (spacing / 2)) : 0;
        iq_ma -= (int32_t)((int64_t)control_speed_actual_mdps *
                           control_haptic_damping_ma_per_dps / 1000LL);
        if (control_position_actual_mdeg < control_haptic_minimum_mdeg) {
          iq_ma += (control_haptic_minimum_mdeg -
                    control_position_actual_mdeg) / 10;
        } else if (control_position_actual_mdeg >
                   control_haptic_maximum_mdeg) {
          iq_ma -= (control_position_actual_mdeg -
                    control_haptic_maximum_mdeg) / 10;
        }
        const int32_t maximum_ma = motor_control_config.maximum_iq_ma;
        if (iq_ma > maximum_ma) { iq_ma = maximum_ma; }
        if (iq_ma < -maximum_ma) { iq_ma = -maximum_ma; }
        current_foc_iq_target = (float)iq_ma * 0.001F;
        control_iq_target_ma = iq_ma;
      } else if (control_mode == POWER_STAGE_CONTROL_POSITION) {
        const int32_t active_target = control_position_target_mdeg;
        const float position_error_counts =
            (float)(active_target - control_position_actual_mdeg) *
            (4096.0F / 360000.0F);
        const float voltage_limit_counts = (float)(
            TIM1->ARR / DIRECT_VOLTAGE_MODULATION_DIVISOR);
        float voltage_q_counts = position_error_counts *
            voltage_limit_counts /
            CONTROL_POSITION_VOLTAGE_FULL_OUTPUT_ERROR_COUNTS;
        if (voltage_q_counts > voltage_limit_counts) {
          voltage_q_counts = voltage_limit_counts;
        } else if (voltage_q_counts < -voltage_limit_counts) {
          voltage_q_counts = -voltage_limit_counts;
        }
        current_foc_speed_reference_dps = 0.0F;
        current_foc_speed_integral_amps = 0.0F;
        control_speed_target_mdps = 0;
        control_speed_voltage_q_counts = (int32_t)voltage_q_counts;
        current_foc_iq_target = 0.0F;
        control_iq_target_ma = 0;
        const foc_pwm_output_t pwm = foc_oriented_voltage_to_pwm(
            sample.electrical_raw, (int32_t)voltage_q_counts,
            (uint16_t)TIM1->ARR);
        set_compare_values(pwm.a, pwm.b, pwm.c);
      } else {
        const float requested_speed_dps =
            (float)control_speed_target_mdps * 0.001F;
        /* The direct-Vq path needs the already bench-proven gentle ramp.
         * Once current FOC owns torque, use the faster user speed ramp. */
        const float acceleration_dps2 =
            motor_control_config.speed_current_foc_enabled ?
                (control_speed_current_foc ?
                     CONTROL_SPEED_ACCELERATION_DPS2 :
                     (float)motor_control_config.low_speed_acceleration_mdps2 *
                         0.001F) :
                (float)motor_control_config.voltage_speed_acceleration_mdps2 *
                    0.001F;
        const float reference_step = acceleration_dps2 * dt;
        if (control_mode == POWER_STAGE_CONTROL_POSITION_PROFILE) {
          /* The trajectory generator already applies the user acceleration
           * and deceleration limits. A second speed ramp made feedback lag
           * the planned braking point and caused large target overshoot. */
          current_foc_speed_reference_dps = requested_speed_dps;
        } else if (current_foc_speed_reference_dps < requested_speed_dps) {
          current_foc_speed_reference_dps += reference_step;
          if (current_foc_speed_reference_dps > requested_speed_dps) {
            current_foc_speed_reference_dps = requested_speed_dps;
          }
        } else if (current_foc_speed_reference_dps > requested_speed_dps) {
          current_foc_speed_reference_dps -= reference_step;
          if (current_foc_speed_reference_dps < requested_speed_dps) {
            current_foc_speed_reference_dps = requested_speed_dps;
          }
        }

        const int32_t reference_abs_mdps = (int32_t)fabsf(
            current_foc_speed_reference_dps * 1000.0F);
        if (motor_control_config.speed_current_foc_enabled &&
            !control_speed_current_foc &&
            reference_abs_mdps >= motor_control_config.high_speed_enter_mdps) {
          /* Enter current control without removing the q-axis voltage that
           * is already carrying the rotor. This mirrors VESC's practice of
           * keeping its voltage integrator continuous across control-state
           * changes. Starting both Vq and Iq from zero made the rotor coast
           * to a stop before the speed loop rebuilt torque. */
          float transition_iq_amps = (float)control_iq_ma * 0.001F;
          const float maximum_iq_amps =
              (float)motor_control_config.maximum_iq_ma * 0.001F;
          if (transition_iq_amps > maximum_iq_amps) {
            transition_iq_amps = maximum_iq_amps;
          } else if (transition_iq_amps < -maximum_iq_amps) {
            transition_iq_amps = -maximum_iq_amps;
          }
          control_speed_current_foc = true;
          current_foc_integral_d = 0.0F;
          current_foc_integral_q = (float)control_speed_voltage_q_counts;
          current_foc_speed_integral_amps = 0.0F;
          current_foc_speed_iq_command_amps = transition_iq_amps;
          current_foc_id_target = 0.0F;
          current_foc_iq_target = transition_iq_amps;
          control_speed_voltage_q_counts = 0;
          control_speed_voltage_integral_counts = 0.0F;
          control_pll_initialized = false;
        } else if (motor_control_config.speed_current_foc_enabled &&
                   control_speed_current_foc &&
                   reference_abs_mdps <=
                       motor_control_config.high_speed_exit_mdps) {
          control_speed_current_foc = false;
          current_foc_integral_d = 0.0F;
          current_foc_integral_q = 0.0F;
          current_foc_speed_integral_amps = 0.0F;
          current_foc_speed_iq_command_amps = 0.0F;
          current_foc_target_position_counts = (float)sample.position_counts;
        }

        if (control_speed_current_foc) {
          const float actual_speed_dps =
              (float)control_speed_actual_mdps * 0.001F;
          const float target_erpm = current_foc_speed_reference_dps *
              (CONTROL_MOTOR_POLE_PAIRS / 6.0F);
          const float actual_erpm = actual_speed_dps *
              (CONTROL_MOTOR_POLE_PAIRS / 6.0F);
          const float speed_error_erpm = target_erpm - actual_erpm;
          const float maximum_iq =
              (float)motor_control_config.maximum_iq_ma * 0.001F;
          current_foc_speed_integral_amps += speed_error_erpm *
              motor_control_config.speed_pid_ki *
              motor_control_config.speed_pid_normalization * dt * maximum_iq;
          if (current_foc_speed_integral_amps > maximum_iq) {
            current_foc_speed_integral_amps = maximum_iq;
          } else if (current_foc_speed_integral_amps < -maximum_iq) {
            current_foc_speed_integral_amps = -maximum_iq;
          }
          float iq_target = speed_error_erpm *
              motor_control_config.speed_pid_kp *
              motor_control_config.speed_pid_normalization * maximum_iq +
              current_foc_speed_integral_amps;
          if (current_foc_speed_reference_dps > 0.0F) {
            iq_target += motor_control_config.speed_current_feedforward_amp;
          } else if (current_foc_speed_reference_dps < 0.0F) {
            iq_target -= motor_control_config.speed_current_feedforward_amp;
          }
          if (iq_target > maximum_iq) {
            iq_target = maximum_iq;
            if (speed_error_erpm > 0.0F) {
              current_foc_speed_integral_amps -= speed_error_erpm *
                  motor_control_config.speed_pid_ki *
                  motor_control_config.speed_pid_normalization * dt *
                  maximum_iq;
            }
          } else if (iq_target < -maximum_iq) {
            iq_target = -maximum_iq;
            if (speed_error_erpm < 0.0F) {
              current_foc_speed_integral_amps -= speed_error_erpm *
                  motor_control_config.speed_pid_ki *
                  motor_control_config.speed_pid_normalization * dt *
                  maximum_iq;
            }
          }
          /* VESC ramps externally requested setpoints before the current
           * controller. Keep the same separation here so entering the
           * high-speed path cannot step directly from Vq to breakaway Iq. */
          const float iq_step =
              motor_control_config.speed_iq_ramp_amp_per_second * dt;
          if (current_foc_speed_iq_command_amps < iq_target) {
            current_foc_speed_iq_command_amps += iq_step;
            if (current_foc_speed_iq_command_amps > iq_target) {
              current_foc_speed_iq_command_amps = iq_target;
            }
          } else if (current_foc_speed_iq_command_amps > iq_target) {
            current_foc_speed_iq_command_amps -= iq_step;
            if (current_foc_speed_iq_command_amps < iq_target) {
              current_foc_speed_iq_command_amps = iq_target;
            }
          }
          current_foc_id_target = 0.0F;
          current_foc_iq_target = current_foc_speed_iq_command_amps;
          control_iq_target_ma =
              (int32_t)(current_foc_speed_iq_command_amps * 1000.0F);
        } else {
          const float low_speed_limit_counts = (float)(
              TIM1->ARR / DIRECT_VOLTAGE_MODULATION_DIVISOR);
          float maximum_limit_counts = (float)(
              TIM1->ARR /
              motor_control_config.speed_voltage_max_modulation_divisor);
          if (maximum_limit_counts > (float)motor_control_config.
                  speed_voltage_safe_sample_max_counts) {
            maximum_limit_counts = (float)motor_control_config.
                speed_voltage_safe_sample_max_counts;
          }
          float speed_ratio = (float)reference_abs_mdps /
              (float)motor_control_config.speed_voltage_full_scale_mdps;
          if (speed_ratio > 1.0F) { speed_ratio = 1.0F; }
          const float voltage_limit_counts = low_speed_limit_counts +
              (maximum_limit_counts - low_speed_limit_counts) * speed_ratio;
          if (control_speed_voltage_fast_ceiling_counts <= 0) {
            control_speed_voltage_fast_ceiling_counts =
                (int32_t)low_speed_limit_counts;
          } else if ((control_speed_voltage_fast_limit_hold_cycles == 0U) &&
                     (control_speed_voltage_fast_ceiling_counts <
                      (int32_t)voltage_limit_counts)) {
            int32_t recovery = (int32_t)(motor_control_config.
                speed_voltage_limit_recovery_counts_per_second * dt);
            if (recovery < 1) { recovery = 1; }
            control_speed_voltage_fast_ceiling_counts += recovery;
            if (control_speed_voltage_fast_ceiling_counts >
                (int32_t)voltage_limit_counts) {
              control_speed_voltage_fast_ceiling_counts =
                  (int32_t)voltage_limit_counts;
            }
          }
          const float effective_voltage_limit_counts =
              control_speed_voltage_fast_ceiling_counts <
                      (int32_t)voltage_limit_counts ?
                  (float)control_speed_voltage_fast_ceiling_counts :
                  voltage_limit_counts;
          control_speed_voltage_limit_counts =
              (int32_t)effective_voltage_limit_counts;
          const float speed_error_dps = current_foc_speed_reference_dps -
              (float)control_speed_actual_mdps * 0.001F;
          float speed_gain_scale = 1.0F;
          if (reference_abs_mdps > motor_control_config.
                  speed_voltage_gain_reduction_start_mdps) {
            const int32_t reduction_range =
                motor_control_config.maximum_speed_mdps -
                motor_control_config.speed_voltage_gain_reduction_start_mdps;
            float reduction_ratio = reduction_range > 0 ?
                (float)(reference_abs_mdps - motor_control_config.
                    speed_voltage_gain_reduction_start_mdps) /
                    (float)reduction_range : 1.0F;
            if (reduction_ratio > 1.0F) { reduction_ratio = 1.0F; }
            speed_gain_scale = 1.0F - reduction_ratio *
                (1.0F - motor_control_config.
                    speed_voltage_high_speed_gain_scale);
          }
          const float feedforward_counts =
              current_foc_speed_reference_dps *
              motor_control_config.speed_voltage_feedforward_counts_per_dps;
          const float proportional_counts = speed_error_dps *
              motor_control_config.speed_voltage_kp_counts_per_dps *
              speed_gain_scale;
          const float previous_integral =
              control_speed_voltage_integral_counts;
          control_speed_voltage_integral_counts += speed_error_dps *
              motor_control_config.speed_voltage_ki_counts_per_degree *
              speed_gain_scale * dt;
          const float integral_limit = motor_control_config.
              speed_voltage_integral_limit_counts;
          if (control_speed_voltage_integral_counts > integral_limit) {
            control_speed_voltage_integral_counts = integral_limit;
          } else if (control_speed_voltage_integral_counts < -integral_limit) {
            control_speed_voltage_integral_counts = -integral_limit;
          }
          float voltage_q_counts = feedforward_counts +
              proportional_counts + control_speed_voltage_integral_counts;

          /* VESC-style conditional integration: when the requested voltage
           * is saturated in the same direction as the speed error, undo this
           * cycle's integrator update instead of winding it further. */
          if (((voltage_q_counts > effective_voltage_limit_counts) &&
               (speed_error_dps > 0.0F)) ||
              ((voltage_q_counts < -effective_voltage_limit_counts) &&
               (speed_error_dps < 0.0F))) {
            control_speed_voltage_integral_counts = previous_integral;
            voltage_q_counts = feedforward_counts + proportional_counts +
                control_speed_voltage_integral_counts;
          }

          /* The present shunt chain resolves about 80.6 mA/count, so it is
           * unsuitable as a precision torque loop. It can still prevent a
           * voltage command from increasing when the coarse Iq estimate is
           * already beyond the configured current envelope. */
          const bool positive_overcurrent =
              (control_iq_ma > motor_control_config.
                  speed_voltage_slow_current_limit_ma) &&
              (voltage_q_counts > (float)control_speed_voltage_q_counts);
          const bool negative_overcurrent =
              (control_iq_ma < -motor_control_config.
                  speed_voltage_slow_current_limit_ma) &&
              (voltage_q_counts < (float)control_speed_voltage_q_counts);
          control_speed_voltage_current_limited =
              positive_overcurrent || negative_overcurrent ||
              (control_speed_voltage_fast_limit_hold_cycles > 0U);
          if (positive_overcurrent || negative_overcurrent) {
            const float decay = motor_control_config.
                speed_voltage_current_limit_decay_per_second * dt;
            voltage_q_counts = (float)control_speed_voltage_q_counts *
                (decay < 1.0F ? (1.0F - decay) : 0.0F);
          }
          if (voltage_q_counts > effective_voltage_limit_counts) {
            voltage_q_counts = effective_voltage_limit_counts;
          } else if (voltage_q_counts < -effective_voltage_limit_counts) {
            voltage_q_counts = -effective_voltage_limit_counts;
          }
          const float voltage_slew_step = motor_control_config.
              speed_voltage_slew_counts_per_second * dt;
          const float previous_voltage_q =
              (float)control_speed_voltage_q_counts;
          if (voltage_q_counts > previous_voltage_q + voltage_slew_step) {
            voltage_q_counts = previous_voltage_q + voltage_slew_step;
          } else if (voltage_q_counts <
                     previous_voltage_q - voltage_slew_step) {
            voltage_q_counts = previous_voltage_q - voltage_slew_step;
          }
          control_speed_voltage_q_counts = (int32_t)voltage_q_counts;
          current_foc_iq_target = 0.0F;
          control_iq_target_ma = 0;
          const foc_pwm_output_t pwm = foc_oriented_voltage_to_pwm(
              sample.electrical_raw, (int32_t)voltage_q_counts,
              (uint16_t)TIM1->ARR);
          set_compare_values(pwm.a, pwm.b, pwm.c);
        }
      }
      current_foc_outer_last_ms = now;
    }
  }

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
          resistance_phase = RESISTANCE_PHASE_RAMP_DOWN;
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
        current_foc_id_target = RESISTANCE_MEASUREMENT_TARGET_AMPS *
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
      restore_control_timer();
      test_state = POWER_STAGE_TEST_ABORTED;
      test_kind = TEST_KIND_NONE;
    } else if (inductance_phase == INDUCTANCE_PHASE_ENABLE_PENDING) {
      const uint16_t neutral = (uint16_t)(TIM1->ARR / 2U);
      if (!power_stage_enable(neutral, neutral, neutral)) {
        test_state = POWER_STAGE_TEST_ABORTED;
        test_kind = TEST_KIND_NONE;
      } else {
        inductance_phase = INDUCTANCE_PHASE_PULSING;
        configure_vesc_inductance_timer();
      }
    } else if (inductance_phase == INDUCTANCE_PHASE_COMPLETE) {
      power_stage_disable();
      restore_control_timer();
      test_state = inductance_valid ? POWER_STAGE_TEST_COMPLETED :
                                      POWER_STAGE_TEST_ABORTED;
      test_kind = TEST_KIND_NONE;
    } else if ((inductance_phase !=
                INDUCTANCE_PHASE_OFFSET_CALIBRATION) &&
               (stage_state != POWER_STAGE_RUNNING)) {
      power_stage_disable();
      restore_control_timer();
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
    const uint32_t alignment_elapsed =
        (uint32_t)(now - test_started_at);
    if (alignment_elapsed < CURRENT_FOC_ALIGNMENT_RAMP_UP_MS) {
      current_foc_id_target = CURRENT_FOC_MAX_TARGET_AMPS *
          (float)alignment_elapsed /
          (float)CURRENT_FOC_ALIGNMENT_RAMP_UP_MS;
    } else {
      current_foc_id_target = CURRENT_FOC_MAX_TARGET_AMPS;
    }
    if ((stage_state != POWER_STAGE_RUNNING) ||
        (alignment_elapsed >= ENCODER_ALIGNMENT_TIMEOUT_MS)) {
      power_stage_disable();
      test_state = POWER_STAGE_TEST_ABORTED;
      test_kind = TEST_KIND_NONE;
    } else if (alignment_elapsed >= ENCODER_ALIGNMENT_REVERSE_END_MS) {
      angle_sensor_sample_t sample;
      if ((test_alignment_offset_samples < 100U) ||
          !angle_sensor_get_sample(&sample)) {
        power_stage_disable();
        test_state = POWER_STAGE_TEST_ABORTED;
        test_kind = TEST_KIND_NONE;
      } else {
        float zero_angle = atan2f(test_alignment_offset_sin_sum,
                                  test_alignment_offset_cos_sum);
        if (zero_angle < 0.0F) {
          zero_angle += 6.28318530718F;
        }
        const uint16_t averaged_zero_raw = (uint16_t)(
            (uint32_t)(zero_angle * (4096.0F / 6.28318530718F) + 0.5F) &
            0x0FFFU);
        const uint16_t target_raw = (uint16_t)(
            (sample.electrical_raw_unaligned - averaged_zero_raw) &
            0x0FFFU);
        test_alignment_calibrated =
            angle_sensor_calibrate_electrical_zero(target_raw);
        power_stage_disable();
        test_state = test_alignment_calibrated ?
            POWER_STAGE_TEST_COMPLETED : POWER_STAGE_TEST_ABORTED;
        test_kind = TEST_KIND_NONE;
      }
    } else if (alignment_elapsed >= ENCODER_ALIGNMENT_SWEEP_START_MS) {
      uint32_t sweep_raw;
      if (alignment_elapsed < ENCODER_ALIGNMENT_FORWARD_END_MS) {
        sweep_raw = ((alignment_elapsed - ENCODER_ALIGNMENT_SWEEP_START_MS) *
                     8192U) /
                    (ENCODER_ALIGNMENT_FORWARD_END_MS -
                     ENCODER_ALIGNMENT_SWEEP_START_MS);
      } else {
        sweep_raw = ((ENCODER_ALIGNMENT_REVERSE_END_MS - alignment_elapsed) *
                     8192U) /
                    (ENCODER_ALIGNMENT_REVERSE_END_MS -
                     ENCODER_ALIGNMENT_FORWARD_END_MS);
      }
      const uint16_t command_raw = (uint16_t)(sweep_raw & 0x0FFFU);
      test_command_electrical_raw = command_raw;
      angle_sensor_sample_t sample;
      if (angle_sensor_get_sample(&sample) &&
          ((uint32_t)(now - sample.timestamp_ms) <= 20U)) {
        const uint16_t offset_raw = (uint16_t)(
            (sample.electrical_raw_unaligned - command_raw) & 0x0FFFU);
        test_alignment_offset_sin_sum += foc_sine_raw(offset_raw);
        test_alignment_offset_cos_sum +=
            foc_sine_raw((uint16_t)(offset_raw + 1024U));
        ++test_alignment_offset_samples;
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
  if (control_command_timeout_latched) {
    return false;
  }
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
  test_alignment_offset_sin_sum = 0.0F;
  test_alignment_offset_cos_sum = 0.0F;
  test_alignment_offset_samples = 0U;
  test_command_electrical_raw = 0U;
  reset_foc_control_statistics();
  current_foc_integral_d = 0.0F;
  current_foc_integral_q = 0.0F;
  current_foc_id_target = 0.0F;
  current_foc_iq_target = 0.0F;
  /* Start from a neutral vector. The 20 kHz d-axis current controller ramps
   * the locking current over 600 ms, avoiding a simultaneous voltage and
   * current-reference step when the MOSFET bridge is enabled. */
  if (!power_stage_enable(neutral, neutral, neutral)) {
    test_state = POWER_STAGE_TEST_ABORTED;
    test_kind = TEST_KIND_NONE;
    return false;
  }
  current_foc_active = true;
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
    current_foc_speed_iq_command_amps = 0.0F;
    control_speed_actual_mdps = 0;
    control_speed_sample_position_counts = sample.position_counts;
    control_speed_sample_timestamp_ms = sample.timestamp_ms;
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

bool power_stage_set_iq_current_ma(int32_t iq_target_ma)
{
  const uint32_t now = HAL_GetTick();
  const uint16_t neutral = (uint16_t)(TIM1->ARR / 2U);
  angle_sensor_sample_t sample;

  if ((iq_target_ma > CONTROL_MAX_IQ_MA) ||
      (iq_target_ma < -CONTROL_MAX_IQ_MA)) {
    return false;
  }
  if ((iq_target_ma > -CONTROL_MIN_ACTIVE_IQ_MA) &&
      (iq_target_ma < CONTROL_MIN_ACTIVE_IQ_MA)) {
    power_stage_disable();
    return true;
  }
  if ((control_mode != POWER_STAGE_CONTROL_DISABLED) &&
      (control_mode != POWER_STAGE_CONTROL_IQ_CURRENT)) {
    power_stage_disable();
  }
  if ((test_state == POWER_STAGE_TEST_RUNNING) ||
      ((stage_state != POWER_STAGE_READY) &&
       (control_mode != POWER_STAGE_CONTROL_IQ_CURRENT)) ||
      !bus_voltage_valid ||
      (bus_voltage_mv < COMMISSIONING_MIN_BUS_MV) ||
      (bus_voltage_mv > COMMISSIONING_MAX_BUS_MV) ||
      (latched_faults != 0U) ||
      !angle_sensor_get_sample(&sample) ||
      !sample.electrical_zero_calibrated ||
      ((uint32_t)(now - sample.timestamp_ms) >
       ENCODER_CONTROL_MAX_SAMPLE_AGE_MS)) {
    return false;
  }

  if (control_mode != POWER_STAGE_CONTROL_IQ_CURRENT) {
    reset_foc_control_statistics();
    current_foc_integral_d = 0.0F;
    current_foc_integral_q = 0.0F;
    current_foc_id_target = 0.0F;
    current_foc_iq_target = 0.0F;
    overcurrent_count = 0U;
    speed_voltage_phase_limit_count = 0U;
    if (!power_stage_enable(neutral, neutral, neutral)) {
      return false;
    }
    current_foc_active = true;
    control_mode = POWER_STAGE_CONTROL_IQ_CURRENT;
  }

  control_iq_target_ma = iq_target_ma;
  current_foc_iq_target = (float)iq_target_ma * 0.001F;
  control_command_at_ms = now;
  return true;
}

bool power_stage_set_torque_millinewton_metres(int32_t torque_target_mnm)
{
  /* T[mN*m] / Kt[N*m/A] is numerically equal to I[mA]. */
  const int32_t maximum_torque_mnm = (int32_t)(
      (float)motor_control_config.maximum_iq_ma *
      motor_control_config.torque_constant_nm_per_amp);
  if (torque_target_mnm < -maximum_torque_mnm ||
      torque_target_mnm > maximum_torque_mnm) {
    return false;
  }
  const int32_t iq_target_ma = (int32_t)lroundf(
      (float)torque_target_mnm /
      motor_control_config.torque_constant_nm_per_amp);
  return power_stage_set_iq_current_ma(iq_target_ma);
}

bool power_stage_set_speed_millidegrees_per_second(int32_t speed_target)
{
  const uint32_t now = HAL_GetTick();
  const uint16_t neutral = (uint16_t)(TIM1->ARR / 2U);
  angle_sensor_sample_t sample;

  if ((speed_target > (int32_t)(CONTROL_SPEED_MAX_DPS * 1000.0F)) ||
      (speed_target < (int32_t)(-CONTROL_SPEED_MAX_DPS * 1000.0F))) {
    return false;
  }
  if ((speed_target > -motor_control_config.minimum_speed_mdps) &&
      (speed_target < motor_control_config.minimum_speed_mdps)) {
    power_stage_disable();
    return true;
  }
  if ((control_mode != POWER_STAGE_CONTROL_DISABLED) &&
      (control_mode != POWER_STAGE_CONTROL_SPEED)) {
    power_stage_disable();
  }
  /* Once speed control is running, repeated host commands are watchdog and
   * setpoint refreshes. Do not reject one merely because the lock-free
   * encoder snapshot was being published during this exact protocol call;
   * the control loop independently validates every subsequent sample. */
  if (control_mode == POWER_STAGE_CONTROL_SPEED) {
    if ((stage_state != POWER_STAGE_RUNNING) || !current_foc_active ||
        (latched_faults != 0U) || !bus_voltage_valid ||
        (bus_voltage_mv < COMMISSIONING_MIN_BUS_MV) ||
        (bus_voltage_mv > COMMISSIONING_MAX_BUS_MV)) {
      return false;
    }
    control_speed_target_mdps = speed_target;
    control_command_at_ms = now;
    return true;
  }
  if ((test_state == POWER_STAGE_TEST_RUNNING) ||
      ((stage_state != POWER_STAGE_READY) &&
       (control_mode != POWER_STAGE_CONTROL_SPEED)) ||
      !bus_voltage_valid ||
      (bus_voltage_mv < COMMISSIONING_MIN_BUS_MV) ||
      (bus_voltage_mv > COMMISSIONING_MAX_BUS_MV) ||
      (latched_faults != 0U) ||
      !angle_sensor_get_sample(&sample) ||
      !sample.electrical_zero_calibrated ||
      ((uint32_t)(now - sample.timestamp_ms) >
       ENCODER_CONTROL_MAX_SAMPLE_AGE_MS)) {
    return false;
  }

  if (control_mode != POWER_STAGE_CONTROL_SPEED) {
    reset_foc_control_statistics();
    current_foc_integral_d = 0.0F;
    current_foc_integral_q = 0.0F;
    current_foc_id_target = 0.0F;
    current_foc_iq_target = 0.0F;
    current_foc_pll_position_counts = (float)sample.position_counts;
    current_foc_pll_speed_counts_per_second = 0.0F;
    current_foc_speed_reference_dps = 0.0F;
    current_foc_target_position_counts = (float)sample.position_counts;
    current_foc_speed_integral_amps = 0.0F;
    control_speed_voltage_q_counts = 0;
    control_speed_voltage_integral_counts = 0.0F;
    control_speed_voltage_fast_ceiling_counts =
        (int32_t)(TIM1->ARR / DIRECT_VOLTAGE_MODULATION_DIVISOR);
    control_speed_voltage_fast_limit_hold_cycles = 0U;
    control_speed_voltage_current_limited = false;
    control_speed_actual_mdps = 0;
    control_speed_sample_position_counts = sample.position_counts;
    control_speed_sample_timestamp_ms = sample.timestamp_ms;
    control_pll_phase_rad = 0.0F;
    control_pll_speed_electrical_rad_per_second = 0.0F;
    control_pll_initialized = false;
    control_pll_sensor_sequence = 0U;
    foc_observer_reset(&speed_observer,
        (float)sample.electrical_raw * (6.28318530718F / 4096.0F));
    speed_observer_phase_raw = sample.electrical_raw;
    speed_observer_encoder_error_raw = 0;
    speed_observer_erpm = 0;
    speed_observer_using_encoder = true;
    precise_encoder_phase_raw = (float)sample.electrical_raw;
    precise_encoder_last_cycles = DWT->CYCCNT;
    precise_encoder_sensor_sequence = 0U;
    precise_encoder_initialized = false;
    precise_encoder_blend = 0.0F;
    precise_encoder_requested = false;
    current_foc_outer_last_ms = now;
    overcurrent_count = 0U;
    speed_voltage_phase_limit_count = 0U;
    if (!power_stage_enable(neutral, neutral, neutral)) {
      return false;
    }
    current_foc_active = true;
    control_mode = POWER_STAGE_CONTROL_SPEED;
  }

  control_speed_target_mdps = speed_target;
  control_command_at_ms = now;
  return true;
}

bool power_stage_set_position_millidegrees(int32_t position_target)
{
  const uint32_t now = HAL_GetTick();
  const uint16_t neutral = (uint16_t)(TIM1->ARR / 2U);
  angle_sensor_sample_t sample;

  if ((position_target < -motor_control_config.maximum_position_mdeg) ||
      (position_target > motor_control_config.maximum_position_mdeg)) {
    return false;
  }
  if ((control_mode != POWER_STAGE_CONTROL_DISABLED) &&
      (control_mode != POWER_STAGE_CONTROL_POSITION)) {
    power_stage_disable();
  }
  if ((test_state == POWER_STAGE_TEST_RUNNING) ||
      ((stage_state != POWER_STAGE_READY) &&
       (control_mode != POWER_STAGE_CONTROL_POSITION)) ||
      !bus_voltage_valid ||
      (bus_voltage_mv < COMMISSIONING_MIN_BUS_MV) ||
      (bus_voltage_mv > COMMISSIONING_MAX_BUS_MV) ||
      (latched_faults != 0U) ||
      !angle_sensor_get_sample(&sample) ||
      !sample.electrical_zero_calibrated ||
      ((uint32_t)(now - sample.timestamp_ms) >
       ENCODER_CONTROL_MAX_SAMPLE_AGE_MS)) {
    return false;
  }

  if (control_mode != POWER_STAGE_CONTROL_POSITION) {
    reset_foc_control_statistics();
    current_foc_integral_d = 0.0F;
    current_foc_integral_q = 0.0F;
    current_foc_id_target = 0.0F;
    current_foc_iq_target = 0.0F;
    current_foc_pll_position_counts = (float)sample.position_counts;
    current_foc_pll_speed_counts_per_second = 0.0F;
    current_foc_speed_reference_dps = 0.0F;
    current_foc_speed_integral_amps = 0.0F;
    current_foc_outer_last_ms = now;
    control_speed_actual_mdps = 0;
    control_speed_sample_position_counts = sample.position_counts;
    control_speed_sample_timestamp_ms = sample.timestamp_ms;
    control_speed_voltage_q_counts = 0;
    control_speed_voltage_integral_counts = 0.0F;
    control_speed_voltage_fast_ceiling_counts =
        (int32_t)(TIM1->ARR / DIRECT_VOLTAGE_MODULATION_DIVISOR);
    control_speed_voltage_fast_limit_hold_cycles = 0U;
    control_speed_voltage_current_limited = false;
    control_speed_current_foc = false;
    foc_observer_reset(&speed_observer,
        (float)sample.electrical_raw * (6.28318530718F / 4096.0F));
    precise_encoder_phase_raw = (float)sample.electrical_raw;
    precise_encoder_last_cycles = DWT->CYCCNT;
    precise_encoder_sensor_sequence = 0U;
    precise_encoder_initialized = false;
    precise_encoder_blend = 0.0F;
    precise_encoder_requested = false;
    overcurrent_count = 0U;
    if (!power_stage_enable(neutral, neutral, neutral)) {
      return false;
    }
    current_foc_active = true;
    control_mode = POWER_STAGE_CONTROL_POSITION;
  }

  control_position_target_mdeg = position_target;
  control_position_actual_mdeg = (int32_t)(
      (int64_t)sample.position_counts * 360000LL / 4096LL);
  control_command_at_ms = now;
  return true;
}

bool power_stage_set_position_profile(int32_t position_target,
                                      int32_t maximum_speed_mdps,
                                      int32_t acceleration_mdps2,
                                      int32_t deceleration_mdps2)
{
  if (acceleration_mdps2 == 0) {
    acceleration_mdps2 =
        motor_control_config.default_profile_acceleration_mdps2;
  }
  if (deceleration_mdps2 == 0) {
    deceleration_mdps2 =
        motor_control_config.default_profile_acceleration_mdps2;
  }
  if (maximum_speed_mdps < motor_control_config.minimum_speed_mdps ||
      maximum_speed_mdps > motor_control_config.maximum_profile_speed_mdps ||
      acceleration_mdps2 <
          motor_control_config.minimum_profile_acceleration_mdps2 ||
      acceleration_mdps2 >
          motor_control_config.maximum_profile_acceleration_mdps2 ||
      deceleration_mdps2 <
          motor_control_config.minimum_profile_acceleration_mdps2 ||
      deceleration_mdps2 >
          motor_control_config.maximum_profile_acceleration_mdps2 ||
      position_target < -motor_control_config.maximum_position_mdeg ||
      position_target > motor_control_config.maximum_position_mdeg) {
    return false;
  }
  if (control_mode == POWER_STAGE_CONTROL_POSITION_PROFILE) {
    control_position_target_mdeg = position_target;
    control_position_profile_speed_mdps = maximum_speed_mdps;
    control_position_profile_acceleration_mdps2 = acceleration_mdps2;
    control_position_profile_deceleration_mdps2 = deceleration_mdps2;
    control_command_at_ms = HAL_GetTick();
    return true;
  }
  angle_sensor_sample_t sample;
  if (!angle_sensor_get_sample(&sample)) { return false; }
  const int32_t actual_mdeg = (int32_t)(
      (int64_t)sample.position_counts * 360000LL / 4096LL);
  if (!power_stage_set_position_millidegrees(actual_mdeg)) { return false; }
  control_position_profile_mdeg = actual_mdeg;
  control_position_target_mdeg = position_target;
  control_position_profile_speed_mdps = maximum_speed_mdps;
  control_position_profile_acceleration_mdps2 = acceleration_mdps2;
  control_position_profile_deceleration_mdps2 = deceleration_mdps2;
  control_position_profile_velocity_mdps = 0.0F;
  control_mode = POWER_STAGE_CONTROL_POSITION_PROFILE;
  control_command_at_ms = HAL_GetTick();
  return true;
}

bool power_stage_set_haptic(int32_t detent_spacing_mdeg,
                            int32_t detent_strength_ma,
                            int32_t damping_ma_per_dps,
                            int32_t minimum_position_mdeg,
                            int32_t maximum_position_mdeg)
{
  if (detent_spacing_mdeg < 100 || detent_spacing_mdeg > 360000 ||
      detent_strength_ma < 0 ||
      detent_strength_ma > motor_control_config.maximum_iq_ma ||
      damping_ma_per_dps < 0 || damping_ma_per_dps > 100 ||
      minimum_position_mdeg >= maximum_position_mdeg ||
      minimum_position_mdeg < -motor_control_config.maximum_position_mdeg ||
      maximum_position_mdeg > motor_control_config.maximum_position_mdeg) {
    return false;
  }
  if (control_mode != POWER_STAGE_CONTROL_HAPTIC) {
    angle_sensor_sample_t sample;
    if (!angle_sensor_get_sample(&sample)) { return false; }
    const int32_t actual_mdeg = (int32_t)(
        (int64_t)sample.position_counts * 360000LL / 4096LL);
    if (!power_stage_set_position_millidegrees(actual_mdeg)) { return false; }
    control_mode = POWER_STAGE_CONTROL_HAPTIC;
  }
  control_haptic_spacing_mdeg = detent_spacing_mdeg;
  control_haptic_strength_ma = detent_strength_ma;
  control_haptic_damping_ma_per_dps = damping_ma_per_dps;
  control_haptic_minimum_mdeg = minimum_position_mdeg;
  control_haptic_maximum_mdeg = maximum_position_mdeg;
  control_command_at_ms = HAL_GetTick();
  return true;
}

void power_stage_stop_commissioning_test(void)
{
  if (control_mode != POWER_STAGE_CONTROL_DISABLED) {
    power_stage_disable();
  }
  if (test_state == POWER_STAGE_TEST_RUNNING) {
    power_stage_disable();
    test_state = POWER_STAGE_TEST_ABORTED;
    test_kind = TEST_KIND_NONE;
  }
  control_command_timeout_latched = false;
}

bool power_stage_is_command_timeout_latched(void)
{
  return control_command_timeout_latched;
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
  if (sample == NULL) {
    return false;
  }

  /* Diagnostics run in the main loop while the ADC ISR publishes at 20 kHz.
   * Never let a telemetry request spin indefinitely waiting for a quiet
   * window; a later protocol frame can retry a missed snapshot. */
  for (uint32_t attempt = 0U; attempt < 3U; ++attempt) {
    const uint32_t sequence_before = current_sequence;
    if ((sequence_before & 1U) != 0U) {
      continue;
    }
    for (uint32_t index = 0U; index < 3U; ++index) {
      sample->raw[index] = current_raw[index];
      sample->offset[index] = current_offset[index];
      sample->centered[index] =
          (int32_t)sample->raw[index] - (int32_t)sample->offset[index];
    }
    const uint32_t sequence_after = current_sequence;
    if ((sequence_before == sequence_after) &&
        ((sequence_after & 1U) == 0U)) {
      sample->sequence = sequence_after >> 1U;
      return true;
    }
  }

  return false;
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
  diagnostics->control_mode = control_mode;
  diagnostics->control_id_ma = control_id_ma;
  diagnostics->control_iq_ma = control_iq_ma;
  diagnostics->control_iq_target_ma = control_iq_target_ma;
  diagnostics->control_speed_voltage_q_counts =
      control_speed_voltage_q_counts;
  diagnostics->control_speed_voltage_limit_counts =
      control_speed_voltage_limit_counts;
  diagnostics->control_speed_voltage_current_limited =
      control_speed_voltage_current_limited ? 1U : 0U;
  diagnostics->observer_phase_raw = speed_observer_phase_raw;
  diagnostics->observer_encoder_error_raw =
      speed_observer_encoder_error_raw;
  diagnostics->observer_erpm = speed_observer_erpm;
  diagnostics->observer_using_encoder =
      speed_observer_using_encoder ? 1U : 0U;
  diagnostics->control_timeout_remaining_ms =
      control_mode == POWER_STAGE_CONTROL_DISABLED ? 0U :
      ((uint32_t)(HAL_GetTick() - control_command_at_ms) >=
       CONTROL_COMMAND_TIMEOUT_MS ? 0U :
       CONTROL_COMMAND_TIMEOUT_MS -
       (uint32_t)(HAL_GetTick() - control_command_at_ms));
  diagnostics->control_speed_target_millidegrees_per_second =
      control_speed_target_mdps;
  diagnostics->control_speed_millidegrees_per_second =
      control_speed_actual_mdps;
  diagnostics->control_position_target_millidegrees =
      control_position_target_mdeg;
  diagnostics->control_position_millidegrees =
      control_position_actual_mdeg;
  diagnostics->control_speed_current_foc =
      control_speed_current_foc ? 1U : 0U;
  diagnostics->control_speed_reference_millidegrees_per_second = (int32_t)(
      current_foc_speed_reference_dps * 1000.0F);
  diagnostics->control_predicted_electrical_raw =
      control_pll_predicted_electrical_raw;
  const uint32_t now_ms = HAL_GetTick();
  angle_sensor_sample_t latest_angle_sample;
  diagnostics->control_encoder_sample_age_ms =
      angle_sensor_get_sample(&latest_angle_sample) ?
          now_ms - latest_angle_sample.timestamp_ms : UINT32_MAX;
  uint16_t measured_electrical_raw = 0U;
  (void)angle_sensor_get_electrical_raw_fast(&measured_electrical_raw);
  int32_t prediction_error =
      (int32_t)control_pll_predicted_electrical_raw -
      (int32_t)measured_electrical_raw;
  if (prediction_error > 2047) {
    prediction_error -= 4096;
  } else if (prediction_error < -2048) {
    prediction_error += 4096;
  }
  diagnostics->control_prediction_error_raw = (int16_t)prediction_error;
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

  if (control_speed_voltage_fast_limit_hold_cycles > 0U) {
    --control_speed_voltage_fast_limit_hold_cycles;
  }

  ++current_sequence;
  for (uint32_t index = 0U; index < 3U; ++index) {
    current_raw[index] = samples[index];
  }
  ++current_sequence;

  /* This deadline is enforced in the 20 kHz ADC interrupt, independently of
   * USB and the RTOS status task. A stalled protocol/telemetry path therefore
   * cannot leave an externally commanded motor energized indefinitely. */
  if ((control_mode != POWER_STAGE_CONTROL_DISABLED) &&
      ((uint32_t)(HAL_GetTick() - control_command_at_ms) >=
       CONTROL_COMMAND_TIMEOUT_MS)) {
    control_command_timeout_latched = true;
    control_mode = POWER_STAGE_CONTROL_DISABLED;
    current_foc_active = false;
    current_foc_id_target = 0.0F;
    current_foc_iq_target = 0.0F;
    current_foc_integral_d = 0.0F;
    current_foc_integral_q = 0.0F;
    control_speed_target_mdps = 0;
    control_speed_voltage_q_counts = 0;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);
    DRV_EN_GATE_GPIO_Port->BSRR =
        (uint32_t)DRV_EN_GATE_Pin << 16U;
    if ((TIM1->CCER & TIM_CCER_CC4E) != 0U) {
      TIM1->BDTR |= TIM_BDTR_MOE;
    }
    if (stage_state == POWER_STAGE_RUNNING) {
      stage_state = POWER_STAGE_READY;
    }
  }

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
    /* At the fixed low-side-shunt sampling point, one phase can be hidden by
     * the active SVM vector. Match the reconstruction selection used below
     * and never treat that unobservable channel's switching transient as a
     * real phase overcurrent. The remaining two measured phases still fully
     * determine the motor currents. */
    const bool current_sample_v7 =
        (test_kind != TEST_KIND_RESISTANCE) &&
        (test_kind != TEST_KIND_INDUCTANCE) &&
        ((TIM1->CR1 & TIM_CR1_DIR) == 0U);
    uint32_t unobservable_phase = 3U;
    if (current_sample_v7) {
      if ((TIM1->CCR1 < TIM1->CCR2) && (TIM1->CCR1 < TIM1->CCR3)) {
        unobservable_phase = 0U;
      } else if ((TIM1->CCR2 < TIM1->CCR1) &&
                 (TIM1->CCR2 < TIM1->CCR3)) {
        unobservable_phase = 1U;
      } else if ((TIM1->CCR3 < TIM1->CCR1) &&
                 (TIM1->CCR3 < TIM1->CCR2)) {
        unobservable_phase = 2U;
      }
    } else {
      if ((TIM1->CCR1 > TIM1->CCR2) && (TIM1->CCR1 > TIM1->CCR3)) {
        unobservable_phase = 0U;
      } else if ((TIM1->CCR2 > TIM1->CCR1) &&
                 (TIM1->CCR2 > TIM1->CCR3)) {
        unobservable_phase = 1U;
      } else if ((TIM1->CCR3 > TIM1->CCR1) &&
                 (TIM1->CCR3 > TIM1->CCR2)) {
        unobservable_phase = 2U;
      }
    }
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
      if ((index != unobservable_phase) &&
          ((centered > current_trip_counts) ||
           (centered < -current_trip_counts))) {
        overcurrent = true;
      }
    }

    /* VESC-style fast protection for encoder-feedback voltage modes. The
     * slower d/q diagnostic is intentionally decimated and can miss PWM-rate
     * phase-current peaks. Back the commanded voltage off in this ISR before
     * the independent hard-trip threshold is reached. */
    const bool direct_speed_voltage_control =
        (control_mode == POWER_STAGE_CONTROL_SPEED) &&
        !control_speed_current_foc && current_foc_active;
    if (direct_speed_voltage_control) {
      bool phase_current_limited = false;
      const int32_t phase_limit = (int32_t)motor_control_config.
          speed_voltage_phase_current_limit_adc_counts;
      for (uint32_t index = 0U; index < 3U; ++index) {
        if ((index != unobservable_phase) &&
            ((centered_samples[index] > phase_limit) ||
             (centered_samples[index] < -phase_limit))) {
          phase_current_limited = true;
        }
      }
      speed_voltage_phase_limit_count = phase_current_limited ?
          (uint8_t)(speed_voltage_phase_limit_count + 1U) : 0U;
      if (speed_voltage_phase_limit_count >=
          CURRENT_TRIP_CONSECUTIVE_SAMPLES) {
        speed_voltage_phase_limit_count = 0U;
        const uint8_t decay_shift = motor_control_config.
            speed_voltage_fast_limit_decay_shift;
        int32_t voltage_q = control_speed_voltage_q_counts;
        int32_t reduction = decay_shift < 31U ?
            (voltage_q < 0 ? -voltage_q : voltage_q) >> decay_shift : 0;
        if (reduction < 1) { reduction = 1; }
        if (voltage_q > 0) {
          voltage_q = voltage_q > reduction ? voltage_q - reduction : 0;
        } else if (voltage_q < 0) {
          voltage_q = voltage_q < -reduction ? voltage_q + reduction : 0;
        }
        control_speed_voltage_q_counts = voltage_q;
        if (control_speed_voltage_fast_ceiling_counts >
            (voltage_q < 0 ? -voltage_q : voltage_q)) {
          control_speed_voltage_fast_ceiling_counts =
              voltage_q < 0 ? -voltage_q : voltage_q;
        }
        uint32_t hold_cycles = (uint32_t)motor_control_config.
            speed_voltage_fast_limit_hold_ms * 20U;
        if (hold_cycles > UINT16_MAX) { hold_cycles = UINT16_MAX; }
        control_speed_voltage_fast_limit_hold_cycles =
            (uint16_t)hold_cycles;
        control_speed_voltage_current_limited = true;
      }
    }
    ++test_current_samples;
    const uint16_t balance_abs = (uint16_t)(balance < 0 ? -balance : balance);
    test_current_balance_abs_sum += balance_abs;
    if (balance_abs > test_current_balance_abs_max) {
      test_current_balance_abs_max = balance_abs;
    }

    /* Passive VESC observer validation. It sees the voltage vector that was
     * active during this ADC interval and reconstructed phase currents, but
     * its selected phase is not yet allowed to drive PWM. */
    if (motor_control_config.observer_diagnostic_enabled &&
        current_foc_active &&
        (control_mode == POWER_STAGE_CONTROL_SPEED) &&
        !control_speed_current_foc && (bus_voltage_mv > 0U)) {
      int32_t observer_current_counts[3] = {
          centered_samples[0], centered_samples[1], centered_samples[2]};
      if (unobservable_phase < 3U) {
        const uint32_t other_1 = (unobservable_phase + 1U) % 3U;
        const uint32_t other_2 = (unobservable_phase + 2U) % 3U;
        observer_current_counts[unobservable_phase] =
            -(observer_current_counts[other_1] +
              observer_current_counts[other_2]);
      }
      const float current_alpha =
          (float)observer_current_counts[0] * CURRENT_ADC_TO_AMPS;
      const float current_beta =
          ((float)observer_current_counts[0] +
           2.0F * (float)observer_current_counts[1]) *
          CURRENT_ADC_TO_AMPS * 0.57735026919F;
      const float inverse_arr = 1.0F / (float)TIM1->ARR;
      const float bus_voltage = (float)bus_voltage_mv * 0.001F;
      const float duty_a = (float)TIM1->CCR1 * inverse_arr;
      const float duty_b = (float)TIM1->CCR2 * inverse_arr;
      const float duty_c = (float)TIM1->CCR3 * inverse_arr;
      const float voltage_alpha = bus_voltage *
          (2.0F * duty_a - duty_b - duty_c) / 3.0F;
      const float voltage_beta = bus_voltage *
          (duty_b - duty_c) * 0.57735026919F;
      float duty_abs = sqrtf(voltage_alpha * voltage_alpha +
                             voltage_beta * voltage_beta) /
          (0.66666666667F * bus_voltage);
      if (duty_abs > 1.0F) { duty_abs = 1.0F; }
      foc_observer_update(&speed_observer, voltage_alpha, voltage_beta,
                          current_alpha, current_beta, duty_abs,
                          CURRENT_FOC_DT_SECONDS);
      foc_observer_run_pll(&speed_observer, CURRENT_FOC_DT_SECONDS);
      uint16_t encoder_electrical_raw;
      if (angle_sensor_get_electrical_raw_fast(&encoder_electrical_raw)) {
        const float encoder_phase_rad =
            (float)encoder_electrical_raw *
            (6.28318530718F / 4096.0F);
        (void)foc_observer_select_phase(&speed_observer,
                                       encoder_phase_rad);
        speed_observer_phase_raw = (uint16_t)(
            (speed_observer.phase_rad < 0.0F ?
             speed_observer.phase_rad + 6.28318530718F :
             speed_observer.phase_rad) *
            (4096.0F / 6.28318530718F)) & 0x0FFFU;
        int32_t observer_error =
            (int32_t)speed_observer_phase_raw -
            (int32_t)encoder_electrical_raw;
        if (observer_error > 2047) { observer_error -= 4096; }
        else if (observer_error < -2048) { observer_error += 4096; }
        speed_observer_encoder_error_raw = (int16_t)observer_error;
        speed_observer_erpm = (int32_t)(
            speed_observer.pll_speed_rad_per_second *
            (60.0F / 6.28318530718F));
        speed_observer_using_encoder = speed_observer.using_encoder;
      }
    }

    const bool direct_voltage_control = current_foc_active &&
        ((((control_mode == POWER_STAGE_CONTROL_SPEED) ||
           (control_mode == POWER_STAGE_CONTROL_POSITION_PROFILE)) &&
          !control_speed_current_foc) ||
         (control_mode == POWER_STAGE_CONTROL_POSITION));
    const bool current_foc_required = current_foc_active &&
        (control_mode != POWER_STAGE_CONTROL_DISABLED) &&
        !direct_voltage_control;
    /* Direct voltage speed control does not otherwise enter the d/q current
     * transform block below. Update its predicted electrical angle and PWM
     * independently on every ADC cycle, using the latest lock-free encoder
     * snapshot and the filtered mechanical PLL speed. */
    if (direct_voltage_control &&
        ((control_mode == POWER_STAGE_CONTROL_SPEED) ||
         (control_mode == POWER_STAGE_CONTROL_POSITION_PROFILE))) {
      uint16_t measured_electrical_raw;
      uint32_t electrical_timestamp_ms;
      uint32_t electrical_timestamp_cycles;
      uint32_t electrical_sequence;
      if (angle_sensor_get_electrical_sample_precise(
              &measured_electrical_raw, &electrical_timestamp_ms,
              &electrical_timestamp_cycles, &electrical_sequence)) {
        uint16_t commanded_electrical_raw = measured_electrical_raw;
        if (motor_control_config.high_speed_pll_prediction_enabled) {
          uint32_t sample_age_ms =
              (uint32_t)(HAL_GetTick() - electrical_timestamp_ms);
          if (sample_age_ms > 10U) { sample_age_ms = 10U; }
          const float prediction_seconds =
              (float)sample_age_ms * 0.001F +
              motor_control_config.encoder_phase_advance_seconds;
          const int32_t advance_raw = (int32_t)lroundf(
              (float)control_speed_actual_mdps * 0.001F *
              CONTROL_MOTOR_POLE_PAIRS * prediction_seconds *
              (4096.0F / 360.0F));
          commanded_electrical_raw =
              (uint16_t)((int32_t)measured_electrical_raw + advance_raw) &
              0x0FFFU;
        }
        control_pll_predicted_electrical_raw = commanded_electrical_raw;

        /* Hardware-timestamped continuous encoder PLL. DWT removes the 1 ms
         * quantization that caused the first interpolation experiment to
         * lose phase at high speed. Its PWM contribution is blended below. */
        const uint32_t now_cycles = DWT->CYCCNT;
        const float raw_per_second =
            (float)control_speed_actual_mdps * 0.001F *
            CONTROL_MOTOR_POLE_PAIRS * (4096.0F / 360.0F);
        const float elapsed_seconds = precise_encoder_initialized ?
            (float)(uint32_t)(now_cycles - precise_encoder_last_cycles) /
                (float)SystemCoreClock : 0.0F;
        if (!precise_encoder_initialized || elapsed_seconds > 0.01F) {
          precise_encoder_phase_raw = (float)measured_electrical_raw;
          precise_encoder_sensor_sequence = electrical_sequence;
          precise_encoder_initialized = true;
        } else {
          precise_encoder_phase_raw += raw_per_second * elapsed_seconds;
          while (precise_encoder_phase_raw >= 4096.0F) {
            precise_encoder_phase_raw -= 4096.0F;
          }
          while (precise_encoder_phase_raw < 0.0F) {
            precise_encoder_phase_raw += 4096.0F;
          }
          if (electrical_sequence != precise_encoder_sensor_sequence) {
            const float sample_age_seconds =
                (float)(uint32_t)(now_cycles - electrical_timestamp_cycles) /
                    (float)SystemCoreClock +
                motor_control_config.encoder_phase_advance_seconds;
            float measured_now_raw = (float)measured_electrical_raw +
                raw_per_second * sample_age_seconds;
            while (measured_now_raw >= 4096.0F) {
              measured_now_raw -= 4096.0F;
            }
            while (measured_now_raw < 0.0F) {
              measured_now_raw += 4096.0F;
            }
            float phase_error = measured_now_raw -
                precise_encoder_phase_raw;
            if (phase_error > 2048.0F) { phase_error -= 4096.0F; }
            else if (phase_error < -2048.0F) { phase_error += 4096.0F; }
            speed_observer_encoder_error_raw =
                (int16_t)lroundf(phase_error);
            if (fabsf(phase_error) <= (float)motor_control_config.
                    encoder_precise_max_error_raw) {
              float phase_correction = phase_error *
                  motor_control_config.encoder_precise_phase_correction_gain;
              const float max_correction = motor_control_config.
                  encoder_precise_max_correction_raw;
              if (phase_correction > max_correction) {
                phase_correction = max_correction;
              } else if (phase_correction < -max_correction) {
                phase_correction = -max_correction;
              }
              precise_encoder_phase_raw += phase_correction;
            }
            precise_encoder_sensor_sequence = electrical_sequence;
          }
        }
        precise_encoder_last_cycles = now_cycles;
        speed_observer_phase_raw =
            (uint16_t)lroundf(precise_encoder_phase_raw) & 0x0FFFU;
        speed_observer_erpm = (int32_t)(
            (float)control_speed_actual_mdps * 0.001F *
            CONTROL_MOTOR_POLE_PAIRS / 6.0F);

        const int32_t actual_abs_mdps = control_speed_actual_mdps < 0 ?
            -control_speed_actual_mdps : control_speed_actual_mdps;
        if (!precise_encoder_requested &&
            actual_abs_mdps >= motor_control_config.
                encoder_precise_phase_enter_mdps &&
            abs(speed_observer_encoder_error_raw) <= (int)motor_control_config.
                encoder_precise_max_error_raw) {
          precise_encoder_requested = true;
        } else if (precise_encoder_requested &&
                   actual_abs_mdps <= motor_control_config.
                       encoder_precise_phase_exit_mdps) {
          precise_encoder_requested = false;
        }
        const float blend_step = motor_control_config.
            encoder_precise_phase_blend_per_second * elapsed_seconds;
        if (precise_encoder_requested) {
          precise_encoder_blend += blend_step;
          if (precise_encoder_blend > 1.0F) {
            precise_encoder_blend = 1.0F;
          }
        } else {
          precise_encoder_blend -= blend_step;
          if (precise_encoder_blend < 0.0F) {
            precise_encoder_blend = 0.0F;
          }
        }
        speed_observer_using_encoder = precise_encoder_blend < 1.0F;
        int32_t precise_delta = (int32_t)speed_observer_phase_raw -
            (int32_t)commanded_electrical_raw;
        if (precise_delta > 2047) { precise_delta -= 4096; }
        else if (precise_delta < -2048) { precise_delta += 4096; }
        const uint16_t blended_electrical_raw = (uint16_t)(
            (int32_t)commanded_electrical_raw +
            (int32_t)lroundf((float)precise_delta *
                             precise_encoder_blend)) & 0x0FFFU;
        const foc_pwm_output_t direct_pwm = foc_oriented_voltage_to_pwm(
            blended_electrical_raw, control_speed_voltage_q_counts,
            (uint16_t)TIM1->ARR);
        set_compare_values(direct_pwm.a, direct_pwm.b, direct_pwm.c);
      }
    }
    /* Direct-Vq control only receives a new AS5600 sample at 500 Hz and does
     * not consume d/q current feedback. Updating it at 1 kHz avoids spending
     * the entire 20 kHz ADC interrupt on unused Clarke/Park diagnostics. */
    const uint8_t transform_divider_limit =
        ((((test_kind == TEST_KIND_CURRENT_FOC) ||
           (test_kind == TEST_KIND_ENCODER_ALIGNMENT) ||
           (test_kind == TEST_KIND_RESISTANCE)) &&
          current_foc_active) ||
         current_foc_required ||
         (test_kind == TEST_KIND_FLUX) ||
         (test_kind == TEST_KIND_INDUCTANCE)) ?
        1U : CURRENT_TRANSFORM_DIAGNOSTIC_DIVIDER;
    if ((((test_kind == TEST_KIND_ENCODER_VOLTAGE) &&
          (test_rotation_started_at != 0U)) ||
         (((test_kind == TEST_KIND_CURRENT_FOC) ||
           (test_kind == TEST_KIND_ENCODER_ALIGNMENT) ||
           (test_kind == TEST_KIND_RESISTANCE)) &&
          current_foc_active) ||
         current_foc_required ||
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

      if ((test_kind == TEST_KIND_INDUCTANCE) &&
          (inductance_phase == INDUCTANCE_PHASE_PULSING)) {
        const uint32_t duty = inductance_pulse_duty_counts;
        if (inductance_pulse_state == 0U) {
          TIM1->CCR4 = duty - VESC_INDUCTANCE_SAMPLE_OFFSET_COUNTS;
          set_compare_values(0U, 0U, 0U);
        } else if (inductance_pulse_state == 1U) {
          inductance_pulse_baseline_counts[1] = centered_samples[1];
        } else if (inductance_pulse_state == 2U) {
          set_compare_values((uint16_t)duty, 0U, (uint16_t)duty);
        } else if (inductance_pulse_state == 3U) {
          inductance_pulse_current_ma_sum += (int64_t)(-
              ((float)(centered_samples[1] -
                       inductance_pulse_baseline_counts[1]) *
               CURRENT_ADC_TO_AMPS * 1000.0F));
          inductance_pulse_bus_mv_sum += bus_voltage_mv;
          ++inductance_pulse_current_samples;
          set_compare_values(0U, 0U, 0U);
        } else if (inductance_pulse_state == 4U) {
          inductance_pulse_baseline_counts[0] = centered_samples[0];
        } else if (inductance_pulse_state == 5U) {
          set_compare_values(0U, (uint16_t)duty, (uint16_t)duty);
        } else if (inductance_pulse_state == 6U) {
          inductance_pulse_current_ma_sum += (int64_t)(-
              ((float)(centered_samples[0] -
                       inductance_pulse_baseline_counts[0]) *
               CURRENT_ADC_TO_AMPS * 1000.0F));
          inductance_pulse_bus_mv_sum += bus_voltage_mv;
          ++inductance_pulse_current_samples;
          set_compare_values(0U, 0U, 0U);
        } else if (inductance_pulse_state == 7U) {
          inductance_pulse_baseline_counts[2] = centered_samples[2];
        } else if (inductance_pulse_state == 8U) {
          set_compare_values((uint16_t)duty, (uint16_t)duty, 0U);
        } else if (inductance_pulse_state == 9U) {
          inductance_pulse_current_ma_sum += (int64_t)(-
              ((float)(centered_samples[2] -
                       inductance_pulse_baseline_counts[2]) *
               CURRENT_ADC_TO_AMPS * 1000.0F));
          inductance_pulse_bus_mv_sum += bus_voltage_mv;
          ++inductance_pulse_current_samples;
          set_compare_values(0U, 0U, 0U);
        } else if (inductance_pulse_state == 10U) {
          ++inductance_pulse_sequences;
          const uint32_t sequence_goal = inductance_pulse_searching ?
              VESC_INDUCTANCE_SEARCH_SEQUENCES :
              VESC_INDUCTANCE_MEASURE_SEQUENCES;
          if (inductance_pulse_sequences >= sequence_goal) {
            const uint32_t average_current_ma =
                inductance_pulse_current_samples == 0U ? 0U :
                (uint32_t)(llabs(inductance_pulse_current_ma_sum) /
                           inductance_pulse_current_samples);
            if (inductance_pulse_searching &&
                average_current_ma < 500U &&
                duty < (VESC_INDUCTANCE_TIMER_ARR + 1U) / 2U) {
              uint32_t next_duty = duty * 3U / 2U;
              const uint32_t maximum_duty =
                  (VESC_INDUCTANCE_TIMER_ARR + 1U) / 2U;
              inductance_pulse_duty_counts =
                  next_duty > maximum_duty ? maximum_duty : next_duty;
              TIM1->CCR4 = inductance_pulse_duty_counts -
                  VESC_INDUCTANCE_SAMPLE_OFFSET_COUNTS;
            } else if (inductance_pulse_searching) {
              inductance_pulse_searching = false;
            } else {
              inductance_delta_current_ma = (int32_t)average_current_ma;
              inductance_voltage_mv = (int32_t)(
                  inductance_pulse_current_samples == 0U ? 0U :
                  inductance_pulse_bus_mv_sum /
                  inductance_pulse_current_samples);
              finish_vesc_inductance_measurement();
              inductance_phase = INDUCTANCE_PHASE_COMPLETE;
            }
            if (inductance_phase != INDUCTANCE_PHASE_COMPLETE) {
              inductance_pulse_sequences = 0U;
              inductance_pulse_current_samples = 0U;
              inductance_pulse_current_ma_sum = 0;
              inductance_pulse_bus_mv_sum = 0U;
            }
          }
          inductance_pulse_state = 0U;
          return;
        }
        ++inductance_pulse_state;
        return;
      }

      uint16_t electrical_raw;
      uint32_t electrical_timestamp_ms = 0U;
      uint32_t electrical_sequence = 0U;
      const bool fixed_identification_angle =
          (test_kind == TEST_KIND_ENCODER_ALIGNMENT) ||
          (test_kind == TEST_KIND_RESISTANCE) ||
          (test_kind == TEST_KIND_INDUCTANCE);
      const bool angle_valid = fixed_identification_angle ?
          ((electrical_raw = test_kind == TEST_KIND_ENCODER_ALIGNMENT ?
               test_command_electrical_raw : 0U), true) :
          angle_sensor_get_electrical_sample_fast(&electrical_raw,
                                                   &electrical_timestamp_ms,
                                                   &electrical_sequence);
      if (angle_valid) {
        uint16_t control_electrical_raw = electrical_raw;
        if (((control_mode == POWER_STAGE_CONTROL_SPEED) ||
             (control_mode == POWER_STAGE_CONTROL_POSITION_PROFILE)) &&
            !control_speed_current_foc &&
            motor_control_config.high_speed_pll_prediction_enabled) {
          uint32_t sample_age_ms =
              (uint32_t)(HAL_GetTick() - electrical_timestamp_ms);
          if (sample_age_ms > 10U) { sample_age_ms = 10U; }
          const float prediction_seconds =
              (float)sample_age_ms * 0.001F +
              motor_control_config.encoder_phase_advance_seconds;
          const float mechanical_speed_dps =
              (float)control_speed_actual_mdps * 0.001F;
          const int32_t advance_raw = (int32_t)lroundf(
              mechanical_speed_dps * CONTROL_MOTOR_POLE_PAIRS *
              prediction_seconds * (4096.0F / 360.0F));
          control_pll_predicted_electrical_raw =
              (uint16_t)((int32_t)electrical_raw + advance_raw) & 0x0FFFU;
          control_electrical_raw = control_pll_predicted_electrical_raw;
        }
        if (((control_mode == POWER_STAGE_CONTROL_SPEED) ||
             (control_mode == POWER_STAGE_CONTROL_POSITION_PROFILE)) &&
            control_speed_current_foc &&
            motor_control_config.high_speed_pll_prediction_enabled) {
          const float measured_phase_rad = (float)electrical_raw *
              (6.28318530718F / 4096.0F);
          if (!control_pll_initialized) {
            control_pll_phase_rad = measured_phase_rad;
            control_pll_speed_electrical_rad_per_second = 0.0F;
            control_pll_sensor_sequence = electrical_sequence;
            control_pll_sensor_timestamp_ms = electrical_timestamp_ms;
            control_pll_initialized = true;
          } else {
            control_pll_phase_rad +=
                control_pll_speed_electrical_rad_per_second *
                CURRENT_FOC_DT_SECONDS;
            if (control_pll_phase_rad >= 6.28318530718F) {
              control_pll_phase_rad -= 6.28318530718F;
            } else if (control_pll_phase_rad < 0.0F) {
              control_pll_phase_rad += 6.28318530718F;
            }
            if (electrical_sequence != control_pll_sensor_sequence) {
              float delta_theta = measured_phase_rad - control_pll_phase_rad;
              if (delta_theta > 3.14159265359F) {
                delta_theta -= 6.28318530718F;
              } else if (delta_theta < -3.14159265359F) {
                delta_theta += 6.28318530718F;
              }
              uint32_t sample_elapsed_ms = electrical_timestamp_ms -
                  control_pll_sensor_timestamp_ms;
              if (sample_elapsed_ms < 1U) { sample_elapsed_ms = 1U; }
              if (sample_elapsed_ms > 10U) { sample_elapsed_ms = 10U; }
              const float sample_dt = (float)sample_elapsed_ms * 0.001F;
              control_pll_phase_rad +=
                  CONTROL_FOC_PLL_KP * delta_theta * sample_dt;
              if (control_pll_phase_rad >= 6.28318530718F) {
                control_pll_phase_rad -= 6.28318530718F;
              } else if (control_pll_phase_rad < 0.0F) {
                control_pll_phase_rad += 6.28318530718F;
              }
              control_pll_speed_electrical_rad_per_second +=
                  CONTROL_FOC_PLL_KI * delta_theta * sample_dt;
              control_pll_sensor_sequence = electrical_sequence;
              control_pll_sensor_timestamp_ms = electrical_timestamp_ms;
            }
          }
          float predicted_phase_rad = control_pll_phase_rad +
              control_pll_speed_electrical_rad_per_second *
              motor_control_config.encoder_phase_advance_seconds;
          while (predicted_phase_rad >= 6.28318530718F) {
            predicted_phase_rad -= 6.28318530718F;
          }
          while (predicted_phase_rad < 0.0F) {
            predicted_phase_rad += 6.28318530718F;
          }
          control_pll_predicted_electrical_raw = (uint16_t)(
              predicted_phase_rad * (4096.0F / 6.28318530718F)) & 0x0FFFU;
          if (((control_mode == POWER_STAGE_CONTROL_SPEED) ||
               (control_mode == POWER_STAGE_CONTROL_POSITION_PROFILE)) &&
              control_speed_current_foc &&
              motor_control_config.high_speed_pll_prediction_enabled) {
            control_electrical_raw = control_pll_predicted_electrical_raw;
          }
          /* Keep the slow mechanical position PLL as the sole speed source
           * unless high-speed electrical prediction is explicitly enabled.
           * Otherwise this ISR-side estimate overwrites the filtered value
           * consumed by the speed PI, even while prediction is disabled. */
          if (control_speed_current_foc &&
              motor_control_config.high_speed_pll_prediction_enabled) {
            control_speed_actual_mdps = (int32_t)(
                control_pll_speed_electrical_rad_per_second *
                (180000.0F / 3.14159265359F) /
                CONTROL_MOTOR_POLE_PAIRS);
          }
        }
        const bool stationary_identification =
            (test_kind == TEST_KIND_RESISTANCE) ||
            (test_kind == TEST_KIND_INDUCTANCE);
        const bool inline_identification = stationary_identification ||
            (test_kind == TEST_KIND_FLUX);
        const bool full_clarke_currents = inline_identification ||
            (test_kind == TEST_KIND_ENCODER_ALIGNMENT) ||
            (control_mode != POWER_STAGE_CONTROL_DISABLED);
        const float current_scale = full_clarke_currents ?
            -CURRENT_ADC_TO_AMPS : CURRENT_ADC_TO_AMPS;
        const int32_t *transform_samples = full_clarke_currents ?
            centered_samples : corrected;
        const float ia = (float)transform_samples[0] * current_scale;
        const float ib = (float)transform_samples[1] * current_scale;
        const float ic = (float)transform_samples[2] * current_scale;
        /* FOC owns Clarke/Park math; PowerStage owns ADC sample selection. */
        const foc_dq_sample_t dq = foc_clarke_park(
            ia, ib, ic, full_clarke_currents, control_electrical_raw);
        const float id = dq.d;
        const float iq = dq.q;
        const int32_t id_ma = (int32_t)(1000.0F * id);
        const int32_t iq_ma = (int32_t)(1000.0F * iq);
        control_id_ma = id_ma;
        control_iq_ma = iq_ma;
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

        if ((((control_mode == POWER_STAGE_CONTROL_SPEED) &&
              !control_speed_current_foc) ||
             (control_mode == POWER_STAGE_CONTROL_POSITION) ||
             (control_mode == POWER_STAGE_CONTROL_POSITION_PROFILE)) &&
            current_foc_active) {
          const int32_t voltage_q_counts = control_speed_voltage_q_counts;
          const foc_pwm_output_t pwm = foc_oriented_voltage_to_pwm(
              control_electrical_raw, voltage_q_counts, (uint16_t)TIM1->ARR);
          set_compare_values(pwm.a, pwm.b, pwm.c);
        } else if ((((test_kind == TEST_KIND_CURRENT_FOC) ||
              (test_kind == TEST_KIND_ENCODER_ALIGNMENT) ||
              (test_kind == TEST_KIND_RESISTANCE)) ||
             (control_mode != POWER_STAGE_CONTROL_DISABLED)) &&
            current_foc_active) {
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
          const float voltage_limit_counts =
              (float)(TIM1->ARR /
                  (test_kind == TEST_KIND_RESISTANCE ?
                       RESISTANCE_MODULATION_DIVISOR :
                       (control_speed_current_foc ?
                            HIGH_SPEED_MODULATION_DIVISOR :
                            CURRENT_FOC_MODULATION_DIVISOR)));

          const float current_ki = test_kind == TEST_KIND_RESISTANCE ?
              RESISTANCE_FOC_KI_COUNTS_PER_AMP_SECOND :
              motor_control_config.current_ki_pwm_counts_per_amp_second;
          foc_current_controller_t controller = {
            .integral_d = current_foc_integral_d,
            .integral_q = current_foc_integral_q,
          };
          const foc_voltage_output_t voltage = foc_current_control(
              &controller, id, iq, id_target, iq_target,
              motor_control_config.current_kp_pwm_counts_per_amp,
              current_ki,
              motor_control_config.current_loop_period_seconds,
              voltage_limit_counts, control_electrical_raw);
          current_foc_integral_d = controller.integral_d;
          current_foc_integral_q = controller.integral_q;
          if (voltage.integral_d_saturated) {
            ++test_integral_d_saturated_samples;
          }
          if (voltage.integral_q_saturated) {
            ++test_integral_q_saturated_samples;
          }
          if (voltage.voltage_saturated) {
            ++test_voltage_saturated_samples;
          }
          test_voltage_request_sum_counts += voltage.request_counts;
          if (voltage.request_counts > test_voltage_request_max_counts) {
            test_voltage_request_max_counts = voltage.request_counts;
          }
          const float voltage_d = voltage.d;
          const float voltage_q = voltage.q;

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
                    (applied_vd_mv * 2000) / (id_ma * 3) : 0;
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

          const foc_pwm_output_t pwm =
              control_speed_current_foc &&
              motor_control_config.high_speed_svm_enabled ?
              foc_svm_voltage_to_pwm(voltage.alpha, voltage.beta,
                                     (uint16_t)TIM1->ARR, NULL) :
              foc_voltage_to_pwm(voltage.alpha, voltage.beta,
                                 (uint16_t)TIM1->ARR);
          set_compare_values(pwm.a, pwm.b, pwm.c);
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

  /*
   * DRV8301 keeps nFAULT low briefly while EN_GATE wakes its internal
   * supplies and charge pump.  power_stage_enable() deliberately waits for
   * that interval and checks the pin before entering RUNNING.  Ignore the
   * corresponding falling edge here; once running, nFAULT is an immediate
   * shutdown request.
   */
  if (stage_state != POWER_STAGE_RUNNING) {
    return;
  }

  power_stage_disable();
  stage_state = POWER_STAGE_FAULT;
  test_state = POWER_STAGE_TEST_ABORTED;
  /* SPI is intentionally not used in the ISR. Status is read later. */
  latched_faults |= DRV8301_FAULT_FAULT;
  fault_status_pending = true;
}
