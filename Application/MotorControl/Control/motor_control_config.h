#ifndef MOTOR_CONTROL_CONFIG_H
#define MOTOR_CONTROL_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/*
 * User-tunable motor and motion-control parameters.
 *
 * Keep board wiring, ADC channels and timer details in PowerStage. Values
 * that describe the attached motor or externally visible motion behavior
 * belong here, following VESC's mc_configuration separation.
 */
typedef struct {
  uint8_t pole_pairs;
  float torque_constant_nm_per_amp;
  float current_adc_amps_per_count;

  int32_t maximum_iq_ma;
  int32_t minimum_active_iq_ma;
  int32_t minimum_speed_mdps;
  int32_t maximum_speed_mdps;
  int32_t speed_voltage_full_scale_mdps;
  int32_t maximum_profile_speed_mdps;
  int32_t maximum_position_mdeg;
  int32_t default_speed_acceleration_mdps2;
  int32_t low_speed_acceleration_mdps2;
  int32_t voltage_speed_acceleration_mdps2;
  int32_t default_profile_acceleration_mdps2;
  int32_t minimum_profile_acceleration_mdps2;
  int32_t maximum_profile_acceleration_mdps2;
  float position_profile_following_kp_per_second;
  float position_profile_following_speed_fraction;

  uint32_t command_timeout_ms;
  uint32_t sensor_startup_grace_ms;
  uint32_t encoder_max_sample_age_ms;
  uint8_t encoder_invalid_sample_limit;
  uint32_t minimum_bus_voltage_mv;
  uint32_t maximum_bus_voltage_mv;

  float encoder_pll_kp;
  float encoder_pll_ki;
  float encoder_phase_advance_seconds;
  float encoder_precise_phase_correction_gain;
  uint16_t encoder_precise_max_error_raw;
  float encoder_precise_max_correction_raw;
  int32_t encoder_precise_phase_enter_mdps;
  int32_t encoder_precise_phase_exit_mdps;
  float encoder_precise_phase_blend_per_second;
  int32_t high_speed_enter_mdps;
  int32_t high_speed_exit_mdps;
  float speed_pid_kp;
  float speed_pid_ki;
  float speed_pid_normalization;
  float speed_current_feedforward_amp;
  float speed_iq_ramp_amp_per_second;
  float speed_voltage_kp_counts_per_dps;
  float speed_voltage_ki_counts_per_degree;
  float speed_voltage_feedforward_counts_per_dps;
  float speed_voltage_integral_limit_counts;
  float speed_voltage_slew_counts_per_second;
  int32_t speed_voltage_gain_reduction_start_mdps;
  float speed_voltage_high_speed_gain_scale;
  float speed_voltage_current_limit_decay_per_second;
  int32_t speed_voltage_slow_current_limit_ma;
  uint16_t speed_voltage_phase_current_limit_adc_counts;
  uint8_t speed_voltage_fast_limit_decay_shift;
  uint16_t speed_voltage_fast_limit_hold_ms;
  uint16_t speed_voltage_limit_recovery_counts_per_second;
  float current_kp_pwm_counts_per_amp;
  float current_ki_pwm_counts_per_amp_second;
  float current_loop_period_seconds;

  float motor_resistance_ohm;
  float motor_inductance_h;
  float motor_flux_linkage_wb;
  float observer_gain;
  float observer_gain_slow;
  float observer_pll_kp;
  float observer_pll_ki;
  float observer_transition_erpm;
  uint8_t observer_diagnostic_enabled;

  uint16_t current_modulation_divisor;
  uint16_t high_speed_modulation_divisor;
  uint16_t direct_voltage_modulation_divisor;
  uint16_t speed_voltage_max_modulation_divisor;
  uint16_t speed_voltage_safe_sample_max_counts;
  uint8_t speed_current_foc_enabled;
  uint8_t high_speed_pll_prediction_enabled;
  uint8_t high_speed_svm_enabled;
  float speed_full_output_error_counts;
  float position_full_output_error_counts;
} motor_control_config_t;

typedef struct {
  uint8_t pole_pairs;
  float torque_constant_nm_per_amp;
  float motor_resistance_ohm;
  float motor_inductance_h;
  float motor_flux_linkage_wb;
  int32_t maximum_iq_ma;
  int32_t maximum_speed_mdps;
  int32_t maximum_position_mdeg;
  uint32_t minimum_bus_voltage_mv;
  uint32_t maximum_bus_voltage_mv;
  uint32_t command_timeout_ms;
} motor_user_config_t;

extern motor_control_config_t motor_control_config;
extern const motor_control_config_t motor_control_default_config;

void motor_control_config_get_user(motor_user_config_t *config);
void motor_control_config_get_default_user(motor_user_config_t *config);
bool motor_control_config_validate_user(const motor_user_config_t *config);
bool motor_control_config_apply_user(const motor_user_config_t *config);

#endif
