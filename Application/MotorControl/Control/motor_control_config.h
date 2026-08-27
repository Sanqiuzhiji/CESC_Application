#ifndef MOTOR_CONTROL_CONFIG_H
#define MOTOR_CONTROL_CONFIG_H

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
  int32_t maximum_position_mdeg;
  int32_t default_speed_acceleration_mdps2;
  int32_t default_profile_acceleration_mdps2;
  int32_t minimum_profile_acceleration_mdps2;
  int32_t maximum_profile_acceleration_mdps2;

  uint32_t command_timeout_ms;
  uint32_t sensor_startup_grace_ms;
  uint32_t encoder_max_sample_age_ms;
  uint8_t encoder_invalid_sample_limit;
  uint32_t minimum_bus_voltage_mv;
  uint32_t maximum_bus_voltage_mv;

  float encoder_pll_kp;
  float encoder_pll_ki;
  float current_kp_pwm_counts_per_amp;
  float current_ki_pwm_counts_per_amp_second;
  float current_loop_period_seconds;

  uint16_t current_modulation_divisor;
  uint16_t direct_voltage_modulation_divisor;
  float speed_full_output_error_counts;
  float position_full_output_error_counts;
} motor_control_config_t;

extern const motor_control_config_t motor_control_config;

#endif
