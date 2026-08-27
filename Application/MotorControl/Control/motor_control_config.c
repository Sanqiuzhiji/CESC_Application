#include "motor_control_config.h"

/*
 * Default configuration for the present CESC board and motor. This is the
 * single review point for parameters that an application may later load from
 * non-volatile storage or expose through the protocol.
 */
const motor_control_config_t motor_control_config = {
  .pole_pairs = 11U,
  .torque_constant_nm_per_amp = 0.23F,
  .current_adc_amps_per_count = -0.08058608F,

  .maximum_iq_ma = 300,
  .minimum_active_iq_ma = 10,
  .minimum_speed_mdps = 50,
  .maximum_speed_mdps = 45000,
  .maximum_position_mdeg = 3600000,
  .default_speed_acceleration_mdps2 = 45000,
  .default_profile_acceleration_mdps2 = 10000,
  .minimum_profile_acceleration_mdps2 = 100,
  .maximum_profile_acceleration_mdps2 = 90000,

  .command_timeout_ms = 500U,
  .sensor_startup_grace_ms = 200U,
  .encoder_max_sample_age_ms = 50U,
  .encoder_invalid_sample_limit = 5U,
  .minimum_bus_voltage_mv = 6000U,
  .maximum_bus_voltage_mv = 10000U,

  .encoder_pll_kp = 200.0F,
  .encoder_pll_ki = 3000.0F,
  .current_kp_pwm_counts_per_amp = 150.0F,
  .current_ki_pwm_counts_per_amp_second = 280000.0F,
  .current_loop_period_seconds = 0.00005F,

  .current_modulation_divisor = 10U,
  .direct_voltage_modulation_divisor = 12U,
  .speed_full_output_error_counts = 46.0F,
  .position_full_output_error_counts = 20.0F,
};
