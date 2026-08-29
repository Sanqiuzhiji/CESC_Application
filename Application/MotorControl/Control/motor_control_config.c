#include "motor_control_config.h"

#include <math.h>
#include <string.h>

/*
 * Default configuration for the present CESC board and motor. This is the
 * single review point for parameters that an application may later load from
 * non-volatile storage or expose through the protocol.
 */
const motor_control_config_t motor_control_default_config = {
  .pole_pairs = 11U,
  .torque_constant_nm_per_amp = 0.23F,
  .current_adc_amps_per_count = -0.08058608F,

  .maximum_iq_ma = 300,
  .minimum_active_iq_ma = 10,
  .minimum_speed_mdps = 50,
  .maximum_speed_mdps = 3000000,
  /* Reach the configured voltage ceiling at 300 RPM. Commands above this
   * retain the full available bus voltage; achievable speed then depends on
   * motor Kv, bus voltage and mechanical load. */
  .speed_voltage_full_scale_mdps = 1800000,
  .maximum_profile_speed_mdps = 3000000,
  .maximum_position_mdeg = 3600000,
  .default_speed_acceleration_mdps2 = 600000,
  .low_speed_acceleration_mdps2 = 20000,
  .voltage_speed_acceleration_mdps2 = 120000,
  .default_profile_acceleration_mdps2 = 10000,
  .minimum_profile_acceleration_mdps2 = 100,
  .maximum_profile_acceleration_mdps2 = 600000,
  /* VESC-style cascaded position loop: trajectory position error trims the
   * trajectory velocity before the proven speed controller consumes it. */
  .position_profile_following_kp_per_second = 1.0F,
  .position_profile_following_speed_fraction = 0.25F,

  .command_timeout_ms = 1000U,
  .sensor_startup_grace_ms = 200U,
  .encoder_max_sample_age_ms = 50U,
  .encoder_invalid_sample_limit = 5U,
  .minimum_bus_voltage_mv = 6000U,
  /* Nominal 12 V supply. Keep enough tolerance for adapter regulation and
   * regenerative transients while rejecting an unintended higher-voltage
   * source before enabling the bridge. */
  .maximum_bus_voltage_mv = 15000U,

  .encoder_pll_kp = 200.0F,
  .encoder_pll_ki = 3000.0F,
  .encoder_phase_advance_seconds = 0.0004F,
  .encoder_precise_phase_correction_gain = 0.05F,
  /* An AS5600 mechanical-angle glitch is multiplied by all 11 pole pairs.
   * Follow VESC's PLL principle: reject implausible innovations and only let
   * each accepted sample pull the continuously integrated PWM phase gently. */
  .encoder_precise_max_error_raw = 384U,
  .encoder_precise_max_correction_raw = 8.0F,
  .encoder_precise_phase_enter_mdps = 2400000,
  .encoder_precise_phase_exit_mdps = 2100000,
  .encoder_precise_phase_blend_per_second = 5.0F,
  .high_speed_enter_mdps = 120000,
  .high_speed_exit_mdps = 90000,
  .speed_pid_kp = 0.010F,
  .speed_pid_ki = 0.007F,
  .speed_pid_normalization = 0.05F,
  .speed_current_feedforward_amp = 0.05F,
  .speed_iq_ramp_amp_per_second = 1.0F,
  /* Direct-Vq fallback mirrors VESC's speed PI structure, but its output is
   * voltage counts because this board cannot resolve a fine Iq command. */
  .speed_voltage_kp_counts_per_dps = 0.25F,
  .speed_voltage_ki_counts_per_degree = 2.00F,
  .speed_voltage_feedforward_counts_per_dps = 0.75F,
  .speed_voltage_integral_limit_counts = 600.0F,
  .speed_voltage_slew_counts_per_second = 3000.0F,
  .speed_voltage_gain_reduction_start_mdps = 2400000,
  .speed_voltage_high_speed_gain_scale = 0.25F,
  .speed_voltage_current_limit_decay_per_second = 4.0F,
  /* The shunt chain resolves about 80.6 mA/count. Keep this coarse voltage-
   * mode guard above offset/quantization error; it is separate from the
   * user's 300 mA commanded-torque limit. */
  .speed_voltage_slow_current_limit_ma = 800,
  /* Require repeated phase peaks before reducing voltage. Single-cycle
   * switching transients are expected with the present shunt sampling. */
  .speed_voltage_phase_current_limit_adc_counts = 30U,
  /* A coarse 1/8 voltage cut caused one visible kick near 500 RPM. Keep the
   * fast limiter, but reduce each soft-limit action to 1/64; the independent
   * software hard trip and DRV8301 protection remain unchanged. */
  .speed_voltage_fast_limit_decay_shift = 6U,
  .speed_voltage_fast_limit_hold_ms = 100U,
  .speed_voltage_limit_recovery_counts_per_second = 1000U,
  .current_kp_pwm_counts_per_amp = 150.0F,
  .current_ki_pwm_counts_per_amp_second = 280000.0F,
  .current_loop_period_seconds = 0.00005F,

  /* Bench commissioning values, using the same phase-parameter convention
   * as the VESC observer. Re-run commissioning before final tuning. */
  .motor_resistance_ohm = 2.20F,
  .motor_inductance_h = 0.00116F,
  .motor_flux_linkage_wb = 0.023F,
  .observer_gain = 517000.0F,
  .observer_gain_slow = 0.30F,
  .observer_pll_kp = 2000.0F,
  .observer_pll_ki = 30000.0F,
  /* 2500 eRPM is the VESC default and equals about 227 mechanical RPM for
   * this 11-pole-pair motor. */
  .observer_transition_erpm = 2500.0F,
  /* Keep the observer out of the ADC ISR until its execution time is moved
   * into a measured, bounded scheduling slot. It is diagnostic-only and must
   * not disturb the proven encoder control path. */
  .observer_diagnostic_enabled = 0U,

  .current_modulation_divisor = 10U,
  .high_speed_modulation_divisor = 10U,
  .direct_voltage_modulation_divisor = 12U,
  .speed_voltage_max_modulation_divisor = 2U,
  /* TIM8 now samples both V0 and V7 after their settling interval. Keep a
   * small margin below half-period modulation for bootstrap/dead-time and
   * current-amplifier settling. */
  .speed_voltage_safe_sample_max_counts = 2000U,
  .speed_current_foc_enabled = 0U,
  .high_speed_pll_prediction_enabled = 1U,
  .high_speed_svm_enabled = 0U,
  .speed_full_output_error_counts = 46.0F,
  .position_full_output_error_counts = 20.0F,
};

motor_control_config_t motor_control_config;

static void extract_user(const motor_control_config_t *source,
                         motor_user_config_t *config)
{
  memset(config, 0, sizeof(*config));
  config->pole_pairs = source->pole_pairs;
  config->torque_constant_nm_per_amp = source->torque_constant_nm_per_amp;
  config->motor_resistance_ohm = source->motor_resistance_ohm;
  config->motor_inductance_h = source->motor_inductance_h;
  config->motor_flux_linkage_wb = source->motor_flux_linkage_wb;
  config->maximum_iq_ma = source->maximum_iq_ma;
  config->maximum_speed_mdps = source->maximum_speed_mdps;
  config->maximum_position_mdeg = source->maximum_position_mdeg;
  config->minimum_bus_voltage_mv = source->minimum_bus_voltage_mv;
  config->maximum_bus_voltage_mv = source->maximum_bus_voltage_mv;
  config->command_timeout_ms = source->command_timeout_ms;
}

void motor_control_config_get_user(motor_user_config_t *config)
{
  if (config != NULL) extract_user(&motor_control_config, config);
}

void motor_control_config_get_default_user(motor_user_config_t *config)
{
  if (config != NULL) extract_user(&motor_control_default_config, config);
}

bool motor_control_config_validate_user(const motor_user_config_t *config)
{
  return config != NULL && config->pole_pairs >= 1U &&
      config->pole_pairs <= 64U &&
      isfinite(config->torque_constant_nm_per_amp) &&
      config->torque_constant_nm_per_amp >= 0.001F &&
      config->torque_constant_nm_per_amp <= 10.0F &&
      isfinite(config->motor_resistance_ohm) &&
      config->motor_resistance_ohm >= 0.001F &&
      config->motor_resistance_ohm <= 100.0F &&
      isfinite(config->motor_inductance_h) &&
      config->motor_inductance_h >= 0.000001F &&
      config->motor_inductance_h <= 1.0F &&
      isfinite(config->motor_flux_linkage_wb) &&
      config->motor_flux_linkage_wb >= 0.000001F &&
      config->motor_flux_linkage_wb <= 1.0F &&
      config->maximum_iq_ma >= 10 && config->maximum_iq_ma <= 300 &&
      config->maximum_speed_mdps >= 6000 &&
      config->maximum_speed_mdps <= 3000000 &&
      config->maximum_position_mdeg >= 360000 &&
      config->maximum_position_mdeg <= 36000000 &&
      config->minimum_bus_voltage_mv >= 3000U &&
      config->minimum_bus_voltage_mv <= 12000U &&
      config->maximum_bus_voltage_mv >= 6000U &&
      config->maximum_bus_voltage_mv <= 15000U &&
      config->minimum_bus_voltage_mv < config->maximum_bus_voltage_mv &&
      config->command_timeout_ms >= 100U &&
      config->command_timeout_ms <= 5000U;
}

bool motor_control_config_apply_user(const motor_user_config_t *config)
{
  if (!motor_control_config_validate_user(config)) return false;
  motor_control_config.pole_pairs = config->pole_pairs;
  motor_control_config.torque_constant_nm_per_amp =
      config->torque_constant_nm_per_amp;
  motor_control_config.motor_resistance_ohm = config->motor_resistance_ohm;
  motor_control_config.motor_inductance_h = config->motor_inductance_h;
  motor_control_config.motor_flux_linkage_wb = config->motor_flux_linkage_wb;
  motor_control_config.maximum_iq_ma = config->maximum_iq_ma;
  motor_control_config.maximum_speed_mdps = config->maximum_speed_mdps;
  motor_control_config.maximum_profile_speed_mdps = config->maximum_speed_mdps;
  motor_control_config.maximum_position_mdeg = config->maximum_position_mdeg;
  motor_control_config.minimum_bus_voltage_mv = config->minimum_bus_voltage_mv;
  motor_control_config.maximum_bus_voltage_mv = config->maximum_bus_voltage_mv;
  motor_control_config.command_timeout_ms = config->command_timeout_ms;
  return true;
}
