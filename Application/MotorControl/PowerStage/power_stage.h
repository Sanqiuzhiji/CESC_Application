#ifndef POWER_STAGE_H
#define POWER_STAGE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  POWER_STAGE_UNINITIALIZED = 0,
  POWER_STAGE_CALIBRATING,
  POWER_STAGE_READY,
  POWER_STAGE_RUNNING,
  POWER_STAGE_FAULT
} power_stage_state_t;

typedef enum {
  POWER_STAGE_CONTROL_DISABLED = 0,
  POWER_STAGE_CONTROL_IQ_CURRENT,
  POWER_STAGE_CONTROL_SPEED,
  POWER_STAGE_CONTROL_POSITION,
  POWER_STAGE_CONTROL_POSITION_PROFILE,
  POWER_STAGE_CONTROL_HAPTIC
} power_stage_control_mode_t;

typedef struct {
  uint16_t raw[3];
  int32_t centered[3];
  uint16_t offset[3];
  uint32_t sequence;
} power_stage_current_sample_t;

typedef struct {
  power_stage_state_t state;
  power_stage_current_sample_t current;
  uint16_t drv_faults;
  uint16_t bus_voltage_raw;
  uint32_t bus_voltage_mv;
  bool gate_enabled;
  bool pwm_outputs_enabled;
  bool fault_pin_active;
  bool bus_voltage_valid;
  uint32_t test_current_samples;
  int64_t test_current_sum[3];
  int16_t test_current_min[3];
  int16_t test_current_max[3];
  uint64_t test_current_balance_abs_sum;
  uint16_t test_current_balance_abs_max;
  uint32_t test_v0_samples;
  uint32_t test_v7_samples;
  uint32_t test_reconstructed_samples[3];
  uint32_t test_transform_samples;
  int64_t test_id_sum_ma;
  int64_t test_iq_sum_ma;
  int32_t test_id_min_ma;
  int32_t test_id_max_ma;
  int32_t test_iq_min_ma;
  int32_t test_iq_max_ma;
  int64_t test_iq_target_sum_ma;
  int32_t test_iq_target_min_ma;
  int32_t test_iq_target_max_ma;
  uint32_t test_voltage_saturated_samples;
  uint32_t test_integral_d_saturated_samples;
  uint32_t test_integral_q_saturated_samples;
  uint64_t test_voltage_request_sum_counts;
  uint16_t test_voltage_request_max_counts;
  uint32_t resistance_measurement_samples;
  int64_t resistance_iq_sum_ma;
  int64_t resistance_vq_sum_mv;
  uint8_t resistance_phase;
  uint8_t resistance_valid;
  uint32_t resistance_forward_samples;
  uint32_t resistance_reverse_samples;
  int32_t resistance_id_ma;
  int32_t resistance_iq_ma;
  int32_t resistance_vd_mv;
  int32_t resistance_vq_mv;
  int32_t resistance_live_milliohms;
  int32_t resistance_forward_milliohms;
  int32_t resistance_reverse_milliohms;
  int32_t resistance_average_milliohms;
  uint8_t inductance_phase;
  uint8_t inductance_valid;
  uint32_t inductance_forward_samples;
  uint32_t inductance_reverse_samples;
  int32_t inductance_delta_current_ma;
  int32_t inductance_voltage_mv;
  uint32_t inductance_forward_uh;
  uint32_t inductance_reverse_uh;
  uint32_t inductance_average_uh;
  uint8_t flux_valid;
  uint32_t flux_samples;
  int32_t flux_speed_millidegrees_per_second;
  int32_t flux_iq_ma;
  int32_t flux_vq_mv;
  uint32_t flux_linkage_uwb;
  uint32_t back_emf_constant_uv_per_rad_s;
  uint32_t kv_millirpm_per_volt;
  power_stage_control_mode_t control_mode;
  int32_t control_id_ma;
  int32_t control_iq_ma;
  int32_t control_iq_target_ma;
  uint32_t control_timeout_remaining_ms;
  int32_t control_speed_target_millidegrees_per_second;
  int32_t control_speed_millidegrees_per_second;
  int32_t control_position_target_millidegrees;
  int32_t control_position_millidegrees;
} power_stage_diagnostics_t;

typedef enum {
  POWER_STAGE_TEST_IDLE = 0,
  POWER_STAGE_TEST_RUNNING,
  POWER_STAGE_TEST_COMPLETED,
  POWER_STAGE_TEST_ABORTED
} power_stage_test_state_t;

/**
 * Start synchronized current sampling with all gate outputs disabled.
 * This function never enables the MOSFET bridge.
 */
bool power_stage_init(void);

/** Perform deferred diagnostics such as reading DRV8301 fault registers. */
void power_stage_process(void);

/** Disable PWM outputs and EN_GATE immediately. Safe from interrupt context. */
void power_stage_disable(void);

/**
 * Enable the bridge at the supplied raw timer duties. During commissioning,
 * compares are normally clamped to +/-10% around the 50% neutral point.
 * Resistance identification applies its own bounded voltage-vector limit.
 * Not used automatically; callers must first verify READY state and hardware.
 */
bool power_stage_enable(uint16_t duty_a, uint16_t duty_b, uint16_t duty_c);

/** Atomically update three PWM compare values while the bridge is running. */
bool power_stage_set_duty(uint16_t duty_a, uint16_t duty_b, uint16_t duty_c);

/** Start the fixed, low-energy commissioning rotation. */
bool power_stage_start_commissioning_test(int8_t direction);
bool power_stage_start_encoder_alignment(void);
bool power_stage_start_encoder_voltage_test(int8_t direction);
bool power_stage_start_current_foc_test(int8_t direction);
bool power_stage_start_resistance_measurement(void);
bool power_stage_start_inductance_measurement(void);
bool power_stage_start_flux_measurement(int8_t direction);
/**
 * Enter or refresh sensored FOC current control. The command must be refreshed
 * before the internal command watchdog expires. A zero target releases the
 * bridge, matching VESC current-mode semantics.
 */
bool power_stage_set_iq_current_ma(int32_t iq_target_ma);
/** Set estimated shaft torque in mN*m using motor_control_config.Kt. */
bool power_stage_set_torque_millinewton_metres(int32_t torque_target_mnm);
bool power_stage_set_speed_millidegrees_per_second(int32_t speed_target);
bool power_stage_set_position_millidegrees(int32_t position_target);
bool power_stage_set_position_profile(int32_t position_target,
                                      int32_t maximum_speed_mdps,
                                      int32_t acceleration_mdps2,
                                      int32_t deceleration_mdps2);
bool power_stage_set_haptic(int32_t detent_spacing_mdeg,
                            int32_t detent_strength_ma,
                            int32_t damping_ma_per_dps,
                            int32_t minimum_position_mdeg,
                            int32_t maximum_position_mdeg);
void power_stage_stop_commissioning_test(void);
power_stage_test_state_t power_stage_get_test_state(void);
uint8_t power_stage_get_test_steps_completed(void);

power_stage_state_t power_stage_get_state(void);
bool power_stage_get_current_sample(power_stage_current_sample_t *sample);
uint16_t power_stage_get_latched_faults(void);
bool power_stage_get_diagnostics(power_stage_diagnostics_t *diagnostics);

#endif /* POWER_STAGE_H */
