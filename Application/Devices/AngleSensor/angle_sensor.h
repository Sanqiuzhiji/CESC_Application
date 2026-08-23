#ifndef ANGLE_SENSOR_H
#define ANGLE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  ANGLE_SENSOR_STATUS_UNINITIALIZED = 0,
  ANGLE_SENSOR_STATUS_OK,
  ANGLE_SENSOR_STATUS_NOT_FOUND,
  ANGLE_SENSOR_STATUS_NO_MAGNET,
  ANGLE_SENSOR_STATUS_MAGNET_WEAK,
  ANGLE_SENSOR_STATUS_MAGNET_STRONG,
  ANGLE_SENSOR_STATUS_IO_ERROR
} angle_sensor_status_t;

typedef struct {
  uint16_t raw;
  float degrees;
  int32_t position_counts;
  float position_degrees;
  uint16_t electrical_raw_unaligned;
  float electrical_degrees_unaligned;
  uint16_t electrical_zero_raw;
  uint16_t electrical_raw;
  float electrical_degrees;
  bool electrical_zero_calibrated;
  uint32_t timestamp_ms;
  angle_sensor_status_t status;
} angle_sensor_sample_t;

enum { ANGLE_SENSOR_MOTOR_POLE_PAIRS = 11U };

/** Initialize the configured angle sensor backend. */
bool angle_sensor_init(void);

/** Perform scheduled, non-blocking-at-application-level sensor servicing. */
void angle_sensor_process(void);

/** Copy the latest cached sample. Returns false until the first valid sample. */
bool angle_sensor_get_sample(angle_sensor_sample_t *sample);

/** Convenience accessors for control code that only needs one representation. */
bool angle_sensor_read_degrees(float *degrees);
bool angle_sensor_read_raw(uint16_t *raw);

/** Calibrate electrical zero using the latest shaft sample at a known target. */
bool angle_sensor_calibrate_electrical_zero(uint16_t target_electrical_raw);

/** Lock-free cached electrical angle read suitable for the ADC ISR. */
bool angle_sensor_get_electrical_raw_fast(uint16_t *electrical_raw);

angle_sensor_status_t angle_sensor_get_status(void);

#endif /* ANGLE_SENSOR_H */
