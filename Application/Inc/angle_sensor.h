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
  uint32_t timestamp_ms;
  angle_sensor_status_t status;
} angle_sensor_sample_t;

/** Initialize the configured angle sensor backend. */
bool angle_sensor_init(void);

/** Perform scheduled, non-blocking-at-application-level sensor servicing. */
void angle_sensor_process(void);

/** Copy the latest cached sample. Returns false until the first valid sample. */
bool angle_sensor_get_sample(angle_sensor_sample_t *sample);

/** Convenience accessors for control code that only needs one representation. */
bool angle_sensor_read_degrees(float *degrees);
bool angle_sensor_read_raw(uint16_t *raw);

angle_sensor_status_t angle_sensor_get_status(void);

#endif /* ANGLE_SENSOR_H */
