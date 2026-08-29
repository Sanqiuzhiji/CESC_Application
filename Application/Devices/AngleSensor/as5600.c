#include "as5600.h"

#include <stddef.h>

enum {
  AS5600_REG_CONF_HIGH = 0x07U,
  AS5600_REG_STATUS = 0x0BU,
  AS5600_REG_RAW_ANGLE_HIGH = 0x0CU,
  AS5600_STATUS_MAGNET_TOO_STRONG = 1U << 3,
  AS5600_STATUS_MAGNET_TOO_WEAK = 1U << 4,
  AS5600_STATUS_MAGNET_DETECTED = 1U << 5
};

enum {
  AS5600_CONF_SLOW_FILTER_MASK = 0x03U,
  AS5600_CONF_SLOW_FILTER_FASTEST = 0x03U
};

bool as5600_init(as5600_t *device, const as5600_bus_t *bus)
{
  if ((device == NULL) || (bus == NULL) || (bus->read == NULL)) {
    return false;
  }

  device->bus = *bus;
  return (device->bus.ready == NULL) ||
         device->bus.ready(device->bus.context, AS5600_I2C_ADDRESS);
}

bool as5600_read_raw(as5600_t *device, uint16_t *raw)
{
  uint8_t data[2];

  if ((device == NULL) || (raw == NULL) ||
      !device->bus.read(device->bus.context, AS5600_I2C_ADDRESS,
                        AS5600_REG_RAW_ANGLE_HIGH, data, sizeof(data))) {
    return false;
  }

  *raw = (uint16_t)((((uint16_t)data[0] & 0x0FU) << 8) | data[1]);
  return true;
}

bool as5600_set_fast_filter(as5600_t *device)
{
  uint8_t conf_high;
  if ((device == NULL) || (device->bus.write == NULL) ||
      !device->bus.read(device->bus.context, AS5600_I2C_ADDRESS,
                        AS5600_REG_CONF_HIGH, &conf_high, 1U)) {
    return false;
  }
  conf_high = (uint8_t)((conf_high & ~AS5600_CONF_SLOW_FILTER_MASK) |
                        AS5600_CONF_SLOW_FILTER_FASTEST);
  return device->bus.write(device->bus.context, AS5600_I2C_ADDRESS,
                           AS5600_REG_CONF_HIGH, &conf_high, 1U);
}

as5600_magnet_status_t as5600_read_magnet_status(as5600_t *device)
{
  uint8_t status;

  if ((device == NULL) ||
      !device->bus.read(device->bus.context, AS5600_I2C_ADDRESS,
                        AS5600_REG_STATUS, &status, sizeof(status))) {
    return AS5600_MAGNET_IO_ERROR;
  }
  if ((status & AS5600_STATUS_MAGNET_DETECTED) == 0U) {
    return AS5600_MAGNET_NOT_DETECTED;
  }
  if ((status & AS5600_STATUS_MAGNET_TOO_WEAK) != 0U) {
    return AS5600_MAGNET_TOO_WEAK;
  }
  if ((status & AS5600_STATUS_MAGNET_TOO_STRONG) != 0U) {
    return AS5600_MAGNET_TOO_STRONG;
  }
  return AS5600_MAGNET_OK;
}
