#ifndef AS5600_H
#define AS5600_H

#include <stdbool.h>
#include <stdint.h>

enum {
  AS5600_I2C_ADDRESS = 0x36U,
  AS5600_RAW_MAX = 4095U
};

typedef bool (*as5600_bus_ready_fn)(void *context, uint8_t address);
typedef bool (*as5600_bus_read_fn)(void *context, uint8_t address,
                                  uint8_t reg, uint8_t *data,
                                  uint16_t length);

typedef struct {
  void *context;
  as5600_bus_ready_fn ready;
  as5600_bus_read_fn read;
} as5600_bus_t;

typedef enum {
  AS5600_MAGNET_OK = 0,
  AS5600_MAGNET_NOT_DETECTED,
  AS5600_MAGNET_TOO_WEAK,
  AS5600_MAGNET_TOO_STRONG,
  AS5600_MAGNET_IO_ERROR
} as5600_magnet_status_t;

typedef struct {
  as5600_bus_t bus;
} as5600_t;

bool as5600_init(as5600_t *device, const as5600_bus_t *bus);
bool as5600_read_raw(as5600_t *device, uint16_t *raw);
as5600_magnet_status_t as5600_read_magnet_status(as5600_t *device);

#endif /* AS5600_H */
