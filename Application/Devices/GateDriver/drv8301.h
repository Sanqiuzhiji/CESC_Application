#ifndef DRV8301_H
#define DRV8301_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  DRV8301_RESULT_OK = 0,
  DRV8301_RESULT_INVALID_ARGUMENT,
  DRV8301_RESULT_IO_ERROR
} drv8301_result_t;

enum {
  DRV8301_FAULT_FETLC_OC = 1U << 0,
  DRV8301_FAULT_FETHC_OC = 1U << 1,
  DRV8301_FAULT_FETLB_OC = 1U << 2,
  DRV8301_FAULT_FETHB_OC = 1U << 3,
  DRV8301_FAULT_FETLA_OC = 1U << 4,
  DRV8301_FAULT_FETHA_OC = 1U << 5,
  DRV8301_FAULT_OTW = 1U << 6,
  DRV8301_FAULT_OTSD = 1U << 7,
  DRV8301_FAULT_PVDD_UV = 1U << 8,
  DRV8301_FAULT_GVDD_UV = 1U << 9,
  DRV8301_FAULT_FAULT = 1U << 10,
  DRV8301_FAULT_GVDD_OV = 1U << 11
};

/** Prepare the SPI interface and leave the gate driver disabled. */
drv8301_result_t drv8301_init(void);

/** Read one 11-bit DRV8301 register using its pipelined SPI protocol. */
drv8301_result_t drv8301_read_register(uint8_t address, uint16_t *value);

/** Write one 11-bit DRV8301 register. Status registers are rejected. */
drv8301_result_t drv8301_write_register(uint8_t address, uint16_t value);

/** Read and combine status registers 1 and 2 into DRV8301_FAULT_* bits. */
drv8301_result_t drv8301_read_faults(uint16_t *faults);

/** Clear latched faults without changing the other control-register bits. */
drv8301_result_t drv8301_clear_faults(void);

#endif /* DRV8301_H */
