#include "drv8301.h"

#include "main.h"
#include "spi.h"

enum {
  DRV8301_REGISTER_STATUS_1 = 0U,
  DRV8301_REGISTER_STATUS_2 = 1U,
  DRV8301_REGISTER_CONTROL_1 = 2U,
  DRV8301_REGISTER_CONTROL_2 = 3U,
  DRV8301_REGISTER_DATA_MASK = 0x07FFU,
  DRV8301_COMMAND_READ = 1U << 15,
  DRV8301_COMMAND_ADDRESS_SHIFT = 11U,
  DRV8301_CONTROL_1_GATE_RESET = 1U << 2,
  DRV8301_SPI_TIMEOUT_MS = 10U
};

static drv8301_result_t transfer_frame(uint16_t transmit, uint16_t *receive)
{
  uint16_t received = 0U;

  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
  const HAL_StatusTypeDef status =
      HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&transmit,
                              (uint8_t *)&received, 1U,
                              DRV8301_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

  if (status != HAL_OK) {
    return DRV8301_RESULT_IO_ERROR;
  }
  if (receive != NULL) {
    *receive = received;
  }
  return DRV8301_RESULT_OK;
}

drv8301_result_t drv8301_init(void)
{
  HAL_GPIO_WritePin(DRV_EN_GATE_GPIO_Port, DRV_EN_GATE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
  return DRV8301_RESULT_OK;
}

drv8301_result_t drv8301_read_register(uint8_t address, uint16_t *value)
{
  uint16_t ignored;
  uint16_t response;

  if ((value == NULL) || (address > DRV8301_REGISTER_CONTROL_2)) {
    return DRV8301_RESULT_INVALID_ARGUMENT;
  }

  const uint16_t command =
      DRV8301_COMMAND_READ |
      ((uint16_t)address << DRV8301_COMMAND_ADDRESS_SHIFT);

  /* DRV8301 returns the requested register during the following SPI frame. */
  if (transfer_frame(command, &ignored) != DRV8301_RESULT_OK ||
      transfer_frame(DRV8301_COMMAND_READ, &response) != DRV8301_RESULT_OK) {
    return DRV8301_RESULT_IO_ERROR;
  }

  *value = response & DRV8301_REGISTER_DATA_MASK;
  return DRV8301_RESULT_OK;
}

drv8301_result_t drv8301_write_register(uint8_t address, uint16_t value)
{
  uint16_t ignored;

  if ((address < DRV8301_REGISTER_CONTROL_1) ||
      (address > DRV8301_REGISTER_CONTROL_2) ||
      ((value & ~DRV8301_REGISTER_DATA_MASK) != 0U)) {
    return DRV8301_RESULT_INVALID_ARGUMENT;
  }

  const uint16_t command =
      ((uint16_t)address << DRV8301_COMMAND_ADDRESS_SHIFT) | value;
  return transfer_frame(command, &ignored);
}

drv8301_result_t drv8301_read_faults(uint16_t *faults)
{
  uint16_t status_1;
  uint16_t status_2;

  if (faults == NULL) {
    return DRV8301_RESULT_INVALID_ARGUMENT;
  }
  if (drv8301_read_register(DRV8301_REGISTER_STATUS_1, &status_1) !=
          DRV8301_RESULT_OK ||
      drv8301_read_register(DRV8301_REGISTER_STATUS_2, &status_2) !=
          DRV8301_RESULT_OK) {
    return DRV8301_RESULT_IO_ERROR;
  }

  *faults = (status_1 & 0x03FFU) | ((status_2 & 0x0080U) << 4U);
  return DRV8301_RESULT_OK;
}

drv8301_result_t drv8301_clear_faults(void)
{
  uint16_t control_1;

  if (drv8301_read_register(DRV8301_REGISTER_CONTROL_1, &control_1) !=
      DRV8301_RESULT_OK) {
    return DRV8301_RESULT_IO_ERROR;
  }
  return drv8301_write_register(
      DRV8301_REGISTER_CONTROL_1,
      (control_1 | DRV8301_CONTROL_1_GATE_RESET) &
          DRV8301_REGISTER_DATA_MASK);
}
