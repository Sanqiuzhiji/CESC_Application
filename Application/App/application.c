#include "application.h"

#include <stdbool.h>
#include "angle_sensor.h"
#include "cesc_protocol.h"
#include "main.h"
#include "cmsis_os2.h"
#include "power_stage.h"
#include "motor_config_store.h"

extern osSemaphoreId_t usbRxSemaphoreHandle;

enum {
  HEARTBEAT_PERIOD_MS = 1200U,
  HEARTBEAT_FIRST_FLASH_END_MS = 200U,
  HEARTBEAT_SECOND_FLASH_START_MS = 400U,
  HEARTBEAT_SECOND_FLASH_END_MS = 800U,
  POWER_READY_PERIOD_MS = 2000U,
  POWER_READY_FLASH_END_MS = 100U,
  POWER_CALIBRATION_TOGGLE_MS = 100U
};

static uint32_t heartbeat_started_at;

static void update_heartbeat(void)
{
  const uint32_t elapsed = HAL_GetTick() - heartbeat_started_at;
  const uint32_t phase = elapsed % HEARTBEAT_PERIOD_MS;
  const bool led_on =
      (phase < HEARTBEAT_FIRST_FLASH_END_MS) ||
      ((phase >= HEARTBEAT_SECOND_FLASH_START_MS) &&
       (phase < HEARTBEAT_SECOND_FLASH_END_MS));

  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin,
                    led_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void update_power_stage_indicator(void)
{
  const uint32_t elapsed = HAL_GetTick() - heartbeat_started_at;
  bool led_on;

  switch (power_stage_get_state()) {
  case POWER_STAGE_CALIBRATING:
    led_on = ((elapsed / POWER_CALIBRATION_TOGGLE_MS) & 1U) == 0U;
    break;
  case POWER_STAGE_READY:
    /* One short red flash every two seconds means ADC calibration passed. */
    led_on = (elapsed % POWER_READY_PERIOD_MS) < POWER_READY_FLASH_END_MS;
    break;
  case POWER_STAGE_RUNNING:
    led_on = true;
    break;
  case POWER_STAGE_UNINITIALIZED:
  case POWER_STAGE_FAULT:
  default:
    led_on = true;
    break;
  }

  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin,
                    led_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void application_init(void)
{
  motor_config_store_init();
  (void)power_stage_init();
  cesc_protocol_init();
  heartbeat_started_at = HAL_GetTick();
}

void application_protocol_process(void)
{
  cesc_protocol_process();
}

void application_status_process(void)
{
  power_stage_process();
  cesc_protocol_periodic();
  motor_config_store_process();
  update_heartbeat();
  update_power_stage_indicator();
}

void application_process(void)
{
  cesc_protocol_process();
  angle_sensor_process();
  power_stage_process();
  cesc_protocol_periodic();
  motor_config_store_process();
  update_heartbeat();
  update_power_stage_indicator();
}

void application_usb_receive(const uint8_t *data, uint32_t length)
{
  cesc_protocol_receive(data, length);
  if (osKernelGetState() == osKernelRunning) {
    (void)osSemaphoreRelease(usbRxSemaphoreHandle);
  }
}
