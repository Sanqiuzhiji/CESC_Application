#include "application.h"

#include <stdbool.h>
#include "angle_sensor.h"
#include "cesc_protocol.h"
#include "main.h"
#include "cmsis_os2.h"

extern osSemaphoreId_t usbRxSemaphoreHandle;

enum {
  HEARTBEAT_PERIOD_MS = 1200U,
  HEARTBEAT_FIRST_FLASH_END_MS = 200U,
  HEARTBEAT_SECOND_FLASH_START_MS = 400U,
  HEARTBEAT_SECOND_FLASH_END_MS = 800U
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

void application_init(void)
{
  cesc_protocol_init();
  heartbeat_started_at = HAL_GetTick();
}

void application_protocol_process(void)
{
  cesc_protocol_process();
}

void application_status_process(void)
{
  cesc_protocol_periodic();
  update_heartbeat();
}

void application_process(void)
{
  cesc_protocol_process();
  angle_sensor_process();
  cesc_protocol_periodic();
  update_heartbeat();
}

void application_usb_receive(const uint8_t *data, uint32_t length)
{
  cesc_protocol_receive(data, length);
  if (osKernelGetState() == osKernelRunning) {
    (void)osSemaphoreRelease(usbRxSemaphoreHandle);
  }
}
