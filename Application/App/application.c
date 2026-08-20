#include "application.h"

#include <stdbool.h>
#include <stdio.h>

#include "angle_sensor.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "vesc_protocol.h"

enum {
  HEARTBEAT_PERIOD_MS = 1200U,
  HEARTBEAT_FIRST_FLASH_END_MS = 200U,
  HEARTBEAT_SECOND_FLASH_START_MS = 400U,
  HEARTBEAT_SECOND_FLASH_END_MS = 800U,
  ANGLE_PRINT_PERIOD_MS = 50U
};

static uint32_t heartbeat_started_at;
static uint32_t last_angle_print_ms;
static char angle_print_buffer[32];

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

static void print_angle_sample(void)
{
  const uint32_t now = HAL_GetTick();
  angle_sensor_sample_t sample;
  uint32_t centidegrees;
  int length;

  if ((uint32_t)(now - last_angle_print_ms) < ANGLE_PRINT_PERIOD_MS) {
    return;
  }
  last_angle_print_ms = now;

  if (!angle_sensor_get_sample(&sample)) {
    return;
  }

  centidegrees = (((uint32_t)sample.raw * 36000U) + 2048U) / 4096U;
  length = snprintf(angle_print_buffer, sizeof(angle_print_buffer),
                    "samples: %lu.%02lu\n",
                    (unsigned long)(centidegrees / 100U),
                    (unsigned long)(centidegrees % 100U));
  if ((length > 0) && ((size_t)length < sizeof(angle_print_buffer))) {
    (void)CDC_Transmit_FS((uint8_t *)angle_print_buffer, (uint16_t)length);
  }
}

void application_init(void)
{
  vesc_protocol_init();
  (void)angle_sensor_init();
  heartbeat_started_at = HAL_GetTick();
  last_angle_print_ms = heartbeat_started_at;
}

void application_process(void)
{
  vesc_protocol_process();
  angle_sensor_process();
  print_angle_sample();
  update_heartbeat();
}

void application_usb_receive(const uint8_t *data, uint32_t length)
{
  vesc_protocol_receive(data, length);
}
