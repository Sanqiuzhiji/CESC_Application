#include "cesc_commands.h"

#include <string.h>

#include "angle_sensor.h"
#include "main.h"

enum {
  CESC_HEADER_SIZE = 7U,
  CESC_RESPONSE_HEADER_SIZE = 8U,
  CESC_MESSAGE_REQUEST = 0U,
  CESC_MESSAGE_RESPONSE = 1U,
  CESC_SERVICE_SYSTEM = 0U,
  CESC_SERVICE_SENSOR = 1U,
  CESC_SYSTEM_PING = 0U,
  CESC_SYSTEM_GET_INFO = 1U,
  CESC_SENSOR_GET_SAMPLE = 0U,
  CESC_STATUS_OK = 0U,
  CESC_STATUS_BAD_LENGTH = 1U,
  CESC_STATUS_BAD_ARGUMENT = 2U,
  CESC_STATUS_UNSUPPORTED = 3U,
  CESC_STATUS_NOT_READY = 4U,
  CESC_STATUS_BAD_VERSION = 5U,
  CESC_SENSOR_ID_SHAFT = 0U,
  CESC_SENSOR_TYPE_AS5600 = 1U
};

static void write_u16_be(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value >> 8);
  data[1] = (uint8_t)value;
}

static void write_u32_be(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value >> 24);
  data[1] = (uint8_t)(value >> 16);
  data[2] = (uint8_t)(value >> 8);
  data[3] = (uint8_t)value;
}

static uint16_t begin_response(uint8_t *response, const uint8_t *request,
                               uint8_t status)
{
  response[0] = CESC_PROTOCOL_MARKER;
  response[1] = CESC_PROTOCOL_VERSION;
  response[2] = CESC_MESSAGE_RESPONSE;
  response[3] = request[3];
  response[4] = request[4];
  response[5] = request[5];
  response[6] = request[6];
  response[7] = status;
  return CESC_RESPONSE_HEADER_SIZE;
}

static void process_system(const uint8_t *request, uint16_t length,
                           cesc_reply_fn reply)
{
  static const char device_name[] = "CESC";
  uint8_t response[32];
  uint16_t index;

  if (length != CESC_HEADER_SIZE) {
    index = begin_response(response, request, CESC_STATUS_BAD_LENGTH);
  } else if (request[6] == CESC_SYSTEM_PING) {
    index = begin_response(response, request, CESC_STATUS_OK);
    write_u32_be(&response[index], HAL_GetTick());
    index += 4U;
  } else if (request[6] == CESC_SYSTEM_GET_INFO) {
    index = begin_response(response, request, CESC_STATUS_OK);
    response[index++] = 1U; /* Firmware major. */
    response[index++] = 0U; /* Firmware minor. */
    response[index++] = 0U; /* Firmware patch. */
    response[index++] = (uint8_t)(sizeof(device_name) - 1U);
    memcpy(&response[index], device_name, sizeof(device_name) - 1U);
    index += sizeof(device_name) - 1U;
  } else {
    index = begin_response(response, request, CESC_STATUS_UNSUPPORTED);
  }
  reply(response, index);
}

static void process_sensor(const uint8_t *request, uint16_t length,
                           cesc_reply_fn reply)
{
  uint8_t response[24];
  uint16_t index;
  angle_sensor_sample_t sample;

  if (request[6] != CESC_SENSOR_GET_SAMPLE) {
    index = begin_response(response, request, CESC_STATUS_UNSUPPORTED);
  } else if ((length != (CESC_HEADER_SIZE + 1U)) ||
             (request[7] != CESC_SENSOR_ID_SHAFT)) {
    index = begin_response(response, request,
                           length != (CESC_HEADER_SIZE + 1U)
                               ? CESC_STATUS_BAD_LENGTH
                               : CESC_STATUS_BAD_ARGUMENT);
  } else if (!angle_sensor_get_sample(&sample)) {
    index = begin_response(response, request, CESC_STATUS_NOT_READY);
  } else {
    const uint32_t angle_millidegrees =
        (((uint32_t)sample.raw * 360000U) + 2048U) / 4096U;
    index = begin_response(response, request, CESC_STATUS_OK);
    response[index++] = CESC_SENSOR_ID_SHAFT;
    response[index++] = CESC_SENSOR_TYPE_AS5600;
    response[index++] = (uint8_t)sample.status;
    write_u16_be(&response[index], sample.raw);
    index += 2U;
    write_u32_be(&response[index], angle_millidegrees);
    index += 4U;
    write_u32_be(&response[index], sample.timestamp_ms);
    index += 4U;
  }
  reply(response, index);
}

bool cesc_commands_process(const uint8_t *payload, uint16_t length,
                           cesc_reply_fn reply)
{
  uint8_t response[CESC_RESPONSE_HEADER_SIZE];
  uint16_t response_length;

  if ((payload == NULL) || (length == 0U) ||
      (payload[0] != CESC_PROTOCOL_MARKER)) {
    return false;
  }
  if ((reply == NULL) || (length < CESC_HEADER_SIZE) ||
      (payload[2] != CESC_MESSAGE_REQUEST)) {
    return true;
  }
  if (payload[1] != CESC_PROTOCOL_VERSION) {
    response_length = begin_response(response, payload, CESC_STATUS_BAD_VERSION);
    reply(response, response_length);
    return true;
  }

  switch (payload[5]) {
    case CESC_SERVICE_SYSTEM:
      process_system(payload, length, reply);
      break;
    case CESC_SERVICE_SENSOR:
      process_sensor(payload, length, reply);
      break;
    default:
      response_length = begin_response(response, payload,
                                       CESC_STATUS_UNSUPPORTED);
      reply(response, response_length);
      break;
  }
  return true;
}
