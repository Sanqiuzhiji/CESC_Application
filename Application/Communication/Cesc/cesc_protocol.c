#include "cesc_protocol.h"

#include <stdbool.h>
#include <string.h>

#include "angle_sensor.h"
#include "cmsis_os2.h"
#include "firmware_update.h"
#include "main.h"
#include "usb_cdc_transport.h"
#include "cesc_crc.h"

enum {
    MAGIC_0 = 0x43,
    MAGIC_1 = 0x45,
    PROTOCOL_VERSION = 1,
    HEADER_SIZE = 10,
    CRC_SIZE = 2,
    MAX_PAYLOAD = 1024,
    RX_RING_SIZE = 4096,
    RESPONSE_BUFFER_SIZE = MAX_PAYLOAD,
    MESSAGE_REQUEST = 0,
    MESSAGE_RESPONSE = 1,
    MESSAGE_STREAM = 3,
    SERVICE_SYSTEM = 0x00,
    SERVICE_FIRMWARE = 0x01,
    SERVICE_SENSOR = 0x02,
    SERVICE_TELEMETRY = 0x03,
    SERVICE_CONFIGURATION = 0x04,
    SERVICE_MOTOR = 0x05,
    SERVICE_DIAGNOSTIC = 0x06,
    SYSTEM_HELLO = 0x00,
    SYSTEM_GET_DEVICE_INFO = 0x01,
    SYSTEM_PING = 0x02,
    SYSTEM_GET_CAPABILITIES = 0x03,
    SYSTEM_GET_COMM_STATS = 0x04,
    SYSTEM_RESET = 0x05,
    FIRMWARE_BEGIN = 0x00,
    FIRMWARE_WRITE = 0x01,
    FIRMWARE_FINISH = 0x02,
    FIRMWARE_ACTIVATE = 0x03,
    FIRMWARE_ABORT = 0x04,
    FIRMWARE_GET_STATUS = 0x05,
    SENSOR_ENUMERATE = 0x00,
    SENSOR_GET_SAMPLE = 0x01,
    SENSOR_GET_STATUS = 0x02,
    TELEMETRY_ENUM_CHANNELS = 0x00,
    TELEMETRY_SUBSCRIBE = 0x01,
    TELEMETRY_UNSUBSCRIBE = 0x02,
    TELEMETRY_STOP_ALL = 0x03,
    TELEMETRY_GET_STREAM_STATUS = 0x04,
    TELEMETRY_STREAM_DATA = 0x80,
    STATUS_OK = 0,
    STATUS_INVALID_SERVICE = 1,
    STATUS_INVALID_COMMAND = 2,
    STATUS_INVALID_LENGTH = 3,
    STATUS_INVALID_ARGUMENT = 4,
    STATUS_NOT_READY = 5,
    STATUS_BUSY = 6,
    STATUS_NOT_SUPPORTED = 10,
    STATUS_VERSION_MISMATCH = 11,
    STATUS_OUT_OF_RANGE = 14,
    STATUS_VERIFY_FAILED = 15,
    CAP_FIRMWARE = 1U << 0,
    CAP_SENSOR = 1U << 1,
    CAP_TELEMETRY = 1U << 2,
    DATA_UINT8 = 0,
    DATA_UINT16 = 2,
    DATA_FLOAT32 = 8,
    MAX_STREAM_CHANNELS = 8,
    CHANNEL_RAW_ANGLE = 0,
    CHANNEL_ANGLE_DEGREES = 1,
    CHANNEL_SENSOR_STATUS = 2,
    CHANNEL_COUNT = 3,
    FIRMWARE_IDLE = 0,
    FIRMWARE_RECEIVING = 2,
    FIRMWARE_READY = 4,
    FIRMWARE_FAILED = 5
};

typedef struct {
    volatile uint32_t received_bytes;
    uint32_t transmitted_bytes;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t discarded_bytes;
    volatile uint32_t receive_overflows;
    uint32_t transmit_overflows;
    uint32_t unsupported_requests;
} comm_stats_t;

typedef struct {
    bool active;
    uint16_t id;
    uint32_t period_us;
    uint32_t next_due_ms;
    uint32_t frame_sequence;
    uint32_t sample_sequence;
    uint32_t produced_frames;
    uint32_t dropped_frames;
    uint32_t produced_samples;
    uint8_t channel_count;
    uint16_t channels[MAX_STREAM_CHANNELS];
} telemetry_stream_t;

static volatile uint16_t rx_write_index;
static volatile uint16_t rx_read_index;
static uint8_t rx_ring[RX_RING_SIZE];
static uint8_t payload_buffer[MAX_PAYLOAD];
static uint8_t response_buffer[RESPONSE_BUFFER_SIZE];
static uint8_t response_frame[MAX_PAYLOAD + HEADER_SIZE + CRC_SIZE];
static uint8_t stream_frame[256];
static comm_stats_t stats;
static telemetry_stream_t stream;
static bool session_ready;
static uint32_t session_id;
static uint32_t update_session_id;
static uint16_t update_chunk_size;
static uint8_t firmware_state;
static uint8_t firmware_last_error;

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static void write_u64(uint8_t *data, uint64_t value)
{
    for (uint32_t index = 0U; index < 8U; ++index)
    {
        data[index] = (uint8_t)(value >> (index * 8U));
    }
}

static void write_float(uint8_t *data, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    write_u32(data, bits);
}

static uint16_t append_string(uint8_t *data, uint16_t index,
                              const char *value)
{
    const size_t raw_length = strlen(value);
    const uint8_t length = raw_length > 255U ? 255U : (uint8_t)raw_length;
    data[index++] = length;
    memcpy(&data[index], value, length);
    return (uint16_t)(index + length);
}

static uint16_t ring_count(void)
{
    const uint16_t write = rx_write_index;
    const uint16_t read = rx_read_index;
    return write >= read ? (uint16_t)(write - read)
                         : (uint16_t)(RX_RING_SIZE - read + write);
}

static uint8_t ring_peek(uint16_t offset)
{
    return rx_ring[(uint16_t)((rx_read_index + offset) % RX_RING_SIZE)];
}

static void ring_discard(uint16_t count)
{
    rx_read_index = (uint16_t)((rx_read_index + count) % RX_RING_SIZE);
}

static bool transmit_frame(uint8_t *frame, uint16_t capacity,
                           uint8_t message_type, uint8_t service,
                           uint8_t command, uint16_t sequence,
                           const uint8_t *payload, uint16_t payload_length)
{
    const uint16_t frame_length =
        (uint16_t)(HEADER_SIZE + payload_length + CRC_SIZE);
    uint16_t crc;

    if ((payload_length > MAX_PAYLOAD) || (frame_length > capacity))
    {
        ++stats.transmit_overflows;
        return false;
    }
    frame[0] = MAGIC_0;
    frame[1] = MAGIC_1;
    frame[2] = PROTOCOL_VERSION;
    frame[3] = message_type;
    frame[4] = service;
    frame[5] = command;
    write_u16(&frame[6], sequence);
    write_u16(&frame[8], payload_length);
    if (payload_length > 0U)
    {
        memcpy(&frame[HEADER_SIZE], payload, payload_length);
    }
    crc = cesc_crc16(&frame[2], (uint32_t)(8U + payload_length));
    write_u16(&frame[HEADER_SIZE + payload_length], crc);
    if (!usb_cdc_transport_send(frame, frame_length))
    {
        ++stats.transmit_overflows;
        return false;
    }
    stats.transmitted_bytes += frame_length;
    return true;
}

static void send_response(uint8_t service, uint8_t command, uint16_t sequence,
                          uint16_t status, uint16_t data_length)
{
    write_u16(response_buffer, status);
    (void)transmit_frame(response_frame, sizeof(response_frame),
                         MESSAGE_RESPONSE, service, command, sequence,
                         response_buffer, (uint16_t)(data_length + 2U));
}

static void system_service(uint8_t command, uint16_t sequence,
                           const uint8_t *payload, uint16_t length)
{
    uint16_t index = 2U;
    const uint64_t capabilities = CAP_FIRMWARE | CAP_SENSOR | CAP_TELEMETRY;

    switch (command)
    {
    case SYSTEM_HELLO:
        if (length != 6U)
        {
            send_response(SERVICE_SYSTEM, command, sequence,
                          STATUS_INVALID_LENGTH, 0U);
        }
        else if ((payload[0] > PROTOCOL_VERSION) ||
                 (payload[1] < PROTOCOL_VERSION))
        {
            send_response(SERVICE_SYSTEM, command, sequence,
                          STATUS_VERSION_MISMATCH, 0U);
        }
        else
        {
            session_ready = true;
            response_buffer[index++] = PROTOCOL_VERSION;
            write_u16(&response_buffer[index], MAX_PAYLOAD); index += 2U;
            write_u64(&response_buffer[index], capabilities); index += 8U;
            write_u32(&response_buffer[index], session_id); index += 4U;
            send_response(SERVICE_SYSTEM, command, sequence, STATUS_OK,
                          (uint16_t)(index - 2U));
        }
        break;

    case SYSTEM_GET_DEVICE_INFO:
        if (length != 0U) { send_response(SERVICE_SYSTEM, command, sequence, STATUS_INVALID_LENGTH, 0U); break; }
        write_u16(&response_buffer[index], 1U); index += 2U;
        write_u16(&response_buffer[index], 0U); index += 2U;
        write_u16(&response_buffer[index], 0U); index += 2U;
        write_u16(&response_buffer[index], 0U); index += 2U;
        write_u16(&response_buffer[index], 0U); index += 2U;
        write_u16(&response_buffer[index], 0U); index += 2U;
        memcpy(&response_buffer[index], (const void *)UID_BASE, 12U); index += 12U;
        index = append_string(response_buffer, index, "CESC");
        index = append_string(response_buffer, index, "dev");
        send_response(SERVICE_SYSTEM, command, sequence, STATUS_OK,
                      (uint16_t)(index - 2U));
        break;

    case SYSTEM_PING:
        if (length != 4U) { send_response(SERVICE_SYSTEM, command, sequence, STATUS_INVALID_LENGTH, 0U); break; }
        memcpy(&response_buffer[index], payload, 4U); index += 4U;
        write_u32(&response_buffer[index], HAL_GetTick()); index += 4U;
        send_response(SERVICE_SYSTEM, command, sequence, STATUS_OK,
                      (uint16_t)(index - 2U));
        break;

    case SYSTEM_GET_CAPABILITIES:
        if (length != 0U) { send_response(SERVICE_SYSTEM, command, sequence, STATUS_INVALID_LENGTH, 0U); break; }
        write_u64(&response_buffer[index], capabilities); index += 8U;
        write_u16(&response_buffer[index], MAX_PAYLOAD); index += 2U;
        response_buffer[index++] = 8U;
        response_buffer[index++] = 0U;
        send_response(SERVICE_SYSTEM, command, sequence, STATUS_OK,
                      (uint16_t)(index - 2U));
        break;

    case SYSTEM_GET_COMM_STATS:
        if (length != 0U) { send_response(SERVICE_SYSTEM, command, sequence, STATUS_INVALID_LENGTH, 0U); break; }
        write_u32(&response_buffer[index], stats.received_bytes); index += 4U;
        write_u32(&response_buffer[index], stats.transmitted_bytes); index += 4U;
        write_u32(&response_buffer[index], stats.valid_frames); index += 4U;
        write_u32(&response_buffer[index], stats.crc_errors); index += 4U;
        write_u32(&response_buffer[index], stats.length_errors); index += 4U;
        write_u32(&response_buffer[index], stats.discarded_bytes); index += 4U;
        write_u32(&response_buffer[index], stats.receive_overflows); index += 4U;
        write_u32(&response_buffer[index], stats.transmit_overflows); index += 4U;
        write_u32(&response_buffer[index], stats.unsupported_requests); index += 4U;
        send_response(SERVICE_SYSTEM, command, sequence, STATUS_OK,
                      (uint16_t)(index - 2U));
        break;

    case SYSTEM_RESET:
        if ((length != 3U) || (payload[0] > 1U)) { send_response(SERVICE_SYSTEM, command, sequence, length != 3U ? STATUS_INVALID_LENGTH : STATUS_INVALID_ARGUMENT, 0U); break; }
        send_response(SERVICE_SYSTEM, command, sequence, STATUS_OK, 0U);
        {
            uint16_t delay = read_u16(&payload[1]);
            if (delay < 50U) delay = 50U;
            if (delay > 5000U) delay = 5000U;
            (void)osDelay(delay);
        }
        if (payload[0] == 1U) firmware_update_jump_to_bootloader();
        NVIC_SystemReset();
        break;

    default:
        ++stats.unsupported_requests;
        send_response(SERVICE_SYSTEM, command, sequence,
                      STATUS_INVALID_COMMAND, 0U);
        break;
    }
}

static void firmware_service(uint8_t command, uint16_t sequence,
                             const uint8_t *payload, uint16_t length)
{
    uint16_t index = 2U;
    uint16_t status = STATUS_OK;

    switch (command)
    {
    case FIRMWARE_BEGIN:
        if (length != 12U) { status = STATUS_INVALID_LENGTH; break; }
        {
            uint16_t requested_chunk = read_u16(&payload[8]);
            if ((requested_chunk == 0U) ||
                (requested_chunk > (MAX_PAYLOAD - 10U)))
                requested_chunk = MAX_PAYLOAD - 10U;
            if (read_u16(&payload[10]) != 0U)
            {
                status = STATUS_INVALID_ARGUMENT;
                break;
            }
        if ((firmware_state == FIRMWARE_RECEIVING) &&
            (firmware_update_size() == read_u32(payload)) &&
            (firmware_update_expected_crc32() == read_u32(&payload[4])) &&
            (update_chunk_size == requested_chunk))
        {
            /* Idempotent retry of the active BEGIN. */
        }
        else if (firmware_state == FIRMWARE_RECEIVING)
        {
            status = STATUS_BUSY;
            break;
        }
        else if (!firmware_update_begin(read_u32(payload), read_u32(&payload[4])))
        {
            firmware_state = FIRMWARE_FAILED;
            firmware_last_error = STATUS_OUT_OF_RANGE;
            status = STATUS_OUT_OF_RANGE;
            break;
        }
        else
        {
            update_session_id = session_id ^ HAL_GetTick() ^ read_u32(payload);
            if (update_session_id == 0U) update_session_id = 1U;
            update_chunk_size = requested_chunk;
            firmware_state = FIRMWARE_RECEIVING;
            firmware_last_error = 0U;
            stream.active = false;
        }
        }
        write_u32(&response_buffer[index], update_session_id); index += 4U;
        write_u16(&response_buffer[index], update_chunk_size); index += 2U;
        write_u32(&response_buffer[index], firmware_update_next_offset()); index += 4U;
        break;

    case FIRMWARE_WRITE:
        if (length < 11U) { status = STATUS_INVALID_LENGTH; break; }
        if ((read_u32(payload) != update_session_id) ||
            (firmware_state != FIRMWARE_RECEIVING)) { status = STATUS_NOT_READY; break; }
        if ((read_u16(&payload[8]) != (uint16_t)(length - 10U)) ||
            (read_u16(&payload[8]) > update_chunk_size) ||
            !firmware_update_write_block(read_u32(&payload[4]), &payload[10],
                                         (uint32_t)(length - 10U)))
        {
            firmware_last_error = STATUS_INVALID_ARGUMENT;
            status = STATUS_INVALID_ARGUMENT;
            break;
        }
        write_u32(&response_buffer[index], firmware_update_next_offset()); index += 4U;
        break;

    case FIRMWARE_FINISH:
        if (length != 4U) { status = STATUS_INVALID_LENGTH; break; }
        if ((read_u32(payload) != update_session_id) ||
            (firmware_state != FIRMWARE_RECEIVING)) { status = STATUS_NOT_READY; break; }
        {
            uint32_t crc;
            if (!firmware_update_finish(&crc))
            {
                firmware_state = FIRMWARE_FAILED;
                firmware_last_error = STATUS_VERIFY_FAILED;
                status = STATUS_VERIFY_FAILED;
                break;
            }
            firmware_state = FIRMWARE_READY;
            write_u32(&response_buffer[index], firmware_update_size()); index += 4U;
            write_u32(&response_buffer[index], crc); index += 4U;
        }
        break;

    case FIRMWARE_ACTIVATE:
        if (length != 6U) { status = STATUS_INVALID_LENGTH; break; }
        if ((read_u32(payload) != update_session_id) ||
            (firmware_state != FIRMWARE_READY)) { status = STATUS_NOT_READY; break; }
        send_response(SERVICE_FIRMWARE, command, sequence, STATUS_OK, 0U);
        {
            uint16_t delay = read_u16(&payload[4]);
            if (delay < 50U) delay = 50U;
            if (delay > 5000U) delay = 5000U;
            (void)osDelay(delay);
        }
        firmware_update_jump_to_bootloader();
        break;

    case FIRMWARE_ABORT:
        if (length != 4U) { status = STATUS_INVALID_LENGTH; break; }
        if ((update_session_id != 0U) && (read_u32(payload) != update_session_id)) { status = STATUS_INVALID_ARGUMENT; break; }
        firmware_update_abort();
        firmware_state = FIRMWARE_IDLE;
        firmware_last_error = 0U;
        update_session_id = 0U;
        break;

    case FIRMWARE_GET_STATUS:
        if (length != 0U) { status = STATUS_INVALID_LENGTH; break; }
        response_buffer[index++] = firmware_state;
        response_buffer[index++] = firmware_last_error;
        write_u16(&response_buffer[index], update_chunk_size); index += 2U;
        write_u32(&response_buffer[index], update_session_id); index += 4U;
        write_u32(&response_buffer[index], firmware_update_size()); index += 4U;
        write_u32(&response_buffer[index], firmware_update_next_offset()); index += 4U;
        write_u32(&response_buffer[index], firmware_update_expected_crc32()); index += 4U;
        break;

    default:
        ++stats.unsupported_requests;
        status = STATUS_INVALID_COMMAND;
        break;
    }
    send_response(SERVICE_FIRMWARE, command, sequence, status,
                  status == STATUS_OK ? (uint16_t)(index - 2U) : 0U);
}

static void sensor_service(uint8_t command, uint16_t sequence,
                           const uint8_t *payload, uint16_t length)
{
    uint16_t index = 2U;
    angle_sensor_sample_t sample;

    switch (command)
    {
    case SENSOR_ENUMERATE:
        if (length != 0U) { send_response(SERVICE_SENSOR, command, sequence, STATUS_INVALID_LENGTH, 0U); return; }
        response_buffer[index++] = 1U;
        response_buffer[index++] = 0U;
        response_buffer[index++] = 1U;
        response_buffer[index++] = 1U;
        index = append_string(response_buffer, index, "shaft-angle");
        break;
    case SENSOR_GET_SAMPLE:
        if (length != 1U) { send_response(SERVICE_SENSOR, command, sequence, STATUS_INVALID_LENGTH, 0U); return; }
        if (payload[0] != 0U) { send_response(SERVICE_SENSOR, command, sequence, STATUS_INVALID_ARGUMENT, 0U); return; }
        if (!angle_sensor_get_sample(&sample)) { send_response(SERVICE_SENSOR, command, sequence, STATUS_NOT_READY, 0U); return; }
        response_buffer[index++] = 0U;
        response_buffer[index++] = 1U;
        response_buffer[index++] = (uint8_t)sample.status;
        response_buffer[index++] = 0U;
        write_u16(&response_buffer[index], sample.raw); index += 2U;
        write_float(&response_buffer[index], sample.degrees); index += 4U;
        write_u64(&response_buffer[index], (uint64_t)sample.timestamp_ms * 1000U); index += 8U;
        break;
    case SENSOR_GET_STATUS:
        if (length != 1U) { send_response(SERVICE_SENSOR, command, sequence, STATUS_INVALID_LENGTH, 0U); return; }
        if (payload[0] != 0U) { send_response(SERVICE_SENSOR, command, sequence, STATUS_INVALID_ARGUMENT, 0U); return; }
        response_buffer[index++] = 0U;
        response_buffer[index++] = (uint8_t)angle_sensor_get_status();
        write_u16(&response_buffer[index], 0U); index += 2U;
        if (angle_sensor_get_sample(&sample)) write_u32(&response_buffer[index], (HAL_GetTick() - sample.timestamp_ms) * 1000U);
        else write_u32(&response_buffer[index], UINT32_MAX);
        index += 4U;
        write_u32(&response_buffer[index], 0U); index += 4U;
        break;
    default:
        ++stats.unsupported_requests;
        send_response(SERVICE_SENSOR, command, sequence, STATUS_INVALID_COMMAND, 0U);
        return;
    }
    send_response(SERVICE_SENSOR, command, sequence, STATUS_OK,
                  (uint16_t)(index - 2U));
}

static uint16_t append_channel_descriptor(uint8_t *data, uint16_t index,
                                          uint16_t channel)
{
    static const char *const names[] = {"shaft_angle_raw", "shaft_angle", "shaft_sensor_status"};
    static const char *const units[] = {"count", "deg", "enum"};
    static const uint8_t types[] = {DATA_UINT16, DATA_FLOAT32, DATA_UINT8};
    write_u16(&data[index], channel); index += 2U;
    data[index++] = types[channel];
    write_float(&data[index], 1.0F); index += 4U;
    write_float(&data[index], 0.0F); index += 4U;
    index = append_string(data, index, names[channel]);
    return append_string(data, index, units[channel]);
}

static void telemetry_service(uint8_t command, uint16_t sequence,
                              const uint8_t *payload, uint16_t length)
{
    uint16_t index = 2U;
    uint16_t status = STATUS_OK;

    switch (command)
    {
    case TELEMETRY_ENUM_CHANNELS:
        if (length != 3U) { status = STATUS_INVALID_LENGTH; break; }
        {
            const uint16_t first = read_u16(payload);
            uint8_t count = payload[2];
            if (first >= CHANNEL_COUNT) count = 0U;
            else if (count > (CHANNEL_COUNT - first)) count = (uint8_t)(CHANNEL_COUNT - first);
            write_u16(&response_buffer[index], CHANNEL_COUNT); index += 2U;
            response_buffer[index++] = count;
            for (uint16_t channel = first; channel < (uint16_t)(first + count); ++channel)
                index = append_channel_descriptor(response_buffer, index, channel);
        }
        break;
    case TELEMETRY_SUBSCRIBE:
        if (length < 6U) { status = STATUS_INVALID_LENGTH; break; }
        {
            uint32_t period = read_u32(payload);
            const uint8_t samples_per_frame = payload[4];
            const uint8_t count = payload[5];
            if ((count == 0U) || (count > MAX_STREAM_CHANNELS) ||
                (length != (uint16_t)(6U + count * 2U)) ||
                (samples_per_frame != 1U)) { status = STATUS_INVALID_ARGUMENT; break; }
            if (period < 10000U) period = 10000U;
            for (uint8_t i = 0U; i < count; ++i)
            {
                const uint16_t channel = read_u16(&payload[6U + i * 2U]);
                if (channel >= CHANNEL_COUNT) { status = STATUS_INVALID_ARGUMENT; break; }
                stream.channels[i] = channel;
            }
            if (status != STATUS_OK) break;
            stream.active = true;
            stream.id = 1U;
            stream.period_us = period;
            stream.next_due_ms = HAL_GetTick();
            stream.frame_sequence = 0U;
            stream.sample_sequence = 0U;
            stream.produced_frames = 0U;
            stream.dropped_frames = 0U;
            stream.produced_samples = 0U;
            stream.channel_count = count;
            write_u16(&response_buffer[index], stream.id); index += 2U;
            write_u32(&response_buffer[index], stream.period_us); index += 4U;
            response_buffer[index++] = 1U;
            response_buffer[index++] = count;
            for (uint8_t i = 0U; i < count; ++i) { write_u16(&response_buffer[index], stream.channels[i]); index += 2U; }
        }
        break;
    case TELEMETRY_UNSUBSCRIBE:
        if (length != 2U) { status = STATUS_INVALID_LENGTH; break; }
        if (!stream.active || (read_u16(payload) != stream.id)) { status = STATUS_NOT_READY; break; }
        stream.active = false;
        break;
    case TELEMETRY_STOP_ALL:
        if (length != 0U) { status = STATUS_INVALID_LENGTH; break; }
        stream.active = false;
        break;
    case TELEMETRY_GET_STREAM_STATUS:
        if (length != 2U) { status = STATUS_INVALID_LENGTH; break; }
        if (!stream.active || (read_u16(payload) != stream.id)) { status = STATUS_NOT_READY; break; }
        write_u16(&response_buffer[index], stream.id); index += 2U;
        write_u32(&response_buffer[index], stream.produced_frames); index += 4U;
        write_u32(&response_buffer[index], stream.dropped_frames); index += 4U;
        write_u32(&response_buffer[index], stream.produced_samples); index += 4U;
        break;
    default:
        ++stats.unsupported_requests;
        status = STATUS_INVALID_COMMAND;
        break;
    }
    send_response(SERVICE_TELEMETRY, command, sequence, status,
                  status == STATUS_OK ? (uint16_t)(index - 2U) : 0U);
}

static void dispatch_request(uint8_t service, uint8_t command,
                             uint16_t sequence, const uint8_t *payload,
                             uint16_t length)
{
    if ((service != SERVICE_SYSTEM) && !session_ready)
    {
        send_response(service, command, sequence, STATUS_NOT_READY, 0U);
        return;
    }
    switch (service)
    {
    case SERVICE_SYSTEM: system_service(command, sequence, payload, length); break;
    case SERVICE_FIRMWARE: firmware_service(command, sequence, payload, length); break;
    case SERVICE_SENSOR: sensor_service(command, sequence, payload, length); break;
    case SERVICE_TELEMETRY: telemetry_service(command, sequence, payload, length); break;
    case SERVICE_CONFIGURATION:
    case SERVICE_MOTOR:
    case SERVICE_DIAGNOSTIC:
        send_response(service, command, sequence, STATUS_NOT_SUPPORTED, 0U);
        break;
    default:
        ++stats.unsupported_requests;
        send_response(service, command, sequence, STATUS_INVALID_SERVICE, 0U);
        break;
    }
}

void cesc_protocol_init(void)
{
    rx_write_index = 0U;
    rx_read_index = 0U;
    memset(&stats, 0, sizeof(stats));
    memset(&stream, 0, sizeof(stream));
    session_ready = false;
    session_id = *(const uint32_t *)UID_BASE ^
                 *(const uint32_t *)(UID_BASE + 4U) ^
                 *(const uint32_t *)(UID_BASE + 8U) ^ HAL_GetTick();
    if (session_id == 0U) session_id = 1U;
    update_session_id = 0U;
    update_chunk_size = MAX_PAYLOAD - 10U;
    firmware_state = FIRMWARE_IDLE;
    firmware_last_error = 0U;
}

void cesc_protocol_receive(const uint8_t *data, uint32_t length)
{
    if (data == NULL) return;
    stats.received_bytes += length;
    while (length-- > 0U)
    {
        const uint16_t next = (uint16_t)((rx_write_index + 1U) % RX_RING_SIZE);
        if (next == rx_read_index)
        {
            ++stats.receive_overflows;
            break;
        }
        rx_ring[rx_write_index] = *data++;
        __DMB();
        rx_write_index = next;
    }
}

void cesc_protocol_process(void)
{
    for (;;)
    {
        uint16_t available = ring_count();
        uint16_t payload_length;
        uint16_t frame_length;
        uint16_t received_crc;
        uint16_t calculated_crc;

        if (available < 2U) return;
        if ((ring_peek(0U) != MAGIC_0) || (ring_peek(1U) != MAGIC_1))
        {
            ring_discard(1U);
            ++stats.discarded_bytes;
            continue;
        }
        if (available < HEADER_SIZE) return;
        payload_length = (uint16_t)ring_peek(8U) |
                         ((uint16_t)ring_peek(9U) << 8U);
        if ((ring_peek(2U) != PROTOCOL_VERSION) ||
            (ring_peek(3U) > MESSAGE_STREAM) ||
            (payload_length > MAX_PAYLOAD))
        {
            ring_discard(1U);
            ++stats.length_errors;
            ++stats.discarded_bytes;
            continue;
        }
        frame_length = (uint16_t)(HEADER_SIZE + payload_length + CRC_SIZE);
        if (available < frame_length) return;
        for (uint16_t i = 0U; i < payload_length; ++i)
            payload_buffer[i] = ring_peek((uint16_t)(HEADER_SIZE + i));
        {
            uint8_t crc_input[8U + MAX_PAYLOAD];
            for (uint16_t i = 0U; i < 8U; ++i) crc_input[i] = ring_peek((uint16_t)(2U + i));
            memcpy(&crc_input[8], payload_buffer, payload_length);
            calculated_crc = cesc_crc16(crc_input, (uint32_t)(8U + payload_length));
        }
        received_crc = (uint16_t)ring_peek((uint16_t)(HEADER_SIZE + payload_length)) |
                       ((uint16_t)ring_peek((uint16_t)(HEADER_SIZE + payload_length + 1U)) << 8U);
        if (received_crc != calculated_crc)
        {
            ring_discard(1U);
            ++stats.crc_errors;
            ++stats.discarded_bytes;
            continue;
        }
        ++stats.valid_frames;
        if (ring_peek(3U) == MESSAGE_REQUEST)
        {
            dispatch_request(ring_peek(4U), ring_peek(5U),
                             (uint16_t)ring_peek(6U) |
                                 ((uint16_t)ring_peek(7U) << 8U),
                             payload_buffer, payload_length);
        }
        ring_discard(frame_length);
    }
}

void cesc_protocol_periodic(void)
{
    uint8_t payload[128];
    uint16_t index = 0U;
    const uint32_t now = HAL_GetTick();
    angle_sensor_sample_t sample;

    if (!stream.active || ((int32_t)(now - stream.next_due_ms) < 0)) return;
    stream.next_due_ms = now + (stream.period_us + 999U) / 1000U;
    if (!angle_sensor_get_sample(&sample)) return;

    write_u16(&payload[index], stream.id); index += 2U;
    write_u32(&payload[index], stream.sample_sequence); index += 4U;
    write_u64(&payload[index], (uint64_t)sample.timestamp_ms * 1000U); index += 8U;
    write_u32(&payload[index], stream.period_us); index += 4U;
    payload[index++] = 1U;
    payload[index++] = stream.channel_count;
    for (uint8_t i = 0U; i < stream.channel_count; ++i)
    {
        switch (stream.channels[i])
        {
        case CHANNEL_RAW_ANGLE: write_u16(&payload[index], sample.raw); index += 2U; break;
        case CHANNEL_ANGLE_DEGREES: write_float(&payload[index], sample.degrees); index += 4U; break;
        case CHANNEL_SENSOR_STATUS: payload[index++] = (uint8_t)sample.status; break;
        default: return;
        }
    }
    if (transmit_frame(stream_frame, sizeof(stream_frame), MESSAGE_STREAM,
                       SERVICE_TELEMETRY, TELEMETRY_STREAM_DATA,
                       (uint16_t)stream.frame_sequence, payload, index))
    {
        ++stream.produced_frames;
        ++stream.produced_samples;
        ++stream.frame_sequence;
        ++stream.sample_sequence;
    }
    else
    {
        ++stream.dropped_frames;
    }
}
