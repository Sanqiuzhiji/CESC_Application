#include "vesc_protocol.h"

#include <stdbool.h>
#include <string.h>

#include "cesc_commands.h"
#include "firmware_update.h"
#include "main.h"
#include "usb_cdc_transport.h"
#include "vesc_crc.h"

enum
{
    RX_RING_SIZE = 4096U,
    MAX_PAYLOAD_SIZE = 1024U,
    FRAME_OVERHEAD_SIZE = 6U,
    FRAME_BUFFER_SIZE = MAX_PAYLOAD_SIZE + FRAME_OVERHEAD_SIZE,
    SHORT_FRAME_MARKER = 2U,
    LONG_FRAME_MARKER = 3U,
    FRAME_END_MARKER = 3U,
    SHORT_FRAME_HEADER_SIZE = 2U,
    LONG_FRAME_HEADER_SIZE = 3U,
    FRAME_TRAILER_SIZE = 3U
};

typedef enum
{
    COMM_FW_VERSION = 0,
    COMM_JUMP_TO_BOOTLOADER = 1,
    COMM_ERASE_NEW_APP = 2,
    COMM_WRITE_NEW_APP_DATA = 3
} command_id_t;

static volatile uint16_t rx_write_index;
static volatile uint16_t rx_read_index;
static uint8_t rx_ring[RX_RING_SIZE];
static uint8_t frame_buffer[FRAME_BUFFER_SIZE];
static uint16_t frame_length;

static uint32_t read_u32_be(const uint8_t* data)
{
    return ((uint32_t)data[0] << 24) |
        ((uint32_t)data[1] << 16) |
        ((uint32_t)data[2] << 8) |
        (uint32_t)data[3];
}

static void discard_frame_prefix(uint16_t length)
{
    memmove(frame_buffer, &frame_buffer[length], frame_length - length);
    frame_length -= length;
}

static void send_frame(const uint8_t* payload, uint16_t length)
{
    static uint8_t tx_buffer[FRAME_BUFFER_SIZE];
    uint16_t write_index = 0U;

    if (length == 0U || length > MAX_PAYLOAD_SIZE)
    {
        return;
    }

    if (length <= UINT8_MAX)
    {
        tx_buffer[write_index++] = SHORT_FRAME_MARKER;
        tx_buffer[write_index++] = (uint8_t)length;
    }
    else
    {
        tx_buffer[write_index++] = LONG_FRAME_MARKER;
        tx_buffer[write_index++] = (uint8_t)(length >> 8);
        tx_buffer[write_index++] = (uint8_t)length;
    }

    memcpy(&tx_buffer[write_index], payload, length);
    write_index += length;

    const uint16_t crc = vesc_crc16(payload, length);
    tx_buffer[write_index++] = (uint8_t)(crc >> 8);
    tx_buffer[write_index++] = (uint8_t)crc;
    tx_buffer[write_index++] = FRAME_END_MARKER;

    (void)usb_cdc_transport_send(tx_buffer, write_index);
}

static void send_update_result(command_id_t command, bool succeeded)
{
    const uint8_t reply[] = {(uint8_t)command, succeeded ? 1U : 0U};
    send_frame(reply, sizeof(reply));
}

static void process_command(uint8_t* payload, uint16_t length)
{
    if (length == 0U)
    {
        return;
    }

    if (cesc_commands_process(payload, length, send_frame))
    {
        return;
    }

    switch ((command_id_t)payload[0])
    {
    case COMM_FW_VERSION:
        {
            static const char hardware_name[] = "CESC";
            const uint32_t* uid = (const uint32_t*)UID_BASE;
            uint8_t reply[32];
            uint16_t reply_length = 0U;

            reply[reply_length++] = COMM_FW_VERSION;
            reply[reply_length++] = 3U;
            reply[reply_length++] = 61U;
            memcpy(&reply[reply_length], hardware_name, sizeof(hardware_name));
            reply_length += sizeof(hardware_name);
            memcpy(&reply[reply_length], uid, 12U);
            reply_length += 12U;
            reply[reply_length++] = 0U;
            send_frame(reply, reply_length);
            break;
        }

    case COMM_ERASE_NEW_APP:
        send_update_result(COMM_ERASE_NEW_APP,
                           length >= 5U &&
                           firmware_update_erase(read_u32_be(&payload[1])));
        break;

    case COMM_WRITE_NEW_APP_DATA:
        send_update_result(COMM_WRITE_NEW_APP_DATA,
                           length > 5U &&
                           firmware_update_write(read_u32_be(&payload[1]),
                                                 &payload[5], length - 5U));
        break;

    case COMM_JUMP_TO_BOOTLOADER:
        firmware_update_jump_to_bootloader();
        break;

    default:
        break;
    }
}

/* Returns true when bytes were consumed and another parse attempt is useful. */
static bool try_process_frame(void)
{
    uint16_t header_size;
    uint32_t payload_length;

    if (frame_length < SHORT_FRAME_HEADER_SIZE)
    {
        return false;
    }

    if (frame_buffer[0] == SHORT_FRAME_MARKER)
    {
        header_size = SHORT_FRAME_HEADER_SIZE;
        payload_length = frame_buffer[1];
    }
    else if (frame_buffer[0] == LONG_FRAME_MARKER)
    {
        if (frame_length < LONG_FRAME_HEADER_SIZE)
        {
            return false;
        }
        header_size = LONG_FRAME_HEADER_SIZE;
        payload_length = ((uint32_t)frame_buffer[1] << 8) | frame_buffer[2];
    }
    else
    {
        discard_frame_prefix(1U);
        return true;
    }

    if (payload_length == 0U || payload_length > MAX_PAYLOAD_SIZE)
    {
        discard_frame_prefix(1U);
        return true;
    }

    const uint32_t total_length = header_size + payload_length + FRAME_TRAILER_SIZE;
    if (frame_length < total_length)
    {
        return false;
    }

    const uint32_t crc_index = header_size + payload_length;
    const uint16_t received_crc =
        ((uint16_t)frame_buffer[crc_index] << 8) | frame_buffer[crc_index + 1U];
    const bool frame_is_valid =
        frame_buffer[total_length - 1U] == FRAME_END_MARKER &&
        vesc_crc16(&frame_buffer[header_size], payload_length) == received_crc;

    if (!frame_is_valid)
    {
        discard_frame_prefix(1U);
        return true;
    }

    process_command(&frame_buffer[header_size], (uint16_t)payload_length);
    discard_frame_prefix((uint16_t)total_length);
    return true;
}

void vesc_protocol_init(void)
{
    rx_write_index = 0U;
    rx_read_index = 0U;
    frame_length = 0U;
}

void vesc_protocol_receive(const uint8_t* data, uint32_t length)
{
    while (length-- > 0U)
    {
        const uint16_t next_index =
            (uint16_t)((rx_write_index + 1U) % RX_RING_SIZE);
        if (next_index == rx_read_index)
        {
            break;
        }

        rx_ring[rx_write_index] = *data++;
        rx_write_index = next_index;
    }
}

void vesc_protocol_process(void)
{
    while (rx_read_index != rx_write_index && frame_length < FRAME_BUFFER_SIZE)
    {
        frame_buffer[frame_length++] = rx_ring[rx_read_index];
        rx_read_index = (uint16_t)((rx_read_index + 1U) % RX_RING_SIZE);

        while (try_process_frame())
        {
        }
    }
}
