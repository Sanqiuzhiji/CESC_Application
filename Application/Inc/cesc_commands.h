#ifndef CESC_COMMANDS_H
#define CESC_COMMANDS_H

#include <stdbool.h>
#include <stdint.h>

enum {
  CESC_PROTOCOL_MARKER = 0xC8U,
  CESC_PROTOCOL_VERSION = 1U
};

typedef void (*cesc_reply_fn)(const uint8_t *payload, uint16_t length);

/** Return true when the payload belongs to the CESC application protocol. */
bool cesc_commands_process(const uint8_t *payload, uint16_t length,
                           cesc_reply_fn reply);

#endif /* CESC_COMMANDS_H */
