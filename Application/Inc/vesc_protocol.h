#ifndef VESC_PROTOCOL_H
#define VESC_PROTOCOL_H

#include <stdint.h>

void vesc_protocol_init(void);
void vesc_protocol_receive(const uint8_t *data, uint32_t length);
void vesc_protocol_process(void);

#endif /* VESC_PROTOCOL_H */
