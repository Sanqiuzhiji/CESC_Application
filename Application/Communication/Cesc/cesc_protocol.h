#ifndef CESC_PROTOCOL_H
#define CESC_PROTOCOL_H

#include <stdint.h>

void cesc_protocol_init(void);
void cesc_protocol_receive(const uint8_t *data, uint32_t length);
void cesc_protocol_process(void);
void cesc_protocol_periodic(void);

#endif /* CESC_PROTOCOL_H */
