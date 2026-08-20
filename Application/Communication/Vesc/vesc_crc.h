#ifndef VESC_CRC_H
#define VESC_CRC_H

#include <stdint.h>

uint16_t vesc_crc16(const uint8_t *data, uint32_t length);

#endif /* VESC_CRC_H */
