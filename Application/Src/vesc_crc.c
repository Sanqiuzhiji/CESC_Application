#include "vesc_crc.h"

enum {
  CRC16_POLYNOMIAL = 0x1021U,
  CRC16_TOP_BIT = 0x8000U,
  BITS_PER_BYTE = 8U
};

uint16_t vesc_crc16(const uint8_t *data, uint32_t length)
{
  uint16_t crc = 0U;

  while (length-- > 0U)
  {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint32_t bit = 0U; bit < BITS_PER_BYTE; ++bit)
    {
      crc = (crc & CRC16_TOP_BIT)
          ? (uint16_t)((crc << 1) ^ CRC16_POLYNOMIAL)
          : (uint16_t)(crc << 1);
    }
  }

  return crc;
}
