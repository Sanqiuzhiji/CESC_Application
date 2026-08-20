#ifndef FIRMWARE_UPDATE_H
#define FIRMWARE_UPDATE_H

#include <stdbool.h>
#include <stdint.h>

bool firmware_update_erase(uint32_t image_size);
bool firmware_update_write(uint32_t offset, const uint8_t *data, uint32_t length);
bool firmware_update_begin(uint32_t image_size, uint32_t image_crc32);
bool firmware_update_write_block(uint32_t offset, const uint8_t *data,
                                 uint32_t length);
bool firmware_update_finish(uint32_t *calculated_crc32);
void firmware_update_abort(void);
uint32_t firmware_update_size(void);
uint32_t firmware_update_next_offset(void);
uint32_t firmware_update_expected_crc32(void);
void firmware_update_jump_to_bootloader(void) __attribute__((noreturn));

#endif /* FIRMWARE_UPDATE_H */
