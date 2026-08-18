#ifndef FIRMWARE_UPDATE_H
#define FIRMWARE_UPDATE_H

#include <stdbool.h>
#include <stdint.h>

bool firmware_update_erase(uint32_t image_size);
bool firmware_update_write(uint32_t offset, const uint8_t *data, uint32_t length);
void firmware_update_jump_to_bootloader(void) __attribute__((noreturn));

#endif /* FIRMWARE_UPDATE_H */
