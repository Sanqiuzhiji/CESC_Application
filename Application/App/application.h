#ifndef APPLICATION_H
#define APPLICATION_H

#include <stdint.h>

/** Initialize all user-owned application modules. */
void application_init(void);

/** Run one non-blocking iteration of the application. */
void application_process(void);

/** Queue bytes received by the USB CDC interrupt callback. */
void application_usb_receive(const uint8_t *data, uint32_t length);

#endif /* APPLICATION_H */
