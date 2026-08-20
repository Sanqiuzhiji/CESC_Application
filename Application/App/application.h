#ifndef APPLICATION_H
#define APPLICATION_H

#include <stdint.h>

/** Initialize all user-owned application modules. */
void application_init(void);

/** Run one non-blocking iteration of the application. */
void application_process(void);

/** Process queued protocol data from the protocol RTOS task. */
void application_protocol_process(void);

/** Process low-priority indicators and diagnostic output. */
void application_status_process(void);

/** Queue bytes received by the USB CDC interrupt callback. */
void application_usb_receive(const uint8_t *data, uint32_t length);

#endif /* APPLICATION_H */
