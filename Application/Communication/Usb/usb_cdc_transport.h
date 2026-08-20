#ifndef USB_CDC_TRANSPORT_H
#define USB_CDC_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

/** Send one CDC transfer while serializing access between RTOS tasks. */
bool usb_cdc_transport_send(uint8_t *data, uint16_t length);

#endif /* USB_CDC_TRANSPORT_H */
