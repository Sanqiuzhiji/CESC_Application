#ifndef USB_CDC_TRANSPORT_H
#define USB_CDC_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t mutex_timeouts;
    uint32_t submit_timeouts;
    uint32_t completion_timeouts;
    uint32_t nonblocking_drops;
} usb_cdc_transport_diagnostics_t;

/** Send one CDC transfer while serializing access between RTOS tasks. */
bool usb_cdc_transport_send(uint8_t *data, uint16_t length);
bool usb_cdc_transport_try_send(uint8_t *data, uint16_t length);
void usb_cdc_transport_get_diagnostics(
    usb_cdc_transport_diagnostics_t *diagnostics);

#endif /* USB_CDC_TRANSPORT_H */
