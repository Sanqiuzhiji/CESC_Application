#include "usb_cdc_transport.h"

#include "cmsis_os2.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

enum {
    USB_TX_MUTEX_TIMEOUT_MS = 100U,
    USB_TX_SUBMIT_TIMEOUT_MS = 250U,
    USB_TX_COMPLETE_TIMEOUT_MS = 100U
};

extern osMutexId_t usbTxMutexHandle;
extern USBD_HandleTypeDef hUsbDeviceFS;

static volatile usb_cdc_transport_diagnostics_t transport_diagnostics;

static bool wait_for_tx_idle(uint32_t timeout_ms)
{
    const uint32_t started_at = osKernelGetTickCount();

    for (;;)
    {
        const USBD_CDC_HandleTypeDef *cdc =
            (const USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

        if ((cdc != NULL) && (cdc->TxState == 0U))
        {
            return true;
        }
        if ((osKernelGetTickCount() - started_at) >= timeout_ms)
        {
            return false;
        }
        (void)osDelay(1U);
    }
}

bool usb_cdc_transport_send(uint8_t *data, uint16_t length)
{
    bool succeeded = false;
    bool submitted = false;
    const uint32_t started_at = osKernelGetTickCount();

    if ((data == NULL) || (length == 0U))
    {
        return false;
    }
    if (osMutexAcquire(usbTxMutexHandle, USB_TX_MUTEX_TIMEOUT_MS) != osOK)
    {
        ++transport_diagnostics.mutex_timeouts;
        return false;
    }

    /* TxState can change between the idle check and CDC_Transmit_FS().
       Treat USBD_BUSY as transient and keep trying until the submit deadline. */
    while ((osKernelGetTickCount() - started_at) < USB_TX_SUBMIT_TIMEOUT_MS)
    {
        const uint32_t elapsed = osKernelGetTickCount() - started_at;
        const uint32_t remaining = USB_TX_SUBMIT_TIMEOUT_MS - elapsed;

        if (wait_for_tx_idle(remaining) &&
            (CDC_Transmit_FS(data, length) == USBD_OK))
        {
            submitted = true;
            succeeded = wait_for_tx_idle(USB_TX_COMPLETE_TIMEOUT_MS);
            break;
        }
        (void)osDelay(1U);
    }

    if (!submitted) ++transport_diagnostics.submit_timeouts;
    else if (!succeeded) ++transport_diagnostics.completion_timeouts;

    (void)osMutexRelease(usbTxMutexHandle);
    return succeeded;
}

bool usb_cdc_transport_try_send(uint8_t *data, uint16_t length)
{
    bool submitted = false;

    if ((data == NULL) || (length == 0U) ||
        (osMutexAcquire(usbTxMutexHandle, 0U) != osOK))
    {
        ++transport_diagnostics.nonblocking_drops;
        return false;
    }

    const USBD_CDC_HandleTypeDef *cdc =
        (const USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if ((cdc != NULL) && (cdc->TxState == 0U))
    {
        submitted = CDC_Transmit_FS(data, length) == USBD_OK;
    }

    (void)osMutexRelease(usbTxMutexHandle);
    if (!submitted) ++transport_diagnostics.nonblocking_drops;
    return submitted;
}

void usb_cdc_transport_get_diagnostics(
    usb_cdc_transport_diagnostics_t *diagnostics)
{
    if (diagnostics != NULL) *diagnostics = transport_diagnostics;
}
