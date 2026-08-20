#include "usb_cdc_transport.h"

#include "cmsis_os2.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

enum {
    USB_TX_READY_TIMEOUT_MS = 100U,
    USB_TX_COMPLETE_TIMEOUT_MS = 100U
};

extern osMutexId_t usbTxMutexHandle;
extern USBD_HandleTypeDef hUsbDeviceFS;

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

    if ((data == NULL) || (length == 0U) ||
        (osMutexAcquire(usbTxMutexHandle, USB_TX_READY_TIMEOUT_MS) != osOK))
    {
        return false;
    }

    if (wait_for_tx_idle(USB_TX_READY_TIMEOUT_MS) &&
        (CDC_Transmit_FS(data, length) == USBD_OK))
    {
        succeeded = wait_for_tx_idle(USB_TX_COMPLETE_TIMEOUT_MS);
    }

    (void)osMutexRelease(usbTxMutexHandle);
    return succeeded;
}
