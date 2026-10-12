#include "application.h"

#include "cmsis_os2.h"
#include "angle_sensor.h"
#include "motor_config_store.h"

extern osSemaphoreId_t usbRxSemaphoreHandle;

void StartProtocolTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        if (osSemaphoreAcquire(usbRxSemaphoreHandle, osWaitForever) == osOK)
        {
            application_protocol_process();
        }
    }
}

void StartSensorTask(void *argument)
{
    uint32_t next_wake;

    (void)argument;
    (void)angle_sensor_init();
    uint16_t electrical_zero_raw;
    if (motor_config_store_get_electrical_zero(&electrical_zero_raw))
    {
        (void)angle_sensor_restore_electrical_zero(electrical_zero_raw);
    }
    next_wake = osKernelGetTickCount();

    for (;;)
    {
        angle_sensor_process();
        next_wake += 2U;
        (void)osDelayUntil(next_wake);
    }
}

void StartStatusTask(void *argument)
{
    uint32_t next_wake;

    (void)argument;
    next_wake = osKernelGetTickCount();

    for (;;)
    {
        application_status_process();
        next_wake += 10U;
        (void)osDelayUntil(next_wake);
    }
}
