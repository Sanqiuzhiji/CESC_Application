#include "angle_sensor.h"

#include <stddef.h>

#include "as5600.h"
#include "i2c.h"
#include "main.h"

enum
{
    ANGLE_SENSOR_POLL_PERIOD_MS = 2U,
    ANGLE_SENSOR_DIAGNOSTIC_PERIOD_MS = 100U,
    ANGLE_SENSOR_I2C_TIMEOUT_MS = 10U
};

static as5600_t sensor;
static angle_sensor_sample_t latest_sample;
static uint32_t last_poll_ms;
static uint32_t last_diagnostic_ms;
static bool sample_valid;

static void recover_i2c_bus(I2C_HandleTypeDef* i2c)
{
    GPIO_InitTypeDef gpio = {0};

    (void)HAL_I2C_DeInit(i2c);
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(1U);
    for (uint32_t pulse = 0U; pulse < 9U; ++pulse)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        HAL_Delay(1U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        HAL_Delay(1U);
    }

    /* Generate STOP: SDA low-to-high while SCL is high. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_Delay(1U);

    __HAL_RCC_I2C2_FORCE_RESET();
    __HAL_RCC_I2C2_RELEASE_RESET();
    MX_I2C2_Init();
}

static bool i2c_ready(void* context, uint8_t address)
{
    I2C_HandleTypeDef* i2c = (I2C_HandleTypeDef*)context;
    const HAL_StatusTypeDef result =
        HAL_I2C_IsDeviceReady(i2c, (uint16_t)address << 1, 2U,
                              ANGLE_SENSOR_I2C_TIMEOUT_MS);
    if (result != HAL_OK)
    {
        recover_i2c_bus(i2c);
    }
    return result == HAL_OK;
}

static bool i2c_read(void* context, uint8_t address, uint8_t reg,
                     uint8_t* data, uint16_t length)
{
    I2C_HandleTypeDef* i2c = (I2C_HandleTypeDef*)context;
    const HAL_StatusTypeDef result =
        HAL_I2C_Mem_Read(i2c, (uint16_t)address << 1, reg,
                         I2C_MEMADD_SIZE_8BIT, data, length,
                         ANGLE_SENSOR_I2C_TIMEOUT_MS);

    if (result != HAL_OK)
    {
        recover_i2c_bus(i2c);
    }
    return result == HAL_OK;
}

static angle_sensor_status_t map_magnet_status(as5600_magnet_status_t status)
{
    switch (status)
    {
    case AS5600_MAGNET_OK: return ANGLE_SENSOR_STATUS_OK;
    case AS5600_MAGNET_NOT_DETECTED: return ANGLE_SENSOR_STATUS_NO_MAGNET;
    case AS5600_MAGNET_TOO_WEAK: return ANGLE_SENSOR_STATUS_MAGNET_WEAK;
    case AS5600_MAGNET_TOO_STRONG: return ANGLE_SENSOR_STATUS_MAGNET_STRONG;
    default: return ANGLE_SENSOR_STATUS_IO_ERROR;
    }
}

bool angle_sensor_init(void)
{
    const as5600_bus_t bus = {
        .context = &hi2c2,
        .ready = i2c_ready,
        .read = i2c_read
    };
    const uint32_t now = HAL_GetTick();

    latest_sample.status = ANGLE_SENSOR_STATUS_UNINITIALIZED;
    sample_valid = false;
    last_poll_ms = now;
    last_diagnostic_ms = now;

    if (!as5600_init(&sensor, &bus))
    {
        latest_sample.status = ANGLE_SENSOR_STATUS_NOT_FOUND;
        return false;
    }

    latest_sample.status = map_magnet_status(as5600_read_magnet_status(&sensor));
    return latest_sample.status != ANGLE_SENSOR_STATUS_IO_ERROR;
}

void angle_sensor_process(void)
{
    const uint32_t now = HAL_GetTick();
    uint16_t raw;

    if ((uint32_t)(now - last_poll_ms) < ANGLE_SENSOR_POLL_PERIOD_MS)
    {
        return;
    }
    last_poll_ms = now;

    if (!as5600_read_raw(&sensor, &raw))
    {
        latest_sample.status = ANGLE_SENSOR_STATUS_IO_ERROR;
        return;
    }

    latest_sample.raw = raw;
    latest_sample.degrees = (float)raw * (360.0F / 4096.0F);
    latest_sample.timestamp_ms = now;
    sample_valid = true;

    if ((uint32_t)(now - last_diagnostic_ms) >=
        ANGLE_SENSOR_DIAGNOSTIC_PERIOD_MS)
    {
        last_diagnostic_ms = now;
        latest_sample.status =
            map_magnet_status(as5600_read_magnet_status(&sensor));
    }
}

bool angle_sensor_get_sample(angle_sensor_sample_t* sample)
{
    if ((sample == NULL) || !sample_valid)
    {
        return false;
    }
    *sample = latest_sample;
    return true;
}

bool angle_sensor_read_degrees(float* degrees)
{
    if ((degrees == NULL) || !sample_valid)
    {
        return false;
    }
    *degrees = latest_sample.degrees;
    return true;
}

bool angle_sensor_read_raw(uint16_t* raw)
{
    if ((raw == NULL) || !sample_valid)
    {
        return false;
    }
    *raw = latest_sample.raw;
    return true;
}

angle_sensor_status_t angle_sensor_get_status(void)
{
    return latest_sample.status;
}
