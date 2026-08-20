#include "firmware_update.h"

#include "main.h"
#include "usb_device.h"
#include "usbd_core.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

enum {
  STAGING_ADDRESS = 0x08080000UL,
  STAGING_SIZE = 3UL * 128UL * 1024UL,
  STAGING_SECTOR_SIZE = 128UL * 1024UL,
  BOOTLOADER_ADDRESS = 0x080E0000UL,
  FLASH_END_ADDRESS = 0x08100000UL,
  SRAM_START_ADDRESS = 0x20000000UL,
  SRAM_END_ADDRESS = 0x20020000UL,
  USB_DISCONNECT_DELAY_MS = 100U,
  NVIC_REGISTER_COUNT = 8U
};

enum {
  BOOTLOADER_HEADER_SIZE = 6U
};

static uint32_t update_size;
static uint32_t update_next_offset;
static uint32_t update_expected_crc32;
static bool update_active;

static const uint32_t staging_sectors[] = {
  FLASH_SECTOR_8, FLASH_SECTOR_9, FLASH_SECTOR_10
};

/* This must not create a stack frame because it replaces the active MSP. */
__attribute__((naked, noreturn))
static void bootloader_branch(uint32_t initial_msp, uint32_t reset_handler)
{
  __asm volatile(
      "movs r2, #0\n"
      "msr control, r2\n"
      "isb\n"
      "msr msp, r0\n"
      "cpsie i\n"
      "bx r1\n");
}

static bool staging_range_is_valid(uint32_t offset, uint32_t length)
{
  return offset <= STAGING_SIZE && length <= (STAGING_SIZE - offset);
}

static bool bootloader_vectors_are_valid(uint32_t stack_pointer,
                                         uint32_t reset_handler)
{
  const uint32_t reset_address = reset_handler & ~1UL;

  return stack_pointer >= SRAM_START_ADDRESS &&
         stack_pointer <= SRAM_END_ADDRESS &&
         (reset_handler & 1UL) != 0U &&
         reset_address >= BOOTLOADER_ADDRESS &&
         reset_address < FLASH_END_ADDRESS;
}

static void set_update_error(void)
{
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
}

bool firmware_update_erase(uint32_t image_size)
{
  if (image_size == 0U || image_size > STAGING_SIZE)
  {
    return false;
  }

  const uint32_t sector_count =
      (image_size + STAGING_SECTOR_SIZE - 1U) / STAGING_SECTOR_SIZE;
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0U;

  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    set_update_error();
    return false;
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  erase.Sector = staging_sectors[0];
  erase.NbSectors = sector_count;

  const HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&erase, &sector_error);
  HAL_FLASH_Lock();
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  if (result != HAL_OK)
  {
    set_update_error();
    return false;
  }
  return true;
}

bool firmware_update_write(uint32_t offset, const uint8_t *data, uint32_t length)
{
  if (data == NULL || length == 0U || !staging_range_is_valid(offset, length))
  {
    return false;
  }

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    set_update_error();
    return false;
  }

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  bool succeeded = true;
  for (uint32_t index = 0U; index < length; ++index)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
                          STAGING_ADDRESS + offset + index,
                          data[index]) != HAL_OK)
    {
      succeeded = false;
      break;
    }
  }

  HAL_FLASH_Lock();
  if (!succeeded)
  {
    set_update_error();
  }
  return succeeded;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data,
                             uint32_t length)
{
  while (length-- > 0U)
  {
    crc ^= *data++;
    for (uint32_t bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? 0xEDB88320UL : 0U);
    }
  }
  return crc;
}

static uint16_t crc16_update(uint16_t crc, uint8_t value)
{
  crc ^= (uint16_t)value << 8U;
  for (uint32_t bit = 0U; bit < 8U; ++bit)
  {
    crc = (uint16_t)((crc << 1U) ^
                     ((crc & 0x8000U) != 0U ? 0x1021U : 0U));
  }
  return crc;
}

bool firmware_update_begin(uint32_t image_size, uint32_t image_crc32)
{
  if ((image_size == 0U) ||
      (image_size > (STAGING_SIZE - BOOTLOADER_HEADER_SIZE)))
  {
    return false;
  }
  if (!firmware_update_erase(image_size + BOOTLOADER_HEADER_SIZE))
  {
    return false;
  }
  update_size = image_size;
  update_next_offset = 0U;
  update_expected_crc32 = image_crc32;
  update_active = true;
  return true;
}

bool firmware_update_write_block(uint32_t offset, const uint8_t *data,
                                 uint32_t length)
{
  if (!update_active || (data == NULL) || (length == 0U) ||
      (offset > update_size) || (length > (update_size - offset)))
  {
    return false;
  }

  if (offset < update_next_offset)
  {
    if ((offset + length) > update_next_offset)
    {
      return false;
    }
    for (uint32_t index = 0U; index < length; ++index)
    {
      if (*(const uint8_t *)(STAGING_ADDRESS + BOOTLOADER_HEADER_SIZE +
                             offset + index) != data[index])
      {
        return false;
      }
    }
    return true;
  }
  if (offset != update_next_offset)
  {
    return false;
  }
  if (!firmware_update_write(BOOTLOADER_HEADER_SIZE + offset, data, length))
  {
    return false;
  }
  update_next_offset += length;
  return true;
}

bool firmware_update_finish(uint32_t *calculated_crc32)
{
  uint32_t crc32 = 0xFFFFFFFFUL;
  uint16_t crc16 = 0U;
  uint8_t header[BOOTLOADER_HEADER_SIZE];

  if (!update_active || (update_next_offset != update_size))
  {
    return false;
  }
  for (uint32_t index = 0U; index < update_size; ++index)
  {
    const uint8_t value = *(const uint8_t *)(STAGING_ADDRESS +
                                             BOOTLOADER_HEADER_SIZE + index);
    crc32 = crc32_update(crc32, &value, 1U);
    crc16 = crc16_update(crc16, value);
  }
  crc32 ^= 0xFFFFFFFFUL;
  if (calculated_crc32 != NULL)
  {
    *calculated_crc32 = crc32;
  }
  if (crc32 != update_expected_crc32)
  {
    return false;
  }

  header[0] = (uint8_t)(update_size >> 24U);
  header[1] = (uint8_t)(update_size >> 16U);
  header[2] = (uint8_t)(update_size >> 8U);
  header[3] = (uint8_t)update_size;
  header[4] = (uint8_t)(crc16 >> 8U);
  header[5] = (uint8_t)crc16;
  if (!firmware_update_write(0U, header, sizeof(header)))
  {
    return false;
  }
  update_active = false;
  return true;
}

void firmware_update_abort(void)
{
  if (update_size != 0U)
  {
    /* Erasing sector 8 invalidates a header written by a completed FINISH. */
    (void)firmware_update_erase(1U);
  }
  update_active = false;
  update_size = 0U;
  update_next_offset = 0U;
  update_expected_crc32 = 0U;
}

uint32_t firmware_update_size(void) { return update_size; }
uint32_t firmware_update_next_offset(void) { return update_next_offset; }
uint32_t firmware_update_expected_crc32(void) { return update_expected_crc32; }

void firmware_update_jump_to_bootloader(void)
{
  const uint32_t boot_msp = *(const volatile uint32_t *)BOOTLOADER_ADDRESS;
  const uint32_t boot_reset =
      *(const volatile uint32_t *)(BOOTLOADER_ADDRESS + sizeof(uint32_t));

  if (!bootloader_vectors_are_valid(boot_msp, boot_reset))
  {
    set_update_error();
    while (1) { }
  }

  (void)USBD_Stop(&hUsbDeviceFS);
  HAL_Delay(USB_DISCONNECT_DELAY_MS);
  (void)USBD_DeInit(&hUsbDeviceFS);
  HAL_RCC_DeInit();
  HAL_DeInit();

  __disable_irq();
  SysTick->CTRL = 0U;
  SysTick->LOAD = 0U;
  SysTick->VAL = 0U;

  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
  for (uint32_t index = 0U; index < NVIC_REGISTER_COUNT; ++index)
  {
    NVIC->ICER[index] = 0xFFFFFFFFUL;
    NVIC->ICPR[index] = 0xFFFFFFFFUL;
  }

  SCB->VTOR = BOOTLOADER_ADDRESS;
  __DSB();
  __ISB();
  bootloader_branch(boot_msp, boot_reset);
}
