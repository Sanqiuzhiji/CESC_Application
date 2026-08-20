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
