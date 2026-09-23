#include "motor_config_store.h"

#include <stddef.h>
#include <string.h>

#include "main.h"

enum {
  CONFIG_MAGIC = 0x43455343UL,
  CONFIG_COMMIT = 0x56414C49UL,
  CONFIG_VERSION = 1U,
  SLOT_A_ADDRESS = 0x08040000UL,
  SLOT_B_ADDRESS = 0x08060000UL
};

typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t payload_size;
  uint32_t sequence;
  motor_user_config_t config;
  uint32_t crc32;
  uint32_t commit;
} config_record_t;

static motor_config_store_status_t store_status;

static uint32_t enter_critical(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void leave_critical(uint32_t primask)
{
  __DMB();
  if (primask == 0U) __enable_irq();
}

static uint32_t crc32(const void *data, uint32_t length)
{
  const uint8_t *bytes = data;
  uint32_t crc = 0xFFFFFFFFUL;
  while (length-- > 0U) {
    crc ^= *bytes++;
    for (uint32_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

static bool record_valid(const config_record_t *record)
{
  return record->magic == CONFIG_MAGIC &&
      record->version == CONFIG_VERSION &&
      record->payload_size == sizeof(motor_user_config_t) &&
      record->commit == CONFIG_COMMIT &&
      record->crc32 == crc32(record, offsetof(config_record_t, crc32)) &&
      motor_control_config_validate_user(&record->config);
}

static bool sequence_newer(uint32_t left, uint32_t right)
{
  return (int32_t)(left - right) > 0;
}

static const config_record_t *selected_record(void)
{
  const config_record_t *a = (const config_record_t *)SLOT_A_ADDRESS;
  const config_record_t *b = (const config_record_t *)SLOT_B_ADDRESS;
  const bool a_valid = record_valid(a);
  const bool b_valid = record_valid(b);
  store_status.slot_a_valid = a_valid;
  store_status.slot_b_valid = b_valid;
  if (!a_valid) return b_valid ? b : NULL;
  if (!b_valid) return a;
  return sequence_newer(b->sequence, a->sequence) ? b : a;
}

void motor_config_store_init(void)
{
  memset(&store_status, 0, sizeof(store_status));
  motor_control_config_restore_defaults();
  (void)motor_config_store_reload();
}

bool motor_config_store_stage(const motor_user_config_t *config)
{
  if (!motor_control_config_apply_user(config)) return false;
  const uint32_t primask = enter_critical();
  store_status.dirty = true;
  leave_critical(primask);
  return true;
}

bool motor_config_store_reload(void)
{
  const config_record_t *record = selected_record();
  if (record == NULL) {
    motor_control_config_restore_defaults();
    const uint32_t primask = enter_critical();
    store_status.source = MOTOR_CONFIG_SOURCE_DEFAULT;
    store_status.sequence = 0U;
    store_status.dirty = false;
    leave_critical(primask);
    return false;
  }
  motor_control_config_restore_defaults();
  if (!motor_control_config_apply_user(&record->config)) return false;
  const uint32_t primask = enter_critical();
  store_status.source =
      (record == (const config_record_t *)SLOT_A_ADDRESS) ?
      MOTOR_CONFIG_SOURCE_SLOT_A : MOTOR_CONFIG_SOURCE_SLOT_B;
  store_status.sequence = record->sequence;
  store_status.dirty = false;
  leave_critical(primask);
  return true;
}

void motor_config_store_restore_defaults(void)
{
  motor_control_config_restore_defaults();
  const uint32_t primask = enter_critical();
  store_status.source = MOTOR_CONFIG_SOURCE_DEFAULT;
  store_status.dirty = true;
  leave_critical(primask);
}

bool motor_config_store_save(void)
{
  motor_user_config_t user;
  config_record_t record;
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0U;
  const config_record_t *current = selected_record();
  const uint32_t address =
      current == (const config_record_t *)SLOT_A_ADDRESS ?
      SLOT_B_ADDRESS : SLOT_A_ADDRESS;
  const uint32_t sector =
      address == SLOT_A_ADDRESS ? FLASH_SECTOR_6 : FLASH_SECTOR_7;

  motor_control_config_get_user(&user);
  memset(&record, 0xFF, sizeof(record));
  record.magic = CONFIG_MAGIC;
  record.version = CONFIG_VERSION;
  record.payload_size = sizeof(motor_user_config_t);
  record.sequence = current != NULL ? current->sequence + 1U : 1U;
  record.config = user;
  record.crc32 = crc32(&record, offsetof(config_record_t, crc32));
  record.commit = CONFIG_COMMIT;

  if (HAL_FLASH_Unlock() != HAL_OK) return false;
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = sector;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  bool ok = HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK;
  const uint32_t *words = (const uint32_t *)&record;
  const uint32_t word_count = sizeof(record) / sizeof(uint32_t);
  for (uint32_t index = 0U; ok && index + 1U < word_count; ++index) {
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                           address + index * sizeof(uint32_t),
                           words[index]) == HAL_OK;
  }
  if (ok) {
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                           address + (word_count - 1U) * sizeof(uint32_t),
                           words[word_count - 1U]) == HAL_OK;
  }
  HAL_FLASH_Lock();
  if (!ok || !record_valid((const config_record_t *)address)) return false;
  return motor_config_store_reload();
}

bool motor_config_store_request_save(void)
{
  const uint32_t primask = enter_critical();
  if (store_status.save_pending) {
    leave_critical(primask);
    return false;
  }
  /* Clear the old result before publishing pending. The host must not
   * mistake a previous successful erase for this request. */
  store_status.last_save_ok = false;
  store_status.save_pending = true;
  leave_critical(primask);
  return true;
}

void motor_config_store_process(void)
{
  if (!store_status.save_pending) return;
  const bool ok = motor_config_store_save();
  /* Publish completion last so a concurrent protocol status read cannot
   * observe "not pending" before dirty/source/sequence and result are final. */
  const uint32_t primask = enter_critical();
  store_status.last_save_ok = ok;
  store_status.save_pending = false;
  leave_critical(primask);
}

void motor_config_store_get_status(motor_config_store_status_t *status)
{
  if (status != NULL) {
    const uint32_t primask = enter_critical();
    *status = store_status;
    leave_critical(primask);
  }
}
