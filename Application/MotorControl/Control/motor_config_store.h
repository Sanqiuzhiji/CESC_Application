#ifndef MOTOR_CONFIG_STORE_H
#define MOTOR_CONFIG_STORE_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_control_config.h"

typedef enum {
  MOTOR_CONFIG_SOURCE_DEFAULT = 0,
  MOTOR_CONFIG_SOURCE_SLOT_A = 1,
  MOTOR_CONFIG_SOURCE_SLOT_B = 2
} motor_config_source_t;

typedef struct {
  motor_config_source_t source;
  uint32_t sequence;
  bool slot_a_valid;
  bool slot_b_valid;
  bool dirty;
  bool save_pending;
  bool last_save_ok;
} motor_config_store_status_t;

void motor_config_store_init(void);
bool motor_config_store_stage(const motor_user_config_t *config);
bool motor_config_store_save(void);
bool motor_config_store_request_save(void);
void motor_config_store_process(void);
bool motor_config_store_reload(void);
void motor_config_store_restore_defaults(void);
void motor_config_store_get_status(motor_config_store_status_t *status);

#endif
