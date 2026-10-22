#include "motor_driver.h"

void motor_interlock_init(motor_interlock_t *il, uint32_t deadtime_ms, uint32_t now_ms)
{
    il->current = COVER_DIR_NONE;
    il->t_off = now_ms; /* boot = "just switched off" */
    il->deadtime_ms = deadtime_ms;
}

esp_err_t motor_interlock_check(const motor_interlock_t *il, cover_dir_t dir, uint32_t now_ms)
{
    if (dir == COVER_DIR_NONE || dir == il->current) {
        return ESP_OK;
    }
    if (il->current != COVER_DIR_NONE) {
        return ESP_ERR_INVALID_STATE; /* direct reversal requested */
    }
    if ((uint32_t)(now_ms - il->t_off) < il->deadtime_ms) {
        return ESP_ERR_INVALID_STATE; /* dead time not elapsed */
    }
    return ESP_OK;
}

void motor_interlock_commit(motor_interlock_t *il, cover_dir_t dir, uint32_t now_ms)
{
    if (dir == COVER_DIR_NONE && il->current != COVER_DIR_NONE) {
        il->t_off = now_ms;
    }
    il->current = dir;
}
