/*
 * Motor power stage abstraction.
 *
 * The state machine only ever asks for OFF, OPEN or CLOSE. Every driver goes
 * through motor_interlock_*(), a second, independent guard which refuses to
 * energise a direction while the other one is on or before the dead time has
 * elapsed. A refused request leaves both outputs OFF and raises output_fault.
 *
 * Implemented drivers:
 *  - "gpio2": two logic outputs, one per direction (see README "Power stage").
 *  - "sim"  : no GPIO at all, drives the simulated plant.
 *
 * Extension point: a "dir+enable" driver (one run output + one direction
 * relay) or a PWM H-bridge driver can be added by implementing motor_driver_t.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "cover_sm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    cover_dir_t current;
    uint32_t t_off;
    uint32_t deadtime_ms;
} motor_interlock_t;

void motor_interlock_init(motor_interlock_t *il, uint32_t deadtime_ms, uint32_t now_ms);
/** Returns ESP_OK if switching to `dir` is allowed now. Does not change state. */
esp_err_t motor_interlock_check(const motor_interlock_t *il, cover_dir_t dir, uint32_t now_ms);
void motor_interlock_commit(motor_interlock_t *il, cover_dir_t dir, uint32_t now_ms);

typedef struct motor_driver motor_driver_t;

struct motor_driver {
    const char *name;
    /** Switch outputs. Must never energise both directions. */
    esp_err_t (*set)(motor_driver_t *self, cover_dir_t dir, uint32_t now_ms);
    /** True if the driver detected an output problem (latched until reboot). */
    bool (*output_fault)(motor_driver_t *self);
    void *ctx;
};

typedef struct {
    int gpio_open;           /* output enabling the OPEN direction  */
    int gpio_close;          /* output enabling the CLOSE direction */
    bool active_high;        /* true: logic 1 energises             */
    uint32_t deadtime_ms;
} motor_gpio2_config_t;

/**
 * Configure two direction outputs. The inactive level is written BEFORE the
 * pins are switched to output mode so that no glitch reaches the power stage.
 */
esp_err_t motor_driver_gpio2_create(const motor_gpio2_config_t *cfg, uint32_t now_ms, motor_driver_t **out);

/** Force both outputs of a gpio2 driver inactive (used from panic/shutdown paths). */
void motor_driver_gpio2_all_off(const motor_gpio2_config_t *cfg);

struct sim_plant;
esp_err_t motor_driver_sim_create(struct sim_plant *plant, uint32_t deadtime_ms, uint32_t now_ms,
                                  motor_driver_t **out);

#ifdef __cplusplus
}
#endif
