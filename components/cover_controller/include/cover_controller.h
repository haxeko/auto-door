/*
 * Cover controller task: owns the state machine, the motor driver, the inputs
 * and the storage requests. Everything else talks to it through a queue.
 *
 * Works completely without network: local buttons are handled inside this
 * task, Matter is only a consumer of status callbacks and a producer of
 * commands.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cover_inputs.h"
#include "cover_sm.h"
#include "esp_err.h"
#include "motor_driver.h"
#include "sim_plant.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    cover_sm_config_t sm;
    uint32_t period_ms;
    uint32_t btn_debounce_ms;
    uint32_t sensor_debounce_ms;
    bool restore_position;
    cover_calib_t default_calib;     /* used if nothing stored; 0/0 = none */

    bool simulation;                 /* true: never touches motor GPIOs */
    motor_gpio2_config_t gpio2;      /* used when !simulation */
    cover_inputs_config_t inputs;    /* limit pins ignored in simulation */
    uint32_t sim_open_ms;
    uint32_t sim_close_ms;
    bool sim_limit_open;
    bool sim_limit_closed;

    uint32_t func_commission_hold_ms;
    uint32_t func_reset_hold_ms;
    uint32_t cal_combo_hold_ms;
} cover_controller_config_t;

typedef enum {
    COVER_FUNC_OPEN_COMMISSIONING,   /* function button released after commission hold */
    COVER_FUNC_RESET_ARMED,          /* reset hold time reached, waiting for release   */
    COVER_FUNC_FACTORY_RESET,        /* released after reset hold. Motor already off.  */
} cover_func_event_t;

/** Called from the control task. Must not block (copy + schedule only). */
typedef void (*cover_status_cb_t)(const cover_status_t *status, void *arg);
/** Called from the control task. Must not block. */
typedef void (*cover_func_cb_t)(cover_func_event_t ev, void *arg);

esp_err_t cover_controller_start(const cover_controller_config_t *cfg, cover_status_cb_t status_cb,
                                 cover_func_cb_t func_cb, void *cb_arg);

/** Non-blocking. STOP is queued in front of everything else. */
esp_err_t cover_controller_post(const cover_cmd_t *cmd);

/** Install manual travel times (NULL clears calibration). Non-blocking. */
esp_err_t cover_controller_set_calibration(const cover_calib_t *calib);

/** Force the motor off and keep it off (used before restart / factory reset). Blocks up to timeout. */
esp_err_t cover_controller_halt(uint32_t timeout_ms);

/** Latest snapshot (thread-safe copy). */
void cover_controller_get_status(cover_status_t *out);

/** Simulated plant, or NULL when driving real hardware. */
sim_plant_t *cover_controller_sim(void);

const char *cover_src_str(cover_src_t s);

#ifdef __cplusplus
}
#endif
