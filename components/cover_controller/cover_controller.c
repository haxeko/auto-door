#include "cover_controller.h"

#include <string.h>

#include "cover_storage.h"
#include "debounce.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "cover_ctl";

#define QUEUE_LEN      16
#define TASK_STACK     4096
#define TASK_PRIO      10 /* above the CHIP task, below Wi-Fi / esp_timer / IDF system tasks */

typedef enum {
    MSG_CMD,
    MSG_SET_CALIB,
    MSG_CLEAR_CALIB,
    MSG_HALT,
} msg_type_t;

typedef struct {
    msg_type_t type;
    union {
        cover_cmd_t cmd;
        cover_calib_t calib;
    };
} msg_t;

typedef struct {
    debounce_t db;
    bool pressed;
    uint32_t t_press;
} button_t;

static struct {
    cover_controller_config_t cfg;
    cover_status_cb_t status_cb;
    cover_func_cb_t func_cb;
    void *cb_arg;

    QueueHandle_t q;
    SemaphoreHandle_t halt_done;
    TaskHandle_t task;

    cover_sm_t sm;
    motor_driver_t *drv;
    cover_dir_t applied;
    bool halted;

    sim_plant_t sim;
    bool sim_active;

    debounce_t db_in[COVER_IN_COUNT];
    button_t btn_open, btn_close, btn_stop, btn_func;
    bool combo_fired;
    bool reset_armed;

    portMUX_TYPE lock;
    cover_status_t snapshot;
} s = {.lock = portMUX_INITIALIZER_UNLOCKED};

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

const char *cover_src_str(cover_src_t src)
{
    switch (src) {
    case COVER_SRC_BUTTON:  return "button";
    case COVER_SRC_MATTER:  return "matter";
    case COVER_SRC_CONSOLE: return "console";
    default:                return "internal";
    }
}

static const char *cmd_str(const cover_cmd_t *c)
{
    switch (c->type) {
    case COVER_CMD_STOP:         return "STOP";
    case COVER_CMD_OPEN:         return "OPEN";
    case COVER_CMD_CLOSE:        return "CLOSE";
    case COVER_CMD_GOTO:         return "GOTO";
    case COVER_CMD_BUTTON_OPEN:  return "BTN_OPEN";
    case COVER_CMD_BUTTON_CLOSE: return "BTN_CLOSE";
    case COVER_CMD_CALIBRATE:    return "CALIBRATE";
    }
    return "?";
}

/* ------------------------------------------------------------------ inputs */

static bool raw_input(cover_input_id_t id)
{
    if (s.sim_active) {
        switch (id) {
        case COVER_IN_LIMIT_OPEN:   return sim_plant_limit_open(&s.sim);
        case COVER_IN_LIMIT_CLOSED: return sim_plant_limit_closed(&s.sim);
        case COVER_IN_OBSTACLE:     return s.sim.obstacle || cover_inputs_read(id);
        case COVER_IN_DRIVER_FAULT: return s.sim.driver_fault || cover_inputs_read(id);
        default:                    return cover_inputs_read(id);
        }
    }
    return cover_inputs_read(id);
}

static void sample_inputs(uint32_t now, bool init)
{
    for (int i = 0; i < COVER_IN_COUNT; i++) {
        bool is_button = i <= COVER_IN_BTN_FUNC;
        uint32_t db_ms = is_button ? s.cfg.btn_debounce_ms : s.cfg.sensor_debounce_ms;
        bool raw = raw_input((cover_input_id_t)i);
        if (init) {
            debounce_init(&s.db_in[i], raw, db_ms, now);
        } else {
            debounce_update(&s.db_in[i], raw, now);
        }
    }
}

static cover_sm_inputs_t sm_inputs(void)
{
    cover_sm_inputs_t in = {
        .limit_open = s.db_in[COVER_IN_LIMIT_OPEN].stable,
        .limit_closed = s.db_in[COVER_IN_LIMIT_CLOSED].stable,
        .obstacle = s.db_in[COVER_IN_OBSTACLE].stable,
        .driver_fault = s.db_in[COVER_IN_DRIVER_FAULT].stable,
        .stop_held = s.db_in[COVER_IN_BTN_STOP].stable,
        .output_fault = s.drv && s.drv->output_fault(s.drv),
    };
    return in;
}

/* ------------------------------------------------------------------- motor */

static void apply_motor(uint32_t now)
{
    cover_dir_t want = s.halted ? COVER_DIR_NONE : cover_sm_motor(&s.sm);
    if (want == s.applied) {
        return;
    }
    esp_err_t err = s.drv->set(s.drv, want, now);
    if (err == ESP_OK) {
        s.applied = want;
        return;
    }
    ESP_LOGE(TAG, "motor driver refused %d: %s - forcing OFF", want, esp_err_to_name(err));
    s.drv->set(s.drv, COVER_DIR_NONE, now);
    s.applied = COVER_DIR_NONE;
    cover_sm_inputs_t in = sm_inputs();
    in.output_fault = true;
    cover_sm_tick(&s.sm, &in, now); /* state machine enters FAULT(OUTPUT) */
}

/* --------------------------------------------------------- events / status */

static void handle_events(void)
{
    uint32_t ev = cover_sm_take_events(&s.sm);
    if (!ev) {
        return;
    }
    cover_status_t st;
    cover_sm_get_status(&s.sm, &st);

    if (ev & COVER_EVT_STATE) {
        if (st.state == COVER_ST_FAULT) {
            ESP_LOGW(TAG, "state FAULT: %s (send STOP to acknowledge once the cause is gone)",
                     cover_fault_str(st.fault));
        } else {
            ESP_LOGI(TAG, "state %s%s", cover_state_str(st.state),
                     st.cal_phase != COVER_CAL_IDLE ? " (calibrating)" : "");
        }
    }
    if (ev & COVER_EVT_CAL_FAILED) {
        ESP_LOGW(TAG, "calibration aborted / implausible, previous calibration kept");
    }
    if (ev & COVER_EVT_CALIB) {
        if (st.calibrated) {
            ESP_LOGI(TAG, "calibration: open %u ms, close %u ms", (unsigned)st.calib.open_ms,
                     (unsigned)st.calib.close_ms);
            cover_storage_save_calib(&st.calib);
        } else {
            ESP_LOGI(TAG, "calibration cleared");
            cover_storage_save_calib(NULL);
        }
    }
    if (ev & COVER_EVT_RUN_START) {
        cover_pos_record_t rec = {.moving = true, .valid = false, .pos = 0};
        cover_storage_save_pos(&rec);
    }
    bool motor_off = cover_sm_motor(&s.sm) == COVER_DIR_NONE;
    if (motor_off && (ev & (COVER_EVT_RUN_STOP | COVER_EVT_POSITION))) {
        cover_pos_record_t rec = {
            .moving = false,
            .valid = st.pos_quality != COVER_POS_UNKNOWN,
            .pos = st.pos,
        };
        cover_storage_save_pos(&rec);
        if (ev & COVER_EVT_RUN_STOP) {
            if (rec.valid) {
                ESP_LOGI(TAG, "stopped at %u.%02u %% closed (%s)", st.pos / 100, st.pos % 100,
                         cover_pos_quality_str(st.pos_quality));
            } else {
                ESP_LOGI(TAG, "stopped, position unknown");
            }
        }
    }

    taskENTER_CRITICAL(&s.lock);
    s.snapshot = st;
    taskEXIT_CRITICAL(&s.lock);

    if (s.status_cb && (ev & (COVER_EVT_STATE | COVER_EVT_POSITION | COVER_EVT_TARGET | COVER_EVT_CALIB))) {
        s.status_cb(&st, s.cb_arg);
    }
}

static void run_cmd(const cover_cmd_t *cmd, uint32_t now)
{
    if (s.halted) {
        ESP_LOGW(TAG, "%s from %s ignored: controller halted", cmd_str(cmd), cover_src_str(cmd->source));
        return;
    }
    cover_result_t r = cover_sm_command(&s.sm, cmd, now);
    if (cmd->type == COVER_CMD_GOTO) {
        ESP_LOGI(TAG, "%s %u.%02u %% from %s -> %s", cmd_str(cmd), cmd->target / 100, cmd->target % 100,
                 cover_src_str(cmd->source), cover_result_str(r));
    } else {
        ESP_LOGI(TAG, "%s from %s -> %s", cmd_str(cmd), cover_src_str(cmd->source), cover_result_str(r));
    }
    if (r != COVER_OK && r != COVER_OK_NOOP) {
        /* Make sure Matter re-reads target/state after a rejected command. */
        s.sm.events |= COVER_EVT_STATE | COVER_EVT_TARGET;
    }
    if (r == COVER_OK_NOOP) {
        s.sm.events |= COVER_EVT_TARGET;
    }
}

/* ----------------------------------------------------------------- buttons */

static void local_cmd(cover_cmd_type_t type, uint32_t now)
{
    cover_cmd_t c = {.type = type, .source = COVER_SRC_BUTTON, .target = 0};
    run_cmd(&c, now);
}

static bool edge(button_t *b, cover_input_id_t id, uint32_t now, bool *released, uint32_t *held_ms)
{
    bool st = s.db_in[id].stable;
    bool pressed_edge = st && !b->pressed;
    *released = !st && b->pressed;
    if (pressed_edge) {
        b->t_press = now;
    }
    *held_ms = (b->pressed || st) ? (uint32_t)(now - b->t_press) : 0;
    b->pressed = st;
    return pressed_edge;
}

static void process_buttons(uint32_t now)
{
    bool rel;
    uint32_t held;

    if (edge(&s.btn_stop, COVER_IN_BTN_STOP, now, &rel, &held)) {
        local_cmd(COVER_CMD_STOP, now);
    }
    bool open_press = edge(&s.btn_open, COVER_IN_BTN_OPEN, now, &rel, &held);
    bool close_press = edge(&s.btn_close, COVER_IN_BTN_CLOSE, now, &rel, &held);

    bool both = s.btn_open.pressed && s.btn_close.pressed;
    if (both) {
        /* hold time counts from the later of the two presses */
        uint32_t later = (int32_t)(s.btn_open.t_press - s.btn_close.t_press) > 0 ? s.btn_open.t_press
                                                                               : s.btn_close.t_press;
        uint32_t since = (uint32_t)(now - later);
        if (!s.combo_fired && since >= s.cfg.cal_combo_hold_ms) {
            s.combo_fired = true;
            ESP_LOGI(TAG, "OPEN+CLOSE held: starting calibration");
            local_cmd(COVER_CMD_CALIBRATE, now);
        }
    } else if (!s.btn_open.pressed && !s.btn_close.pressed) {
        s.combo_fired = false;
    }
    /* Second button of a combo only stops, it never starts the other direction. */
    if (open_press && !(both && close_press)) {
        local_cmd(COVER_CMD_BUTTON_OPEN, now);
    }
    if (close_press && !(both && open_press)) {
        local_cmd(COVER_CMD_BUTTON_CLOSE, now);
    }

    /* Function button: hold >= commission time -> commissioning window,
     * hold >= reset time -> factory reset (on release). Distinct thresholds. */
    edge(&s.btn_func, COVER_IN_BTN_FUNC, now, &rel, &held);
    if (s.btn_func.pressed && !s.reset_armed && held >= s.cfg.func_reset_hold_ms) {
        s.reset_armed = true;
        ESP_LOGW(TAG, "function button: release now to FACTORY RESET");
        if (s.func_cb) {
            s.func_cb(COVER_FUNC_RESET_ARMED, s.cb_arg);
        }
    }
    if (rel) {
        uint32_t dur = (uint32_t)(now - s.btn_func.t_press);
        if (s.reset_armed) {
            s.reset_armed = false;
            ESP_LOGW(TAG, "function button: factory reset");
            local_cmd(COVER_CMD_STOP, now);
            s.halted = true;
            apply_motor(now);
            if (s.func_cb) {
                s.func_cb(COVER_FUNC_FACTORY_RESET, s.cb_arg);
            }
        } else if (dur >= s.cfg.func_commission_hold_ms) {
            ESP_LOGI(TAG, "function button: open commissioning window");
            if (s.func_cb) {
                s.func_cb(COVER_FUNC_OPEN_COMMISSIONING, s.cb_arg);
            }
        } else {
            ESP_LOGI(TAG, "function button short press (%u ms): no action", (unsigned)dur);
        }
    }
}

/* -------------------------------------------------------------------- task */

static void handle_msg(const msg_t *m, uint32_t now)
{
    switch (m->type) {
    case MSG_CMD:
        run_cmd(&m->cmd, now);
        break;
    case MSG_SET_CALIB: {
        cover_result_t r = cover_sm_set_calibration(&s.sm, &m->calib, now);
        ESP_LOGI(TAG, "manual calibration open=%u close=%u ms -> %s", (unsigned)m->calib.open_ms,
                 (unsigned)m->calib.close_ms, cover_result_str(r));
        break;
    }
    case MSG_CLEAR_CALIB:
        cover_sm_set_calibration(&s.sm, NULL, now);
        break;
    case MSG_HALT:
        if (s.sm.state != COVER_ST_FAULT) {
            cover_cmd_t c = {.type = COVER_CMD_STOP, .source = COVER_SRC_INTERNAL};
            cover_sm_command(&s.sm, &c, now);
        }
        s.halted = true;
        apply_motor(now);
        xSemaphoreGive(s.halt_done);
        break;
    }
}

static void control_tick(uint32_t *last_ms)
{
    uint32_t now = now_ms();
    if (s.sim_active) {
        sim_plant_step(&s.sim, now - *last_ms);
    }
    *last_ms = now;
    sample_inputs(now, false);
    cover_sm_inputs_t in = sm_inputs();
    cover_sm_tick(&s.sm, &in, now);
    process_buttons(now);
    apply_motor(now);
    handle_events();
}

static void control_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(s.cfg.period_ms) ? pdMS_TO_TICKS(s.cfg.period_ms) : 1;
    TickType_t next = xTaskGetTickCount();
    uint32_t last_ms = now_ms();

    for (;;) {
        /* Periodic part first so a flood of commands can never starve the
         * limit switch / timeout supervision. */
        if ((int32_t)(xTaskGetTickCount() - next) >= 0) {
            control_tick(&last_ms);
            next += period;
            if ((int32_t)(xTaskGetTickCount() - next) >= 0) {
                next = xTaskGetTickCount() + period; /* fell behind: resync */
            }
        }
        int32_t remaining = (int32_t)(next - xTaskGetTickCount());
        msg_t m;
        if (xQueueReceive(s.q, &m, remaining > 0 ? (TickType_t)remaining : 0) == pdTRUE) {
            uint32_t now = now_ms();
            handle_msg(&m, now);
            while (xQueueReceive(s.q, &m, 0) == pdTRUE) {
                handle_msg(&m, now);
            }
            apply_motor(now);
            handle_events();
        }
    }
}

esp_err_t cover_controller_start(const cover_controller_config_t *cfg, cover_status_cb_t status_cb,
                                 cover_func_cb_t func_cb, void *cb_arg)
{
    if (s.task) {
        return ESP_ERR_INVALID_STATE;
    }
    s.cfg = *cfg;
    s.status_cb = status_cb;
    s.func_cb = func_cb;
    s.cb_arg = cb_arg;
    uint32_t now = now_ms();

    /* 1. Motor driver first so outputs are forced inactive as early as possible. */
    esp_err_t err;
    s.sim_active = cfg->simulation;
    if (s.sim_active) {
        err = motor_driver_sim_create(&s.sim, cfg->sm.deadtime_ms, now, &s.drv);
    } else {
        err = motor_driver_gpio2_create(&cfg->gpio2, now, &s.drv);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor driver init failed: %s", esp_err_to_name(err));
        return err;
    }
    s.applied = COVER_DIR_NONE;

    /* 2. Inputs. In simulation the limit switches come from the simulated plant. */
    cover_inputs_config_t in_cfg = cfg->inputs;
    if (s.sim_active) {
        in_cfg.pin[COVER_IN_LIMIT_OPEN].gpio = -1;
        in_cfg.pin[COVER_IN_LIMIT_CLOSED].gpio = -1;
        s.cfg.sm.has_limit_open = cfg->sim_limit_open;
        s.cfg.sm.has_limit_closed = cfg->sim_limit_closed;
    } else {
        s.cfg.sm.has_limit_open = in_cfg.pin[COVER_IN_LIMIT_OPEN].gpio >= 0;
        s.cfg.sm.has_limit_closed = in_cfg.pin[COVER_IN_LIMIT_CLOSED].gpio >= 0;
    }
    err = cover_inputs_init(&in_cfg);
    if (err != ESP_OK) {
        return err;
    }

    /* 3. Stored calibration and position. */
    cover_calib_t calib;
    bool have_calib = cover_storage_load_calib(&calib);
    if (!have_calib && cfg->default_calib.open_ms && cfg->default_calib.close_ms) {
        calib = cfg->default_calib;
        have_calib = true;
        ESP_LOGI(TAG, "using travel times from Kconfig");
    }
    int32_t restored = -1;
    cover_pos_record_t rec;
    if (cover_storage_load_pos(&rec)) {
        if (rec.moving) {
            ESP_LOGW(TAG, "power lost or reset during a run: position unknown, NOT moving automatically");
        } else if (rec.valid && cfg->restore_position) {
            restored = rec.pos;
        }
    }

    if (s.sim_active) {
        sim_plant_init(&s.sim, cfg->sim_open_ms, cfg->sim_close_ms, cfg->sim_limit_open, cfg->sim_limit_closed,
                       restored >= 0 ? restored : 5000);
    }

    sample_inputs(now, true);
    cover_sm_inputs_t in = sm_inputs();
    cover_sm_init(&s.sm, &s.cfg.sm, have_calib ? &calib : NULL, restored, &in, now);
    s.btn_open.pressed = s.db_in[COVER_IN_BTN_OPEN].stable;   /* a held button at boot is not a press */
    s.btn_close.pressed = s.db_in[COVER_IN_BTN_CLOSE].stable;
    s.btn_stop.pressed = s.db_in[COVER_IN_BTN_STOP].stable;
    s.btn_func.pressed = s.db_in[COVER_IN_BTN_FUNC].stable;
    s.btn_open.t_press = s.btn_close.t_press = s.btn_stop.t_press = s.btn_func.t_press = now;
    cover_sm_get_status(&s.sm, &s.snapshot);

    ESP_LOGI(TAG, "mode %s, driver %s, limit switches open:%s closed:%s, %s, position %s",
             s.sim_active ? "SIMULATION" : "HARDWARE", s.drv->name, s.cfg.sm.has_limit_open ? "yes" : "no",
             s.cfg.sm.has_limit_closed ? "yes" : "no", have_calib ? "calibrated" : "NOT calibrated",
             cover_pos_quality_str(s.sm.pos_q));

    s.q = xQueueCreate(QUEUE_LEN, sizeof(msg_t));
    s.halt_done = xSemaphoreCreateBinary();
    if (!s.q || !s.halt_done) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(control_task, "cover_ctl", TASK_STACK, NULL, TASK_PRIO, &s.task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t cover_controller_post(const cover_cmd_t *cmd)
{
    if (!s.q || !cmd) {
        return ESP_ERR_INVALID_STATE;
    }
    msg_t m = {.type = MSG_CMD, .cmd = *cmd};
    BaseType_t ok = cmd->type == COVER_CMD_STOP ? xQueueSendToFront(s.q, &m, 0) : xQueueSendToBack(s.q, &m, 0);
    if (ok != pdTRUE) {
        ESP_LOGE(TAG, "command queue full, %s dropped", cmd_str(cmd));
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t cover_controller_set_calibration(const cover_calib_t *calib)
{
    if (!s.q) {
        return ESP_ERR_INVALID_STATE;
    }
    msg_t m = {.type = calib ? MSG_SET_CALIB : MSG_CLEAR_CALIB};
    if (calib) {
        m.calib = *calib;
    }
    return xQueueSendToBack(s.q, &m, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t cover_controller_halt(uint32_t timeout_ms)
{
    if (!s.q) {
        return ESP_ERR_INVALID_STATE;
    }
    msg_t m = {.type = MSG_HALT};
    if (xQueueSendToFront(s.q, &m, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return xSemaphoreTake(s.halt_done, pdMS_TO_TICKS(timeout_ms)) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

void cover_controller_get_status(cover_status_t *out)
{
    taskENTER_CRITICAL(&s.lock);
    *out = s.snapshot;
    taskEXIT_CRITICAL(&s.lock);
}

sim_plant_t *cover_controller_sim(void)
{
    return s.sim_active ? &s.sim : NULL;
}
