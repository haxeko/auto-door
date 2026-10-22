/*
 * Host unit tests for the cover state machine and debouncer.
 *
 *   cmake -S test/host -B build-host && cmake --build build-host && ./build-host/test_cover_sm
 *
 * A tiny plant model (true position + limit switches) closes the loop so the
 * tests exercise the same tick/command sequence the firmware uses.
 * Invariants checked on EVERY tick:
 *   - a direction change always has the motor off for >= deadtime in between;
 *   - after (re)init the motor never starts before deadtime has elapsed.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cover_sm.h"
#include "debounce.h"

static int g_failures;
static int g_checks;

#define CHECK(cond)                                                                \
    do {                                                                           \
        g_checks++;                                                                \
        if (!(cond)) {                                                             \
            g_failures++;                                                          \
            fprintf(stderr, "  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
        }                                                                          \
    } while (0)

#define CHECK_EQ(a, b)                                                             \
    do {                                                                           \
        g_checks++;                                                                \
        long _a = (long)(a), _b = (long)(b);                                       \
        if (_a != _b) {                                                            \
            g_failures++;                                                          \
            fprintf(stderr, "  FAIL %s:%d: %s == %s (%ld != %ld)\n", __FILE__,     \
                    __LINE__, #a, #b, _a, _b);                                     \
        }                                                                          \
    } while (0)

#define TICK_MS 10u
#define DEADTIME 500u

/* ---------------------------------------------------------------- plant --- */
typedef struct {
    int32_t pos;          /* true position 0 (open) .. 10000 (closed) */
    uint32_t open_ms;     /* real travel times */
    uint32_t close_ms;
    bool has_limit_open, has_limit_closed;
    bool jammed;          /* motor energised but nothing moves */
    bool reversed;        /* motor wired the wrong way round */
    bool force_limit_open, force_limit_closed; /* stuck switches */
    bool obstacle, driver_fault, stop_held;
    double acc;           /* sub-unit accumulator */
} plant_t;

typedef struct {
    cover_sm_t sm;
    plant_t pl;
    uint32_t now;
    cover_dir_t last_motor;
    uint32_t t_last_off;
    uint32_t violations;
    uint32_t starts;
} rig_t;

static cover_sm_config_t default_cfg(void)
{
    cover_sm_config_t c = {
        .has_limit_open = true,
        .has_limit_closed = true,
        .deadtime_ms = DEADTIME,
        .max_run_uncal_ms = 60000,
        .timeout_margin_pct = 20,
        .end_overrun_pct = 5,
        .min_travel_ms = 1000,
        .pos_report_step = 100,
        .goto_tolerance = 50,
    };
    return c;
}

static cover_sm_inputs_t plant_inputs(const plant_t *p)
{
    cover_sm_inputs_t in = {0};
    in.limit_open = p->force_limit_open || (p->has_limit_open && p->pos <= 0);
    in.limit_closed = p->force_limit_closed || (p->has_limit_closed && p->pos >= 10000);
    in.obstacle = p->obstacle;
    in.driver_fault = p->driver_fault;
    in.stop_held = p->stop_held;
    return in;
}

static void plant_step(plant_t *p, cover_dir_t motor, uint32_t dt)
{
    if (motor == COVER_DIR_NONE || p->jammed) {
        return;
    }
    cover_dir_t eff = motor;
    if (p->reversed) {
        eff = motor == COVER_DIR_OPEN ? COVER_DIR_CLOSE : COVER_DIR_OPEN;
    }
    double rate = eff == COVER_DIR_OPEN ? -10000.0 / p->open_ms : 10000.0 / p->close_ms;
    p->acc += rate * dt;
    int32_t d = (int32_t)p->acc;
    p->acc -= d;
    p->pos += d;
    if (p->pos < 0) p->pos = 0;
    if (p->pos > 10000) p->pos = 10000;
}

static void rig_check_motor(rig_t *r)
{
    cover_dir_t m = cover_sm_motor(&r->sm);
    if (m != r->last_motor) {
        if (r->last_motor != COVER_DIR_NONE && m != COVER_DIR_NONE) {
            r->violations++; /* direct reversal without off period */
            fprintf(stderr, "  direct reversal at %u\n", r->now);
        }
        if (r->last_motor != COVER_DIR_NONE) {
            r->t_last_off = r->now;
        }
        if (m != COVER_DIR_NONE) {
            r->starts++;
            if ((uint32_t)(r->now - r->t_last_off) < DEADTIME) {
                r->violations++;
                fprintf(stderr, "  dead time violated at %u (off since %u)\n", r->now, r->t_last_off);
            }
        }
        r->last_motor = m;
    }
}

static void rig_init(rig_t *r, cover_sm_config_t cfg, const cover_calib_t *cal, int32_t restored, uint32_t t0)
{
    memset(r, 0, sizeof(*r));
    r->now = t0;
    r->pl.open_ms = 10000;
    r->pl.close_ms = 12000;
    r->pl.has_limit_open = cfg.has_limit_open;
    r->pl.has_limit_closed = cfg.has_limit_closed;
    r->pl.pos = 5000;
    r->t_last_off = t0;
    cover_sm_inputs_t in = plant_inputs(&r->pl);
    cover_sm_init(&r->sm, &cfg, cal, restored, &in, r->now);
    r->last_motor = cover_sm_motor(&r->sm);
}

/* Re-init the state machine only (simulated reset), plant keeps its position. */
static void rig_reboot(rig_t *r, cover_sm_config_t cfg, const cover_calib_t *cal, int32_t restored)
{
    /* On reset the GPIOs drop to inactive immediately. */
    if (r->last_motor != COVER_DIR_NONE) {
        r->t_last_off = r->now;
    }
    r->last_motor = COVER_DIR_NONE;
    cover_sm_inputs_t in = plant_inputs(&r->pl);
    cover_sm_init(&r->sm, &cfg, cal, restored, &in, r->now);
    rig_check_motor(r);
}

static void rig_run(rig_t *r, uint32_t ms)
{
    for (uint32_t t = 0; t < ms; t += TICK_MS) {
        r->now += TICK_MS;
        plant_step(&r->pl, cover_sm_motor(&r->sm), TICK_MS);
        cover_sm_inputs_t in = plant_inputs(&r->pl);
        cover_sm_tick(&r->sm, &in, r->now);
        rig_check_motor(r);
    }
}

/* Run until the state machine is idle (or fault), with an upper bound. */
static void rig_run_until_idle(rig_t *r, uint32_t max_ms)
{
    for (uint32_t t = 0; t < max_ms; t += TICK_MS) {
        cover_state_t s = r->sm.state;
        if (s != COVER_ST_OPENING && s != COVER_ST_CLOSING && s != COVER_ST_DEADTIME) {
            return;
        }
        rig_run(r, TICK_MS);
    }
}

static cover_result_t rig_cmd(rig_t *r, cover_cmd_type_t type, uint16_t target)
{
    cover_cmd_t c = {.type = type, .source = COVER_SRC_CONSOLE, .target = target};
    cover_result_t res = cover_sm_command(&r->sm, &c, r->now);
    rig_check_motor(r);
    return res;
}

static const cover_calib_t k_cal = {.open_ms = 10000, .close_ms = 12000};

/* ---------------------------------------------------------------- tests --- */

static void test_boot_is_safe(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, 3000, 1000);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    CHECK_EQ(r.sm.state, COVER_ST_STOPPED);
    CHECK_EQ(r.sm.pos_q, COVER_POS_ESTIMATED);
    CHECK_EQ(r.sm.pos, 3000);
    rig_run(&r, 5000);
    CHECK_EQ(r.starts, 0); /* never moves by itself */

    rig_init(&r, default_cfg(), NULL, 3000, 1000);
    CHECK_EQ(r.sm.state, COVER_ST_UNCALIBRATED);
    CHECK_EQ(r.sm.pos_q, COVER_POS_UNKNOWN); /* restored pos ignored without calibration */
}

static void test_boot_deadtime(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, 5000, 0);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_OPEN, 0), COVER_OK);
    CHECK_EQ(r.sm.state, COVER_ST_DEADTIME); /* boot counts as "just switched off" */
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    rig_run(&r, DEADTIME);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_OPEN);
    CHECK_EQ(r.violations, 0);
}

static void test_open_to_limit_uncalibrated(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), NULL, -1, 0);
    rig_run(&r, 1000);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_OPEN, 0), COVER_OK);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_OPEN);
    CHECK_EQ(r.sm.state, COVER_ST_OPENING);
    rig_run_until_idle(&r, 20000);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    CHECK_EQ(r.sm.state, COVER_ST_UNCALIBRATED);
    CHECK_EQ(r.sm.pos, COVER_POS_OPEN);
    CHECK_EQ(r.sm.pos_q, COVER_POS_AT_LIMIT);
    CHECK_EQ(r.pl.pos, 0);
    /* Already open: no-op, no motion. */
    CHECK_EQ(rig_cmd(&r, COVER_CMD_OPEN, 0), COVER_OK_NOOP);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
}

static void test_reversal_deadtime(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, 5000, 0);
    rig_run(&r, 1000);
    rig_cmd(&r, COVER_CMD_OPEN, 0);
    rig_run(&r, 2000);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_OPEN);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_CLOSE, 0), COVER_OK);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE); /* off immediately */
    CHECK_EQ(r.sm.state, COVER_ST_DEADTIME);
    rig_run(&r, DEADTIME - TICK_MS);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    rig_run(&r, TICK_MS);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_CLOSE);
    /* Reverse again quickly, several times. */
    for (int i = 0; i < 5; i++) {
        rig_cmd(&r, i % 2 ? COVER_CMD_CLOSE : COVER_CMD_OPEN, 0);
        rig_run(&r, 120);
    }
    rig_run_until_idle(&r, 30000);
    CHECK_EQ(r.violations, 0);
}

static void test_stop_priority(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, 5000, 0);
    rig_run(&r, 1000);
    rig_cmd(&r, COVER_CMD_CLOSE, 0);
    rig_run(&r, 1000);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_STOP, 0), COVER_OK);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    CHECK_EQ(r.sm.state, COVER_ST_STOPPED);

    /* STOP during dead time cancels the pending run. */
    rig_cmd(&r, COVER_CMD_OPEN, 0);
    CHECK_EQ(r.sm.state, COVER_ST_DEADTIME);
    rig_cmd(&r, COVER_CMD_STOP, 0);
    rig_run(&r, 2000);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);

    /* Held stop button: stops a run and inhibits new ones. */
    rig_cmd(&r, COVER_CMD_OPEN, 0);
    rig_run(&r, 1000);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_OPEN);
    r.pl.stop_held = true;
    rig_run(&r, TICK_MS);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_CLOSE, 0), COVER_ERR_INHIBITED);
    rig_run(&r, 1000);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    r.pl.stop_held = false;
    rig_run(&r, TICK_MS);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_CLOSE, 0), COVER_OK);
    CHECK_EQ(r.violations, 0);
}

static void test_limit_switch_stops_close(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, 5000, 0);
    rig_run(&r, 1000);
    rig_cmd(&r, COVER_CMD_CLOSE, 0);
    rig_run_until_idle(&r, 20000);
    CHECK_EQ(r.sm.state, COVER_ST_STOPPED);
    CHECK_EQ(r.sm.pos, COVER_POS_CLOSED);
    CHECK_EQ(r.sm.pos_q, COVER_POS_AT_LIMIT);
    CHECK_EQ(r.pl.pos, 10000);
}

static void test_timeout_jam(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, 5000, 0);
    rig_run(&r, 1000);
    r.pl.jammed = true;
    rig_cmd(&r, COVER_CMD_CLOSE, 0);
    rig_run_until_idle(&r, 30000);
    CHECK_EQ(r.sm.state, COVER_ST_FAULT);
    CHECK_EQ(r.sm.fault, COVER_FAULT_TIMEOUT);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    CHECK_EQ(r.sm.pos_q, COVER_POS_UNKNOWN);
    /* Timeout must happen within travel * (1 + margin + overrun). */
    CHECK(r.now <= 1000 + DEADTIME + 12000 * 125 / 100 + 2 * TICK_MS);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_OPEN, 0), COVER_ERR_FAULT);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_STOP, 0), COVER_OK); /* acknowledge */
    CHECK_EQ(r.sm.state, COVER_ST_STOPPED);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_GOTO, 5000), COVER_ERR_POS_UNKNOWN);
}

static void test_timeout_uncalibrated(void)
{
    rig_t r;
    cover_sm_config_t cfg = default_cfg();
    cfg.max_run_uncal_ms = 8000;
    rig_init(&r, cfg, NULL, -1, 0);
    rig_run(&r, 1000);
    r.pl.jammed = true;
    rig_cmd(&r, COVER_CMD_OPEN, 0);
    rig_run(&r, 7900);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_OPEN);
    rig_run(&r, 200);
    CHECK_EQ(r.sm.fault, COVER_FAULT_TIMEOUT);
}

static void test_obstacle_no_reverse(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, 2000, 0);
    rig_run(&r, 1000);
    rig_cmd(&r, COVER_CMD_CLOSE, 0);
    rig_run(&r, 2000);
    r.pl.obstacle = true;
    rig_run(&r, TICK_MS);
    CHECK_EQ(r.sm.state, COVER_ST_FAULT);
    CHECK_EQ(r.sm.fault, COVER_FAULT_OBSTACLE);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    uint32_t starts = r.starts;
    rig_run(&r, 5000);
    CHECK_EQ(r.starts, starts); /* no automatic reversal / restart */
    CHECK(r.sm.pos_q == COVER_POS_ESTIMATED); /* estimate kept after a clean stop */
    CHECK_EQ(rig_cmd(&r, COVER_CMD_STOP, 0), COVER_ERR_FAULT_ACTIVE);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_OPEN, 0), COVER_ERR_FAULT);
    r.pl.obstacle = false;
    rig_run(&r, TICK_MS);
    CHECK_EQ(r.sm.state, COVER_ST_FAULT); /* still latched */
    CHECK_EQ(rig_cmd(&r, COVER_CMD_STOP, 0), COVER_OK);
    CHECK_EQ(r.sm.state, COVER_ST_STOPPED);
    /* Obstacle present while idle: commands are refused. */
    r.pl.obstacle = true;
    rig_run(&r, TICK_MS);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_OPEN, 0), COVER_ERR_INHIBITED);
    CHECK_EQ(r.violations, 0);
}

static void test_both_limits(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, 5000, 0);
    rig_run(&r, 1000);
    rig_cmd(&r, COVER_CMD_OPEN, 0);
    rig_run(&r, 1000);
    r.pl.force_limit_open = true;
    r.pl.force_limit_closed = true;
    rig_run(&r, TICK_MS);
    CHECK_EQ(r.sm.state, COVER_ST_FAULT);
    CHECK_EQ(r.sm.fault, COVER_FAULT_LIMIT_BOTH);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_STOP, 0), COVER_ERR_FAULT_ACTIVE);
    r.pl.force_limit_closed = false;
    rig_run(&r, TICK_MS);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_STOP, 0), COVER_OK);

    /* Both active at boot -> fault immediately, no motion. */
    r.pl.force_limit_closed = true;
    rig_reboot(&r, default_cfg(), &k_cal, -1);
    CHECK_EQ(r.sm.state, COVER_ST_FAULT);
    CHECK_EQ(r.sm.fault, COVER_FAULT_LIMIT_BOTH);
}

static void test_wrong_limit(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, 5000, 0);
    r.pl.reversed = true; /* OPEN actually closes */
    rig_run(&r, 1000);
    rig_cmd(&r, COVER_CMD_OPEN, 0);
    rig_run_until_idle(&r, 30000);
    CHECK_EQ(r.sm.state, COVER_ST_FAULT);
    CHECK_EQ(r.sm.fault, COVER_FAULT_LIMIT_WRONG);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    CHECK(r.pl.pos >= 10000 - 100);
}

static void test_driver_fault(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, 5000, 0);
    rig_run(&r, 1000);
    rig_cmd(&r, COVER_CMD_OPEN, 0);
    rig_run(&r, 500);
    r.pl.driver_fault = true;
    rig_run(&r, TICK_MS);
    CHECK_EQ(r.sm.fault, COVER_FAULT_DRIVER);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
}

static void test_calibration(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), NULL, -1, 0);
    r.pl.pos = 6000;
    rig_run(&r, 1000);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_CALIBRATE, 0), COVER_OK);
    rig_run_until_idle(&r, 120000);
    CHECK_EQ(r.sm.state, COVER_ST_STOPPED);
    CHECK(r.sm.calibrated);
    CHECK_EQ(r.sm.cal_phase, COVER_CAL_IDLE);
    /* measured within 2 ticks of the real travel times */
    CHECK(abs((int)r.sm.calib.close_ms - 12000) <= 2 * (int)TICK_MS);
    CHECK(abs((int)r.sm.calib.open_ms - 10000) <= 2 * (int)TICK_MS);
    CHECK_EQ(r.sm.pos, COVER_POS_OPEN);
    CHECK_EQ(r.sm.pos_q, COVER_POS_AT_LIMIT);
    uint32_t ev = cover_sm_take_events(&r.sm);
    CHECK(ev & COVER_EVT_CALIB);
    CHECK_EQ(r.violations, 0);
}

static void test_calibration_requires_limits_and_abort(void)
{
    rig_t r;
    cover_sm_config_t cfg = default_cfg();
    cfg.has_limit_closed = false;
    rig_init(&r, cfg, NULL, -1, 0);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_CALIBRATE, 0), COVER_ERR_NO_LIMITS);

    rig_init(&r, default_cfg(), NULL, -1, 0);
    rig_run(&r, 1000);
    rig_cmd(&r, COVER_CMD_CALIBRATE, 0);
    rig_run(&r, 2000);
    cover_sm_take_events(&r.sm);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_CLOSE, 0), COVER_ERR_CAL_ABORTED);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    CHECK(cover_sm_take_events(&r.sm) & COVER_EVT_CAL_FAILED);
    CHECK(!r.sm.calibrated);
}

static void test_goto_mid(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, -1, 0);
    r.pl.pos = 0;
    rig_reboot(&r, default_cfg(), &k_cal, -1); /* at open limit -> exact */
    CHECK_EQ(r.sm.pos_q, COVER_POS_AT_LIMIT);
    rig_run(&r, 1000);
    /* Matter: 7500 = 75 % closed */
    CHECK_EQ(rig_cmd(&r, COVER_CMD_GOTO, 7500), COVER_OK);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_CLOSE);
    rig_run_until_idle(&r, 20000);
    CHECK_EQ(r.sm.state, COVER_ST_STOPPED);
    CHECK_EQ(r.sm.pos, 7500);
    CHECK_EQ(r.sm.pos_q, COVER_POS_ESTIMATED);
    CHECK(abs(r.pl.pos - 7500) <= 50); /* plant matches estimate within a tick */
    /* back towards open: 0 = fully open in Matter */
    CHECK_EQ(rig_cmd(&r, COVER_CMD_GOTO, 2500), COVER_OK);
    CHECK_EQ(r.sm.state, COVER_ST_DEADTIME);
    rig_run_until_idle(&r, 20000);
    CHECK(abs(r.pl.pos - 2500) <= 50);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_GOTO, 2520), COVER_OK_NOOP); /* within tolerance */
    CHECK_EQ(rig_cmd(&r, COVER_CMD_GOTO, 10001), COVER_ERR_INVALID);
    /* retarget in the same direction while moving: no stop */
    rig_run(&r, 1000);
    rig_cmd(&r, COVER_CMD_GOTO, 5000);
    rig_run(&r, 1000);
    uint32_t starts = r.starts;
    rig_cmd(&r, COVER_CMD_GOTO, 8000);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_CLOSE);
    rig_run_until_idle(&r, 20000);
    CHECK_EQ(r.starts, starts);
    CHECK(abs(r.pl.pos - 8000) <= 60);
    CHECK_EQ(r.violations, 0);
}

static void test_goto_rejections(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), NULL, -1, 0);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_GOTO, 5000), COVER_ERR_NOT_CALIBRATED);
    rig_init(&r, default_cfg(), &k_cal, -1, 0);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_GOTO, 5000), COVER_ERR_POS_UNKNOWN);
    /* full open / close allowed even when the position is unknown */
    CHECK_EQ(rig_cmd(&r, COVER_CMD_GOTO, 0), COVER_OK);
}

static void test_reset_while_running(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, 5000, 0);
    rig_run(&r, 1000);
    rig_cmd(&r, COVER_CMD_CLOSE, 0);
    rig_run(&r, 3000);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_CLOSE);
    /* Storage marked "moving" -> restored position is unknown. */
    rig_reboot(&r, default_cfg(), &k_cal, -1);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    CHECK_EQ(r.sm.pos_q, COVER_POS_UNKNOWN);
    uint32_t starts = r.starts;
    rig_run(&r, 10000);
    CHECK_EQ(r.starts, starts); /* does not resume or home by itself */
    CHECK_EQ(rig_cmd(&r, COVER_CMD_OPEN, 0), COVER_OK);
    rig_run_until_idle(&r, 30000);
    CHECK_EQ(r.sm.pos_q, COVER_POS_AT_LIMIT);
    CHECK_EQ(r.violations, 0);
}

static void test_no_limit_switches(void)
{
    rig_t r;
    cover_sm_config_t cfg = default_cfg();
    cfg.has_limit_open = false;
    cfg.has_limit_closed = false;
    rig_init(&r, cfg, &k_cal, 5000, 0);
    rig_run(&r, 1000);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_OPEN, 0), COVER_OK);
    uint32_t t0 = r.now;
    rig_run_until_idle(&r, 30000);
    CHECK_EQ(r.sm.state, COVER_ST_STOPPED);
    CHECK_EQ(r.sm.fault, COVER_FAULT_NONE);
    CHECK_EQ(r.sm.pos, COVER_POS_OPEN);
    CHECK_EQ(r.sm.pos_q, COVER_POS_ESTIMATED); /* never "measured" without a switch */
    /* half travel (5000 ms) + 5 % overrun of full travel (500 ms); dead time already elapsed */
    uint32_t took = r.now - t0;
    CHECK(took >= 5000 + 500 && took <= 5000 + 500 + 3 * TICK_MS);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_OPEN, 0), COVER_OK_NOOP);
}

static void test_buttons(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, 5000, 0);
    rig_run(&r, 1000);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_BUTTON_CLOSE, 0), COVER_OK);
    rig_run(&r, 1000);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_CLOSE);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_BUTTON_OPEN, 0), COVER_OK); /* press while moving = stop */
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    rig_run(&r, 2000);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_NONE);
    CHECK_EQ(rig_cmd(&r, COVER_CMD_BUTTON_OPEN, 0), COVER_OK);
    CHECK_EQ(cover_sm_motor(&r.sm), COVER_DIR_OPEN);
}

static void test_manual_move_detected(void)
{
    rig_t r;
    r.pl.pos = 0;
    rig_init(&r, default_cfg(), &k_cal, -1, 0);
    r.pl.pos = 0;
    rig_reboot(&r, default_cfg(), &k_cal, -1);
    CHECK_EQ(r.sm.pos_q, COVER_POS_AT_LIMIT);
    r.pl.pos = 1500; /* pulled by hand */
    rig_run(&r, TICK_MS);
    CHECK_EQ(r.sm.pos_q, COVER_POS_ESTIMATED);
    r.pl.pos = 10000;
    rig_run(&r, TICK_MS);
    CHECK_EQ(r.sm.pos, COVER_POS_CLOSED);
    CHECK_EQ(r.sm.pos_q, COVER_POS_AT_LIMIT);
}

static void test_time_wrap(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, 5000, UINT32_MAX - 3000);
    rig_run(&r, 1000);
    rig_cmd(&r, COVER_CMD_CLOSE, 0);
    rig_run_until_idle(&r, 20000);
    CHECK_EQ(r.sm.state, COVER_ST_STOPPED);
    CHECK_EQ(r.sm.pos_q, COVER_POS_AT_LIMIT);
    CHECK(r.now < 100000); /* really wrapped */
    CHECK_EQ(r.violations, 0);
}

static void test_events(void)
{
    rig_t r;
    rig_init(&r, default_cfg(), &k_cal, 0, 0);
    uint32_t ev = cover_sm_take_events(&r.sm);
    CHECK((ev & (COVER_EVT_STATE | COVER_EVT_POSITION | COVER_EVT_TARGET)) ==
          (COVER_EVT_STATE | COVER_EVT_POSITION | COVER_EVT_TARGET));
    rig_run(&r, 1000);
    cover_sm_take_events(&r.sm);
    rig_cmd(&r, COVER_CMD_CLOSE, 0);
    ev = cover_sm_take_events(&r.sm);
    CHECK(ev & COVER_EVT_RUN_START);
    CHECK(ev & COVER_EVT_TARGET);
    cover_status_t st;
    cover_sm_get_status(&r.sm, &st);
    CHECK(st.target_valid);
    CHECK_EQ(st.target, COVER_POS_CLOSED);
    CHECK_EQ(st.dir, COVER_DIR_CLOSE);
    /* position events are rate limited by pos_report_step (1 %) */
    int pos_events = 0;
    for (int i = 0; i < 300; i++) {
        rig_run(&r, TICK_MS);
        if (cover_sm_take_events(&r.sm) & COVER_EVT_POSITION) pos_events++;
    }
    CHECK(pos_events >= 20 && pos_events <= 26); /* 3 s of 12 s travel = 25 % */
    rig_cmd(&r, COVER_CMD_STOP, 0);
    ev = cover_sm_take_events(&r.sm);
    CHECK(ev & COVER_EVT_RUN_STOP);
    cover_sm_get_status(&r.sm, &st);
    CHECK(!st.target_valid);
}

static void test_debounce(void)
{
    debounce_t d;
    debounce_init(&d, false, 30, 0);
    CHECK(!debounce_update(&d, true, 10));
    CHECK(!debounce_update(&d, false, 20)); /* bounce resets */
    CHECK(!debounce_update(&d, true, 30));
    CHECK(!debounce_update(&d, true, 50));
    CHECK(debounce_update(&d, true, 60));
    CHECK(debounce_update(&d, false, 70));
    CHECK(!debounce_update(&d, false, 100));
    /* wrap */
    debounce_init(&d, false, 30, UINT32_MAX - 10);
    debounce_update(&d, true, UINT32_MAX - 5);
    CHECK(debounce_update(&d, true, 25));
}

typedef void (*test_fn)(void);
static const struct {
    const char *name;
    test_fn fn;
} k_tests[] = {
    {"boot_is_safe", test_boot_is_safe},
    {"boot_deadtime", test_boot_deadtime},
    {"open_to_limit_uncalibrated", test_open_to_limit_uncalibrated},
    {"reversal_deadtime", test_reversal_deadtime},
    {"stop_priority", test_stop_priority},
    {"limit_switch_stops_close", test_limit_switch_stops_close},
    {"timeout_jam", test_timeout_jam},
    {"timeout_uncalibrated", test_timeout_uncalibrated},
    {"obstacle_no_reverse", test_obstacle_no_reverse},
    {"both_limits", test_both_limits},
    {"wrong_limit", test_wrong_limit},
    {"driver_fault", test_driver_fault},
    {"calibration", test_calibration},
    {"calibration_requires_limits_and_abort", test_calibration_requires_limits_and_abort},
    {"goto_mid", test_goto_mid},
    {"goto_rejections", test_goto_rejections},
    {"reset_while_running", test_reset_while_running},
    {"no_limit_switches", test_no_limit_switches},
    {"buttons", test_buttons},
    {"manual_move_detected", test_manual_move_detected},
    {"time_wrap", test_time_wrap},
    {"events", test_events},
    {"debounce", test_debounce},
};

int main(void)
{
    for (size_t i = 0; i < sizeof(k_tests) / sizeof(k_tests[0]); i++) {
        int before = g_failures;
        k_tests[i].fn();
        printf("[%s] %s\n", g_failures == before ? " OK " : "FAIL", k_tests[i].name);
    }
    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
