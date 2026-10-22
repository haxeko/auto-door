/*
 * Cover motion state machine - see cover_sm.h for the contract.
 *
 * Safety rules enforced here (and again in the motor driver):
 *  - the motor output is a single enum, OPEN and CLOSE can never be requested together;
 *  - every start is preceded by `deadtime_ms` with the motor off (reversal, restart, boot);
 *  - init never starts the motor;
 *  - limit switch, timeout, obstacle, driver fault and implausible limit states stop the motor;
 *  - an obstacle only stops the motor. It never reverses automatically.
 */
#include "cover_sm.h"

#include <stddef.h>
#include <string.h>

#define CAL_MAX_MANUAL_MS (10u * 60u * 1000u)

static inline uint32_t elapsed_ms(const cover_sm_t *sm, uint32_t since)
{
    return (uint32_t)(sm->now - since); /* wrap-around safe */
}

static cover_state_t idle_state(const cover_sm_t *sm)
{
    return sm->calibrated ? COVER_ST_STOPPED : COVER_ST_UNCALIBRATED;
}

static bool has_limit(const cover_sm_t *sm, cover_dir_t dir)
{
    return dir == COVER_DIR_OPEN ? sm->cfg.has_limit_open : sm->cfg.has_limit_closed;
}

static bool limit_active(const cover_sm_t *sm, cover_dir_t dir)
{
    return dir == COVER_DIR_OPEN ? sm->in.limit_open : sm->in.limit_closed;
}

static uint16_t end_pos(cover_dir_t dir)
{
    return dir == COVER_DIR_OPEN ? COVER_POS_OPEN : COVER_POS_CLOSED;
}

static uint32_t travel_ms(const cover_sm_t *sm, cover_dir_t dir)
{
    return dir == COVER_DIR_OPEN ? sm->calib.open_ms : sm->calib.close_ms;
}

static void set_state(cover_sm_t *sm, cover_state_t st)
{
    if (sm->state != st) {
        sm->state = st;
        sm->events |= COVER_EVT_STATE;
    }
}

static void set_pos(cover_sm_t *sm, uint16_t pos, cover_pos_quality_t q)
{
    if (pos > COVER_POS_CLOSED) {
        pos = COVER_POS_CLOSED;
    }
    bool q_changed = (q != sm->pos_q);
    sm->pos = pos;
    sm->pos_q = q;
    if (q_changed) {
        sm->pos_reported = pos;
        sm->events |= COVER_EVT_POSITION;
        return;
    }
    uint16_t diff = pos > sm->pos_reported ? pos - sm->pos_reported : sm->pos_reported - pos;
    bool moving = sm->motor != COVER_DIR_NONE;
    if ((!moving && diff != 0) || (moving && diff >= sm->cfg.pos_report_step)) {
        sm->pos_reported = pos;
        sm->events |= COVER_EVT_POSITION;
    }
}

static void force_pos_report(cover_sm_t *sm)
{
    if (sm->pos_reported != sm->pos) {
        sm->pos_reported = sm->pos;
        sm->events |= COVER_EVT_POSITION;
    }
}

static void motor_off(cover_sm_t *sm)
{
    if (sm->motor != COVER_DIR_NONE) {
        sm->motor = COVER_DIR_NONE;
        sm->t_motor_off = sm->now;
        sm->events |= COVER_EVT_RUN_STOP | COVER_EVT_STATE;
        force_pos_report(sm);
    }
}

static void clear_run(cover_sm_t *sm)
{
    if (sm->run_dir != COVER_DIR_NONE) {
        sm->run_dir = COVER_DIR_NONE;
        sm->events |= COVER_EVT_TARGET;
    }
}

static void abort_calibration(cover_sm_t *sm)
{
    if (sm->cal_phase != COVER_CAL_IDLE) {
        sm->cal_phase = COVER_CAL_IDLE;
        sm->events |= COVER_EVT_CAL_FAILED | COVER_EVT_STATE;
    }
}

/* Normal stop: motor off, no fault. */
static void stop_run(cover_sm_t *sm)
{
    motor_off(sm);
    clear_run(sm);
    set_state(sm, idle_state(sm));
}

static void enter_fault(cover_sm_t *sm, cover_fault_t f)
{
    motor_off(sm);
    clear_run(sm);
    abort_calibration(sm);
    if (f == COVER_FAULT_TIMEOUT || f == COVER_FAULT_LIMIT_BOTH || f == COVER_FAULT_LIMIT_WRONG) {
        /* The time based estimate cannot be trusted after these. */
        if (sm->pos_q != COVER_POS_UNKNOWN) {
            set_pos(sm, sm->pos, COVER_POS_UNKNOWN);
        }
    }
    if (sm->fault != f) {
        sm->fault = f;
        sm->events |= COVER_EVT_STATE;
    }
    set_state(sm, COVER_ST_FAULT);
}

static bool fault_cause_present(const cover_sm_t *sm)
{
    switch (sm->fault) {
    case COVER_FAULT_LIMIT_BOTH:
        return sm->in.limit_open && sm->in.limit_closed;
    case COVER_FAULT_DRIVER:
        return sm->in.driver_fault;
    case COVER_FAULT_OBSTACLE:
        return sm->in.obstacle;
    case COVER_FAULT_OUTPUT:
        return sm->in.output_fault;
    default:
        return false;
    }
}

static uint32_t run_limit_ms(const cover_sm_t *sm, cover_dir_t dir)
{
    if (!sm->calibrated || sm->cal_phase != COVER_CAL_IDLE) {
        return sm->cfg.max_run_uncal_ms;
    }
    uint64_t t = (uint64_t)travel_ms(sm, dir) * (100u + sm->cfg.timeout_margin_pct + sm->cfg.end_overrun_pct) / 100u;
    return t > UINT32_MAX ? UINT32_MAX : (uint32_t)t;
}

/* Run time needed to reach an end that has no limit switch. */
static uint32_t end_run_without_switch_ms(const cover_sm_t *sm, cover_dir_t dir)
{
    if (!sm->calibrated) {
        /* No idea how long the travel is: rely on the motor's own end stop and run the
         * configured maximum. Documented requirement for setups without limit switches. */
        return sm->cfg.max_run_uncal_ms;
    }
    uint32_t full = travel_ms(sm, dir);
    uint64_t remaining = full;
    if (sm->run_pos_q_start != COVER_POS_UNKNOWN) {
        uint32_t dist = dir == COVER_DIR_OPEN ? sm->run_pos_start : (COVER_POS_CLOSED - sm->run_pos_start);
        remaining = (uint64_t)full * dist / COVER_POS_CLOSED;
    }
    remaining += (uint64_t)full * sm->cfg.end_overrun_pct / 100u;
    return remaining > UINT32_MAX ? UINT32_MAX : (uint32_t)remaining;
}

static void arrived(cover_sm_t *sm);

/* Energise the motor for the pending run. Preconditions re-checked here. */
static void start_run(cover_sm_t *sm)
{
    cover_dir_t dir = sm->run_dir;
    if (dir == COVER_DIR_NONE) {
        set_state(sm, idle_state(sm));
        return;
    }
    if (sm->in.stop_held || sm->in.obstacle) {
        clear_run(sm);
        abort_calibration(sm);
        set_state(sm, idle_state(sm));
        return;
    }
    if (sm->run_to_end && has_limit(sm, dir) && limit_active(sm, dir)) {
        /* Already at that end. */
        set_pos(sm, end_pos(dir), COVER_POS_AT_LIMIT);
        arrived(sm);
        return;
    }
    sm->run_t_start = sm->now;
    sm->run_pos_start = sm->pos;
    sm->run_pos_q_start = sm->pos_q;
    sm->run_t_max = run_limit_ms(sm, dir);
    if (sm->run_to_end && !has_limit(sm, dir)) {
        uint32_t need = end_run_without_switch_ms(sm, dir);
        if (sm->run_t_max <= need) {
            sm->run_t_max = need + 1000u;
        }
    }
    sm->motor = dir;
    sm->events |= COVER_EVT_RUN_START | COVER_EVT_STATE;
    set_state(sm, dir == COVER_DIR_OPEN ? COVER_ST_OPENING : COVER_ST_CLOSING);
}

/* Ask for a run. Handles same direction update, reversal and dead time. */
static void request_run(cover_sm_t *sm, cover_dir_t dir, uint16_t target, bool to_end)
{
    if (sm->run_dir != dir || sm->run_target != target || sm->run_to_end != to_end) {
        sm->events |= COVER_EVT_TARGET;
    }
    if (sm->motor == dir) {
        /* Same direction: just retarget, keep timing reference. */
        sm->run_dir = dir;
        sm->run_target = target;
        sm->run_to_end = to_end;
        return;
    }
    motor_off(sm); /* no-op when already off; reversal otherwise */
    sm->run_dir = dir;
    sm->run_target = target;
    sm->run_to_end = to_end;
    if (elapsed_ms(sm, sm->t_motor_off) >= sm->cfg.deadtime_ms) {
        start_run(sm);
    } else {
        set_state(sm, COVER_ST_DEADTIME);
    }
}

static void calibration_step(cover_sm_t *sm, cover_dir_t arrived_dir)
{
    switch (sm->cal_phase) {
    case COVER_CAL_TO_OPEN:
        if (arrived_dir == COVER_DIR_OPEN) {
            sm->cal_phase = COVER_CAL_MEASURE_CLOSE;
            request_run(sm, COVER_DIR_CLOSE, COVER_POS_CLOSED, true);
        }
        break;
    case COVER_CAL_MEASURE_CLOSE:
        if (arrived_dir == COVER_DIR_CLOSE) {
            sm->cal_t_close = elapsed_ms(sm, sm->run_t_start);
            sm->cal_phase = COVER_CAL_MEASURE_OPEN;
            request_run(sm, COVER_DIR_OPEN, COVER_POS_OPEN, true);
        }
        break;
    case COVER_CAL_MEASURE_OPEN:
        if (arrived_dir == COVER_DIR_OPEN) {
            uint32_t t_open = elapsed_ms(sm, sm->run_t_start);
            uint32_t t_close = sm->cal_t_close;
            sm->cal_phase = COVER_CAL_IDLE;
            if (t_open < sm->cfg.min_travel_ms || t_close < sm->cfg.min_travel_ms) {
                sm->events |= COVER_EVT_CAL_FAILED | COVER_EVT_STATE;
            } else {
                sm->calib.open_ms = t_open;
                sm->calib.close_ms = t_close;
                sm->calibrated = true;
                sm->events |= COVER_EVT_CALIB | COVER_EVT_STATE;
            }
            set_state(sm, idle_state(sm));
        }
        break;
    default:
        break;
    }
}

/* Run finished normally (target or end reached). */
static void arrived(cover_sm_t *sm)
{
    cover_dir_t dir = sm->run_dir != COVER_DIR_NONE ? sm->run_dir : sm->motor;
    stop_run(sm);
    if (sm->cal_phase != COVER_CAL_IDLE) {
        calibration_step(sm, dir);
    }
}

static void update_estimate(cover_sm_t *sm, uint32_t el)
{
    if (!sm->calibrated || sm->run_pos_q_start == COVER_POS_UNKNOWN) {
        return; /* stays unknown until a limit switch is reached */
    }
    uint32_t travel = travel_ms(sm, sm->motor);
    if (travel == 0) {
        return;
    }
    uint64_t delta = (uint64_t)el * COVER_POS_CLOSED / travel;
    int32_t p;
    if (sm->motor == COVER_DIR_OPEN) {
        p = (int32_t)sm->run_pos_start - (int32_t)(delta > COVER_POS_CLOSED ? COVER_POS_CLOSED : delta);
    } else {
        p = (int32_t)sm->run_pos_start + (int32_t)(delta > COVER_POS_CLOSED ? COVER_POS_CLOSED : delta);
    }
    if (p < (int32_t)COVER_POS_OPEN) {
        p = COVER_POS_OPEN;
    }
    if (p > (int32_t)COVER_POS_CLOSED) {
        p = COVER_POS_CLOSED;
    }
    set_pos(sm, (uint16_t)p, COVER_POS_ESTIMATED);
}

static void running_tick(cover_sm_t *sm)
{
    cover_dir_t dir = sm->motor;

    if (sm->in.obstacle) {
        enter_fault(sm, COVER_FAULT_OBSTACLE);
        return;
    }
    if (sm->in.stop_held) {
        abort_calibration(sm);
        stop_run(sm);
        return;
    }
    if (dir == COVER_DIR_OPEN && sm->in.limit_closed && !sm->in_prev.limit_closed) {
        enter_fault(sm, COVER_FAULT_LIMIT_WRONG);
        return;
    }
    if (dir == COVER_DIR_CLOSE && sm->in.limit_open && !sm->in_prev.limit_open) {
        enter_fault(sm, COVER_FAULT_LIMIT_WRONG);
        return;
    }

    uint32_t el = elapsed_ms(sm, sm->run_t_start);
    update_estimate(sm, el);

    if (has_limit(sm, dir) && limit_active(sm, dir)) {
        set_pos(sm, end_pos(dir), COVER_POS_AT_LIMIT);
        arrived(sm);
        return;
    }

    if (!sm->run_to_end) {
        bool reached = (dir == COVER_DIR_OPEN) ? (sm->pos <= sm->run_target) : (sm->pos >= sm->run_target);
        if (sm->pos_q != COVER_POS_UNKNOWN && reached) {
            set_pos(sm, sm->run_target, COVER_POS_ESTIMATED);
            arrived(sm);
            return;
        }
    } else if (!has_limit(sm, dir)) {
        if (el >= end_run_without_switch_ms(sm, dir)) {
            /* Assumed at the end (motor internal end stop); still only an estimate. */
            set_pos(sm, end_pos(dir), COVER_POS_ESTIMATED);
            arrived(sm);
            return;
        }
    }

    if (el >= sm->run_t_max) {
        enter_fault(sm, COVER_FAULT_TIMEOUT);
    }
}

static void idle_tick(cover_sm_t *sm)
{
    /* Motor is off: only limit switches can tell us something (e.g. manual move). */
    if (sm->in.limit_open && !sm->in_prev.limit_open) {
        set_pos(sm, COVER_POS_OPEN, COVER_POS_AT_LIMIT);
    } else if (sm->in.limit_closed && !sm->in_prev.limit_closed) {
        set_pos(sm, COVER_POS_CLOSED, COVER_POS_AT_LIMIT);
    } else if (sm->pos_q == COVER_POS_AT_LIMIT) {
        bool at_open = sm->pos == COVER_POS_OPEN && sm->cfg.has_limit_open && !sm->in.limit_open;
        bool at_closed = sm->pos == COVER_POS_CLOSED && sm->cfg.has_limit_closed && !sm->in.limit_closed;
        if (at_open || at_closed) {
            /* Switch released without the motor running: moved by hand. */
            set_pos(sm, sm->pos, COVER_POS_ESTIMATED);
        }
    }
}

static bool inputs_differ(const cover_sm_inputs_t *a, const cover_sm_inputs_t *b)
{
    return a->limit_open != b->limit_open || a->limit_closed != b->limit_closed || a->obstacle != b->obstacle ||
           a->driver_fault != b->driver_fault || a->stop_held != b->stop_held || a->output_fault != b->output_fault;
}

void cover_sm_init(cover_sm_t *sm, const cover_sm_config_t *cfg, const cover_calib_t *calib, int32_t restored_pos,
                   const cover_sm_inputs_t *inputs, uint32_t now_ms)
{
    memset(sm, 0, sizeof(*sm));
    sm->cfg = *cfg;
    if (sm->cfg.pos_report_step == 0) {
        sm->cfg.pos_report_step = 1;
    }
    sm->now = now_ms;
    sm->in = *inputs;
    sm->in_prev = *inputs;
    sm->motor = COVER_DIR_NONE;
    /* Treat boot as "motor just switched off": a reset may have interrupted a run. */
    sm->t_motor_off = now_ms;

    if (calib && calib->open_ms >= cfg->min_travel_ms && calib->close_ms >= cfg->min_travel_ms &&
        calib->open_ms <= CAL_MAX_MANUAL_MS && calib->close_ms <= CAL_MAX_MANUAL_MS) {
        sm->calib = *calib;
        sm->calibrated = true;
    }

    if (sm->in.limit_open && !sm->in.limit_closed) {
        sm->pos = COVER_POS_OPEN;
        sm->pos_q = COVER_POS_AT_LIMIT;
    } else if (sm->in.limit_closed && !sm->in.limit_open) {
        sm->pos = COVER_POS_CLOSED;
        sm->pos_q = COVER_POS_AT_LIMIT;
    } else if (restored_pos >= 0 && restored_pos <= (int32_t)COVER_POS_CLOSED && sm->calibrated) {
        sm->pos = (uint16_t)restored_pos;
        sm->pos_q = COVER_POS_ESTIMATED;
    } else {
        sm->pos = 0;
        sm->pos_q = COVER_POS_UNKNOWN;
    }
    sm->pos_reported = sm->pos;
    sm->state = idle_state(sm);
    sm->events = COVER_EVT_STATE | COVER_EVT_POSITION | COVER_EVT_TARGET;

    if (sm->in.output_fault) {
        enter_fault(sm, COVER_FAULT_OUTPUT);
    } else if (sm->in.driver_fault) {
        enter_fault(sm, COVER_FAULT_DRIVER);
    } else if (sm->in.limit_open && sm->in.limit_closed) {
        enter_fault(sm, COVER_FAULT_LIMIT_BOTH);
    }
}

void cover_sm_tick(cover_sm_t *sm, const cover_sm_inputs_t *inputs, uint32_t now_ms)
{
    sm->now = now_ms;
    sm->in_prev = sm->in;
    sm->in = *inputs;
    if (inputs_differ(&sm->in, &sm->in_prev)) {
        sm->events |= COVER_EVT_STATE; /* safety status may have changed */
    }

    if (sm->state != COVER_ST_FAULT) {
        if (sm->in.output_fault) {
            enter_fault(sm, COVER_FAULT_OUTPUT);
        } else if (sm->in.driver_fault) {
            enter_fault(sm, COVER_FAULT_DRIVER);
        } else if (sm->in.limit_open && sm->in.limit_closed) {
            enter_fault(sm, COVER_FAULT_LIMIT_BOTH);
        }
    }

    switch (sm->state) {
    case COVER_ST_FAULT:
        break;
    case COVER_ST_DEADTIME:
        if (sm->in.stop_held || sm->in.obstacle) {
            clear_run(sm);
            abort_calibration(sm);
            set_state(sm, idle_state(sm));
        } else if (elapsed_ms(sm, sm->t_motor_off) >= sm->cfg.deadtime_ms) {
            start_run(sm);
        }
        break;
    case COVER_ST_OPENING:
    case COVER_ST_CLOSING:
        running_tick(sm);
        break;
    default:
        idle_tick(sm);
        break;
    }
}

static bool is_active(const cover_sm_t *sm)
{
    return sm->state == COVER_ST_OPENING || sm->state == COVER_ST_CLOSING || sm->state == COVER_ST_DEADTIME;
}

static cover_result_t do_stop(cover_sm_t *sm)
{
    if (sm->state == COVER_ST_FAULT) {
        if (fault_cause_present(sm)) {
            return COVER_ERR_FAULT_ACTIVE;
        }
        sm->fault = COVER_FAULT_NONE;
        sm->events |= COVER_EVT_STATE;
        set_state(sm, idle_state(sm));
        return COVER_OK;
    }
    bool was_active = is_active(sm);
    abort_calibration(sm);
    stop_run(sm);
    return was_active ? COVER_OK : COVER_OK_NOOP;
}

static cover_result_t do_move_end(cover_sm_t *sm, cover_dir_t dir)
{
    if (!is_active(sm)) {
        if (has_limit(sm, dir) && limit_active(sm, dir)) {
            set_pos(sm, end_pos(dir), COVER_POS_AT_LIMIT);
            return COVER_OK_NOOP;
        }
        if (!has_limit(sm, dir) && sm->pos_q != COVER_POS_UNKNOWN && sm->pos == end_pos(dir)) {
            return COVER_OK_NOOP;
        }
    }
    request_run(sm, dir, end_pos(dir), true);
    return COVER_OK;
}

static cover_result_t do_goto(cover_sm_t *sm, uint16_t target)
{
    if (target > COVER_POS_CLOSED) {
        return COVER_ERR_INVALID;
    }
    if (target == COVER_POS_OPEN) {
        return do_move_end(sm, COVER_DIR_OPEN);
    }
    if (target == COVER_POS_CLOSED) {
        return do_move_end(sm, COVER_DIR_CLOSE);
    }
    if (!sm->calibrated) {
        return COVER_ERR_NOT_CALIBRATED;
    }
    if (sm->pos_q == COVER_POS_UNKNOWN) {
        return COVER_ERR_POS_UNKNOWN;
    }
    uint16_t diff = target > sm->pos ? target - sm->pos : sm->pos - target;
    if (diff <= sm->cfg.goto_tolerance) {
        if (is_active(sm)) {
            stop_run(sm);
            return COVER_OK;
        }
        return COVER_OK_NOOP;
    }
    cover_dir_t dir = target < sm->pos ? COVER_DIR_OPEN : COVER_DIR_CLOSE;
    request_run(sm, dir, target, false);
    return COVER_OK;
}

cover_result_t cover_sm_command(cover_sm_t *sm, const cover_cmd_t *cmd, uint32_t now_ms)
{
    if (!cmd) {
        return COVER_ERR_INVALID;
    }
    sm->now = now_ms;

    cover_cmd_type_t type = cmd->type;
    if ((type == COVER_CMD_BUTTON_OPEN || type == COVER_CMD_BUTTON_CLOSE) && is_active(sm)) {
        type = COVER_CMD_STOP; /* local buttons: any press while moving stops */
    }

    if (type == COVER_CMD_STOP) {
        return do_stop(sm);
    }

    /* Every other command moves the motor. */
    if (sm->state == COVER_ST_FAULT) {
        return COVER_ERR_FAULT;
    }
    if (sm->in.stop_held || sm->in.obstacle) {
        return COVER_ERR_INHIBITED;
    }
    if (sm->cal_phase != COVER_CAL_IDLE) {
        if (type == COVER_CMD_CALIBRATE) {
            return COVER_OK_NOOP;
        }
        abort_calibration(sm);
        stop_run(sm);
        return COVER_ERR_CAL_ABORTED;
    }

    switch (type) {
    case COVER_CMD_OPEN:
    case COVER_CMD_BUTTON_OPEN:
        return do_move_end(sm, COVER_DIR_OPEN);
    case COVER_CMD_CLOSE:
    case COVER_CMD_BUTTON_CLOSE:
        return do_move_end(sm, COVER_DIR_CLOSE);
    case COVER_CMD_GOTO:
        return do_goto(sm, cmd->target);
    case COVER_CMD_CALIBRATE:
        if (!sm->cfg.has_limit_open || !sm->cfg.has_limit_closed) {
            return COVER_ERR_NO_LIMITS;
        }
        sm->cal_phase = COVER_CAL_TO_OPEN;
        sm->events |= COVER_EVT_STATE;
        request_run(sm, COVER_DIR_OPEN, COVER_POS_OPEN, true);
        return COVER_OK;
    default:
        return COVER_ERR_INVALID;
    }
}

cover_result_t cover_sm_set_calibration(cover_sm_t *sm, const cover_calib_t *calib, uint32_t now_ms)
{
    sm->now = now_ms;
    if (calib && (calib->open_ms < sm->cfg.min_travel_ms || calib->close_ms < sm->cfg.min_travel_ms ||
                  calib->open_ms > CAL_MAX_MANUAL_MS || calib->close_ms > CAL_MAX_MANUAL_MS)) {
        return COVER_ERR_INVALID;
    }
    abort_calibration(sm);
    if (sm->state != COVER_ST_FAULT) {
        stop_run(sm);
    }
    if (calib) {
        sm->calib = *calib;
        sm->calibrated = true;
    } else {
        memset(&sm->calib, 0, sizeof(sm->calib));
        sm->calibrated = false;
        if (sm->pos_q == COVER_POS_ESTIMATED) {
            set_pos(sm, sm->pos, COVER_POS_UNKNOWN);
        }
    }
    sm->events |= COVER_EVT_CALIB | COVER_EVT_STATE;
    if (sm->state != COVER_ST_FAULT) {
        set_state(sm, idle_state(sm));
    }
    return COVER_OK;
}

cover_dir_t cover_sm_motor(const cover_sm_t *sm)
{
    return sm->motor;
}

uint32_t cover_sm_take_events(cover_sm_t *sm)
{
    uint32_t e = sm->events;
    sm->events = 0;
    return e;
}

void cover_sm_get_status(const cover_sm_t *sm, cover_status_t *out)
{
    memset(out, 0, sizeof(*out));
    out->state = sm->state;
    out->dir = sm->motor != COVER_DIR_NONE ? sm->motor : sm->run_dir;
    out->fault = sm->fault;
    out->pos_quality = sm->pos_q;
    out->pos = sm->pos;
    out->target_valid = sm->run_dir != COVER_DIR_NONE;
    out->target = sm->run_to_end ? end_pos(sm->run_dir) : sm->run_target;
    out->calibrated = sm->calibrated;
    out->calib = sm->calib;
    out->cal_phase = sm->cal_phase;
    out->inputs = sm->in;
}

const char *cover_state_str(cover_state_t s)
{
    switch (s) {
    case COVER_ST_UNCALIBRATED: return "UNCALIBRATED";
    case COVER_ST_STOPPED:      return "STOPPED";
    case COVER_ST_OPENING:      return "OPENING";
    case COVER_ST_CLOSING:      return "CLOSING";
    case COVER_ST_DEADTIME:     return "DEADTIME";
    case COVER_ST_FAULT:        return "FAULT";
    }
    return "?";
}

const char *cover_fault_str(cover_fault_t f)
{
    switch (f) {
    case COVER_FAULT_NONE:        return "none";
    case COVER_FAULT_LIMIT_BOTH:  return "both limit switches active";
    case COVER_FAULT_LIMIT_WRONG: return "opposite limit switch hit";
    case COVER_FAULT_TIMEOUT:     return "run timeout";
    case COVER_FAULT_OBSTACLE:    return "obstacle";
    case COVER_FAULT_DRIVER:      return "driver fault input";
    case COVER_FAULT_OUTPUT:      return "output driver error";
    }
    return "?";
}

const char *cover_result_str(cover_result_t r)
{
    switch (r) {
    case COVER_OK:                 return "ok";
    case COVER_OK_NOOP:            return "ok (nothing to do)";
    case COVER_ERR_FAULT:          return "rejected: fault latched (send STOP to clear)";
    case COVER_ERR_FAULT_ACTIVE:   return "rejected: fault cause still present";
    case COVER_ERR_INHIBITED:      return "rejected: stop input or obstacle active";
    case COVER_ERR_NOT_CALIBRATED: return "rejected: not calibrated";
    case COVER_ERR_POS_UNKNOWN:    return "rejected: position unknown (fully open or close first)";
    case COVER_ERR_NO_LIMITS:      return "rejected: needs both limit switches";
    case COVER_ERR_INVALID:        return "rejected: invalid argument";
    case COVER_ERR_CAL_ABORTED:    return "calibration aborted";
    }
    return "?";
}

const char *cover_pos_quality_str(cover_pos_quality_t q)
{
    switch (q) {
    case COVER_POS_UNKNOWN:   return "unknown";
    case COVER_POS_ESTIMATED: return "estimated";
    case COVER_POS_AT_LIMIT:  return "limit switch";
    }
    return "?";
}
