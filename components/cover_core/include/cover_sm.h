/*
 * Cover (curtain / blind) motion state machine.
 *
 * Pure C, no ESP-IDF / FreeRTOS dependency, so it can be unit tested on the
 * host (see test/host). The caller owns time (milliseconds, free running,
 * wrap-around safe) and feeds debounced inputs every control period.
 *
 * Position convention follows the Matter Window Covering cluster:
 *   0      = fully open   (WC_PERCENT100THS_MIN_OPEN)
 *   10000  = fully closed (WC_PERCENT100THS_MAX_CLOSED)
 * Values are in 1/100 %.
 *
 * Without an encoder the mid-travel position is a time based ESTIMATE.
 * Only "at limit switch" positions are measured (COVER_POS_AT_LIMIT).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COVER_POS_OPEN    0u
#define COVER_POS_CLOSED  10000u

typedef enum {
    COVER_DIR_NONE = 0,
    COVER_DIR_OPEN,
    COVER_DIR_CLOSE,
} cover_dir_t;

typedef enum {
    COVER_ST_UNCALIBRATED = 0, /* motor off, no valid travel times                 */
    COVER_ST_STOPPED,          /* motor off, calibrated                            */
    COVER_ST_OPENING,          /* motor driving towards open                       */
    COVER_ST_CLOSING,          /* motor driving towards closed                     */
    COVER_ST_DEADTIME,         /* motor off, waiting dead time before a new run    */
    COVER_ST_FAULT,            /* motor off, latched fault, needs STOP to clear    */
} cover_state_t;

typedef enum {
    COVER_FAULT_NONE = 0,
    COVER_FAULT_LIMIT_BOTH,     /* both limit switches active at the same time       */
    COVER_FAULT_LIMIT_WRONG,    /* opposite limit switch hit while moving (wiring?)  */
    COVER_FAULT_TIMEOUT,        /* run exceeded the maximum allowed time             */
    COVER_FAULT_OBSTACLE,       /* obstacle input became active while moving         */
    COVER_FAULT_DRIVER,         /* power stage reported a fault                      */
    COVER_FAULT_OUTPUT,         /* output driver error (interlock / read back)       */
} cover_fault_t;

typedef enum {
    COVER_POS_UNKNOWN = 0,      /* no idea where the cover is                        */
    COVER_POS_ESTIMATED,        /* time based estimate (or restored after reboot)    */
    COVER_POS_AT_LIMIT,         /* confirmed by a limit switch                       */
} cover_pos_quality_t;

typedef enum {
    COVER_CAL_IDLE = 0,
    COVER_CAL_TO_OPEN,          /* moving to the open limit (reference point)        */
    COVER_CAL_MEASURE_CLOSE,    /* timing open -> closed                             */
    COVER_CAL_MEASURE_OPEN,     /* timing closed -> open                             */
} cover_cal_phase_t;

typedef enum {
    COVER_SRC_INTERNAL = 0,
    COVER_SRC_BUTTON,
    COVER_SRC_MATTER,
    COVER_SRC_CONSOLE,
} cover_src_t;

typedef enum {
    COVER_CMD_STOP = 0,
    COVER_CMD_OPEN,             /* go to fully open                                  */
    COVER_CMD_CLOSE,            /* go to fully closed                                */
    COVER_CMD_GOTO,             /* go to `target` (Matter percent100ths)             */
    COVER_CMD_BUTTON_OPEN,      /* local button: stop if moving, else open           */
    COVER_CMD_BUTTON_CLOSE,     /* local button: stop if moving, else close          */
    COVER_CMD_CALIBRATE,        /* automatic calibration with both limit switches    */
} cover_cmd_type_t;

typedef struct {
    cover_cmd_type_t type;
    cover_src_t source;
    uint16_t target;            /* only for COVER_CMD_GOTO                           */
} cover_cmd_t;

typedef enum {
    COVER_OK = 0,
    COVER_OK_NOOP,              /* accepted, nothing to do (already there)           */
    COVER_ERR_FAULT,            /* a fault is latched                                */
    COVER_ERR_FAULT_ACTIVE,     /* STOP cannot clear: cause still present            */
    COVER_ERR_INHIBITED,        /* stop input held / obstacle present                */
    COVER_ERR_NOT_CALIBRATED,   /* mid position requested without calibration        */
    COVER_ERR_POS_UNKNOWN,      /* mid position requested, position unknown          */
    COVER_ERR_NO_LIMITS,        /* calibration needs both limit switches             */
    COVER_ERR_INVALID,          /* bad argument                                      */
    COVER_ERR_CAL_ABORTED,      /* command aborted a running calibration             */
} cover_result_t;

/* Event bits returned by cover_sm_take_events() */
#define COVER_EVT_STATE      (1u << 0)  /* state / direction / fault changed         */
#define COVER_EVT_POSITION   (1u << 1)  /* position or quality changed               */
#define COVER_EVT_TARGET     (1u << 2)  /* target changed                            */
#define COVER_EVT_RUN_START  (1u << 3)  /* motor energised (persist "moving")         */
#define COVER_EVT_RUN_STOP   (1u << 4)  /* motor stopped (persist position)          */
#define COVER_EVT_CALIB      (1u << 5)  /* calibration data changed (persist)        */
#define COVER_EVT_CAL_FAILED (1u << 6)  /* calibration aborted / implausible         */

/* Debounced inputs, sampled every control period. Absent inputs are false. */
typedef struct {
    bool limit_open;
    bool limit_closed;
    bool obstacle;
    bool driver_fault;
    bool stop_held;             /* local STOP button currently held                  */
    bool output_fault;          /* motor driver detected an output problem           */
} cover_sm_inputs_t;

typedef struct {
    bool has_limit_open;        /* hardware has an open limit switch                 */
    bool has_limit_closed;      /* hardware has a closed limit switch                */
    uint32_t deadtime_ms;       /* motor off time before any (re)start               */
    uint32_t max_run_uncal_ms;  /* run limit when travel times are unknown           */
    uint32_t timeout_margin_pct;/* run limit = travel * (100 + margin) / 100         */
    uint32_t end_overrun_pct;   /* extra run to an end WITHOUT limit switch          */
    uint32_t min_travel_ms;     /* calibration plausibility lower bound              */
    uint16_t pos_report_step;   /* POSITION event granularity while moving (1/100%)  */
    uint16_t goto_tolerance;    /* |target - pos| below this is a no-op (1/100%)    */
} cover_sm_config_t;

typedef struct {
    uint32_t open_ms;           /* full travel closed -> open                        */
    uint32_t close_ms;          /* full travel open -> closed                        */
} cover_calib_t;

/* Read-only snapshot for reporting (Matter, console, logs). */
typedef struct {
    cover_state_t state;
    cover_dir_t dir;            /* running or pending direction                      */
    cover_fault_t fault;
    cover_pos_quality_t pos_quality;
    uint16_t pos;               /* valid if pos_quality != COVER_POS_UNKNOWN         */
    bool target_valid;
    uint16_t target;            /* where the current run is heading                  */
    bool calibrated;
    cover_calib_t calib;
    cover_cal_phase_t cal_phase;
    cover_sm_inputs_t inputs;
} cover_status_t;

typedef struct {
    /* configuration */
    cover_sm_config_t cfg;
    cover_calib_t calib;
    bool calibrated;

    /* state */
    cover_state_t state;
    cover_fault_t fault;
    cover_sm_inputs_t in;
    cover_sm_inputs_t in_prev;

    /* position */
    uint16_t pos;
    cover_pos_quality_t pos_q;
    uint16_t pos_reported;

    /* current or pending run */
    cover_dir_t run_dir;
    uint16_t run_target;
    bool run_to_end;            /* run until limit switch / end of travel           */
    uint32_t run_t_start;
    uint16_t run_pos_start;
    cover_pos_quality_t run_pos_q_start;
    uint32_t run_t_max;

    /* motor output and dead time */
    cover_dir_t motor;
    uint32_t t_motor_off;

    /* calibration */
    cover_cal_phase_t cal_phase;
    uint32_t cal_t_close;

    uint32_t events;
    uint32_t now;
} cover_sm_t;

/**
 * Initialise. Motor is always OFF after init (no automatic motion on boot).
 *
 * @param calib       stored calibration or NULL
 * @param restored_pos position restored from storage, or < 0 if unknown
 */
void cover_sm_init(cover_sm_t *sm, const cover_sm_config_t *cfg, const cover_calib_t *calib,
                   int32_t restored_pos, const cover_sm_inputs_t *inputs, uint32_t now_ms);

/** Periodic update with fresh debounced inputs. Call before cover_sm_command(). */
void cover_sm_tick(cover_sm_t *sm, const cover_sm_inputs_t *inputs, uint32_t now_ms);

/** Handle a command. Never blocks. */
cover_result_t cover_sm_command(cover_sm_t *sm, const cover_cmd_t *cmd, uint32_t now_ms);

/** Desired motor output. Never OPEN and CLOSE at the same time by construction. */
cover_dir_t cover_sm_motor(const cover_sm_t *sm);

/** Install / clear calibration (e.g. manual entry). Stops motion. */
cover_result_t cover_sm_set_calibration(cover_sm_t *sm, const cover_calib_t *calib, uint32_t now_ms);

/** Fetch and clear pending COVER_EVT_* bits. */
uint32_t cover_sm_take_events(cover_sm_t *sm);

void cover_sm_get_status(const cover_sm_t *sm, cover_status_t *out);

const char *cover_state_str(cover_state_t s);
const char *cover_fault_str(cover_fault_t f);
const char *cover_result_str(cover_result_t r);
const char *cover_pos_quality_str(cover_pos_quality_t q);

#ifdef __cplusplus
}
#endif
