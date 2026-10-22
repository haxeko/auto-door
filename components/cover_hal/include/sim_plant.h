/*
 * Simulated cover mechanics for the default (safe) simulation mode.
 * No GPIO is touched. Faults can be injected from the console.
 * All functions are called from the control task only, except the setters
 * which are protected by a spinlock.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cover_sm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sim_plant {
    int32_t pos;            /* true position, 0 = open, 10000 = closed */
    uint32_t open_ms;
    uint32_t close_ms;
    bool has_limit_open;
    bool has_limit_closed;
    cover_dir_t motor;
    /* injected faults */
    bool jammed;
    bool obstacle;
    bool driver_fault;
    int8_t force_limit_open;   /* -1 auto, 0 forced off, 1 forced on */
    int8_t force_limit_closed;
    int32_t acc_x1000;
} sim_plant_t;

void sim_plant_init(sim_plant_t *p, uint32_t open_ms, uint32_t close_ms, bool has_limit_open,
                    bool has_limit_closed, int32_t start_pos);
void sim_plant_step(sim_plant_t *p, uint32_t dt_ms);
bool sim_plant_limit_open(const sim_plant_t *p);
bool sim_plant_limit_closed(const sim_plant_t *p);

/* Thread-safe fault injection (console task). */
void sim_plant_set_obstacle(sim_plant_t *p, bool on);
void sim_plant_set_driver_fault(sim_plant_t *p, bool on);
void sim_plant_set_jammed(sim_plant_t *p, bool on);
void sim_plant_force_limits(sim_plant_t *p, int8_t open, int8_t closed);
void sim_plant_set_pos(sim_plant_t *p, int32_t pos);

#ifdef __cplusplus
}
#endif
