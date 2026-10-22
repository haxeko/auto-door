/*
 * Time based debouncer for buttons and switches. Pure C, host testable.
 *
 * The stable state only changes after the raw level has been constant for
 * `debounce_ms`. Wrap-around safe millisecond time.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool stable;
    bool raw;
    uint32_t t_raw_change;
    uint32_t debounce_ms;
} debounce_t;

static inline void debounce_init(debounce_t *d, bool initial, uint32_t debounce_ms, uint32_t now)
{
    d->stable = initial;
    d->raw = initial;
    d->t_raw_change = now;
    d->debounce_ms = debounce_ms;
}

/** Feed a raw sample. Returns the debounced state. */
static inline bool debounce_update(debounce_t *d, bool raw, uint32_t now)
{
    if (raw != d->raw) {
        d->raw = raw;
        d->t_raw_change = now;
    }
    if (d->stable != d->raw && (uint32_t)(now - d->t_raw_change) >= d->debounce_ms) {
        d->stable = d->raw;
    }
    return d->stable;
}

#ifdef __cplusplus
}
#endif
