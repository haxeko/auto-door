/*
 * Matter side of the curtain profile: Window Covering endpoint, command
 * delegate and attribute synchronisation.
 */
#pragma once

#include <esp_err.h>
#include <esp_matter.h>

#include "cover_controller.h"

/** Create the Window Covering endpoint (Lift + PositionAwareLift). */
esp_err_t app_matter_cover_create(esp_matter::node_t *node, uint16_t *endpoint_id);

/** Status callback for cover_controller (control task context, non-blocking). */
void app_matter_cover_status_cb(const cover_status_t *status, void *arg);

/** Call once after esp_matter::start(): pushes the real state over persisted values. */
void app_matter_cover_matter_started();

/** Schedules opening of a commissioning window on the Matter thread. */
void app_matter_open_commissioning_window(uint16_t timeout_s);
