#include "sim_plant.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "motor_driver.h"

static const char *TAG = "sim";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

void sim_plant_init(sim_plant_t *p, uint32_t open_ms, uint32_t close_ms, bool has_limit_open, bool has_limit_closed,
                    int32_t start_pos)
{
    memset(p, 0, sizeof(*p));
    p->open_ms = open_ms ? open_ms : 1;
    p->close_ms = close_ms ? close_ms : 1;
    p->has_limit_open = has_limit_open;
    p->has_limit_closed = has_limit_closed;
    p->pos = start_pos;
    p->force_limit_open = -1;
    p->force_limit_closed = -1;
}

void sim_plant_step(sim_plant_t *p, uint32_t dt_ms)
{
    taskENTER_CRITICAL(&s_lock);
    if (p->motor != COVER_DIR_NONE && !p->jammed) {
        /* thousandths of a 1/100 % to avoid drift with short ticks */
        int64_t rate = p->motor == COVER_DIR_OPEN ? -(10000LL * 1000) / p->open_ms : (10000LL * 1000) / p->close_ms;
        p->acc_x1000 += (int32_t)(rate * dt_ms);
        int32_t d = p->acc_x1000 / 1000;
        p->acc_x1000 -= d * 1000;
        p->pos += d;
        if (p->pos < 0) {
            p->pos = 0;
        }
        if (p->pos > 10000) {
            p->pos = 10000;
        }
    }
    taskEXIT_CRITICAL(&s_lock);
}

bool sim_plant_limit_open(const sim_plant_t *p)
{
    if (p->force_limit_open >= 0) {
        return p->force_limit_open;
    }
    return p->has_limit_open && p->pos <= 0;
}

bool sim_plant_limit_closed(const sim_plant_t *p)
{
    if (p->force_limit_closed >= 0) {
        return p->force_limit_closed;
    }
    return p->has_limit_closed && p->pos >= 10000;
}

#define SIM_SET(field, val)            \
    do {                               \
        taskENTER_CRITICAL(&s_lock);   \
        p->field = (val);              \
        taskEXIT_CRITICAL(&s_lock);    \
    } while (0)

void sim_plant_set_obstacle(sim_plant_t *p, bool on) { SIM_SET(obstacle, on); }
void sim_plant_set_driver_fault(sim_plant_t *p, bool on) { SIM_SET(driver_fault, on); }
void sim_plant_set_jammed(sim_plant_t *p, bool on) { SIM_SET(jammed, on); }
void sim_plant_set_pos(sim_plant_t *p, int32_t pos) { SIM_SET(pos, pos < 0 ? 0 : (pos > 10000 ? 10000 : pos)); }

void sim_plant_force_limits(sim_plant_t *p, int8_t open, int8_t closed)
{
    taskENTER_CRITICAL(&s_lock);
    p->force_limit_open = open;
    p->force_limit_closed = closed;
    taskEXIT_CRITICAL(&s_lock);
}

/* ------------------------------------------------------------ sim driver --- */

typedef struct {
    motor_driver_t base;
    sim_plant_t *plant;
    motor_interlock_t il;
    bool fault;
} sim_drv_t;

static esp_err_t sim_set(motor_driver_t *self, cover_dir_t dir, uint32_t now_ms)
{
    sim_drv_t *c = (sim_drv_t *)self->ctx;
    if (dir == c->il.current) {
        return ESP_OK;
    }
    taskENTER_CRITICAL(&s_lock);
    c->plant->motor = COVER_DIR_NONE;
    taskEXIT_CRITICAL(&s_lock);
    motor_interlock_commit(&c->il, COVER_DIR_NONE, now_ms);
    if (dir == COVER_DIR_NONE) {
        ESP_LOGI(TAG, "[SIM] motor OFF (pos %ld)", (long)c->plant->pos);
        return ESP_OK;
    }
    if (c->fault || motor_interlock_check(&c->il, dir, now_ms) != ESP_OK) {
        c->fault = true;
        ESP_LOGE(TAG, "[SIM] interlock refused %s", dir == COVER_DIR_OPEN ? "OPEN" : "CLOSE");
        return ESP_ERR_INVALID_STATE;
    }
    taskENTER_CRITICAL(&s_lock);
    c->plant->motor = dir;
    taskEXIT_CRITICAL(&s_lock);
    motor_interlock_commit(&c->il, dir, now_ms);
    ESP_LOGI(TAG, "[SIM] motor %s (no GPIO driven)", dir == COVER_DIR_OPEN ? "OPEN" : "CLOSE");
    return ESP_OK;
}

static bool sim_output_fault(motor_driver_t *self)
{
    return ((sim_drv_t *)self->ctx)->fault;
}

esp_err_t motor_driver_sim_create(sim_plant_t *plant, uint32_t deadtime_ms, uint32_t now_ms, motor_driver_t **out)
{
    sim_drv_t *c = calloc(1, sizeof(*c));
    if (!c) {
        return ESP_ERR_NO_MEM;
    }
    c->plant = plant;
    motor_interlock_init(&c->il, deadtime_ms, now_ms);
    c->base.name = "sim";
    c->base.set = sim_set;
    c->base.output_fault = sim_output_fault;
    c->base.ctx = c;
    *out = &c->base;
    return ESP_OK;
}
