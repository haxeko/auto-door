#include "cover_storage.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

static const char *TAG = "storage";

#define NS          "cover"
#define KEY_CALIB   "calib"
#define KEY_POS     "pos"
#define CALIB_MAGIC 0xC0C1u
#define POS_MAGIC   0xC0D1u
#define FORMAT_VER  1u

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t reserved;
    uint32_t open_ms;
    uint32_t close_ms;
    uint32_t check;
} calib_blob_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t flags; /* bit0 moving, bit1 valid */
    uint16_t pos;
    uint16_t check;
} pos_blob_t;

static TaskHandle_t s_task;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_calib_pending, s_calib_erase;
static cover_calib_t s_calib_req;
static bool s_pos_pending;
static cover_pos_record_t s_pos_req;
static cover_pos_record_t s_pos_stored;
static bool s_pos_stored_known;

static uint32_t calib_check(const calib_blob_t *b)
{
    return (b->open_ms * 2654435761u) ^ (b->close_ms * 40503u) ^ 0xA5A5A5A5u;
}

static uint16_t pos_check(const pos_blob_t *b)
{
    return (uint16_t)((b->pos * 31u) ^ (b->flags * 7u) ^ 0x5A5Au);
}

static bool pos_equal(const cover_pos_record_t *a, const cover_pos_record_t *b)
{
    return a->moving == b->moving && a->valid == b->valid && (!a->valid || a->pos == b->pos);
}

static esp_err_t write_blob(const char *key, const void *data, size_t len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = data ? nvs_set_blob(h, key, data, len) : nvs_erase_key(h, key);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            err = ESP_OK;
        }
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }
    return err;
}

static void storage_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        bool do_calib, erase, do_pos;
        cover_calib_t calib;
        cover_pos_record_t pos;
        taskENTER_CRITICAL(&s_lock);
        do_calib = s_calib_pending;
        erase = s_calib_erase;
        calib = s_calib_req;
        do_pos = s_pos_pending;
        pos = s_pos_req;
        s_calib_pending = s_pos_pending = false;
        taskEXIT_CRITICAL(&s_lock);

        if (do_calib) {
            esp_err_t err;
            if (erase) {
                err = write_blob(KEY_CALIB, NULL, 0);
            } else {
                calib_blob_t b = {.magic = CALIB_MAGIC, .version = FORMAT_VER,
                                  .open_ms = calib.open_ms, .close_ms = calib.close_ms};
                b.check = calib_check(&b);
                err = write_blob(KEY_CALIB, &b, sizeof(b));
            }
            ESP_LOGI(TAG, "calibration %s: %s", erase ? "erased" : "saved", esp_err_to_name(err));
        }
        if (do_pos && !(s_pos_stored_known && pos_equal(&pos, &s_pos_stored))) {
            pos_blob_t b = {.magic = POS_MAGIC, .version = FORMAT_VER,
                            .flags = (uint8_t)((pos.moving ? 1u : 0u) | (pos.valid ? 2u : 0u)),
                            .pos = pos.valid ? pos.pos : 0};
            b.check = pos_check(&b);
            esp_err_t err = write_blob(KEY_POS, &b, sizeof(b));
            if (err == ESP_OK) {
                s_pos_stored = pos;
                s_pos_stored_known = true;
            }
            ESP_LOGD(TAG, "position record (moving=%d valid=%d pos=%u): %s", pos.moving, pos.valid, pos.pos,
                     esp_err_to_name(err));
        }
    }
}

esp_err_t cover_storage_init(void)
{
    if (s_task) {
        return ESP_OK;
    }
    cover_pos_record_t r;
    if (cover_storage_load_pos(&r)) {
        s_pos_stored = r;
        s_pos_stored_known = true;
    }
    BaseType_t ok = xTaskCreate(storage_task, "cover_nvs", 3072, NULL, tskIDLE_PRIORITY + 1, &s_task);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

static bool read_blob(const char *key, void *data, size_t len)
{
    nvs_handle_t h;
    if (nvs_open(NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    size_t sz = len;
    esp_err_t err = nvs_get_blob(h, key, data, &sz);
    nvs_close(h);
    return err == ESP_OK && sz == len;
}

bool cover_storage_load_calib(cover_calib_t *out)
{
    calib_blob_t b;
    if (!read_blob(KEY_CALIB, &b, sizeof(b)) || b.magic != CALIB_MAGIC || b.version != FORMAT_VER ||
        b.check != calib_check(&b)) {
        return false;
    }
    out->open_ms = b.open_ms;
    out->close_ms = b.close_ms;
    return true;
}

bool cover_storage_load_pos(cover_pos_record_t *out)
{
    pos_blob_t b;
    if (!read_blob(KEY_POS, &b, sizeof(b)) || b.magic != POS_MAGIC || b.version != FORMAT_VER ||
        b.check != pos_check(&b) || b.pos > COVER_POS_CLOSED) {
        return false;
    }
    out->moving = b.flags & 1u;
    out->valid = (b.flags & 2u) != 0;
    out->pos = b.pos;
    return true;
}

static void kick(void)
{
    if (s_task) {
        xTaskNotifyGive(s_task);
    }
}

void cover_storage_save_calib(const cover_calib_t *calib)
{
    taskENTER_CRITICAL(&s_lock);
    s_calib_pending = true;
    s_calib_erase = (calib == NULL);
    if (calib) {
        s_calib_req = *calib;
    }
    taskEXIT_CRITICAL(&s_lock);
    kick();
}

void cover_storage_save_pos(const cover_pos_record_t *rec)
{
    taskENTER_CRITICAL(&s_lock);
    s_pos_pending = true;
    s_pos_req = *rec;
    taskEXIT_CRITICAL(&s_lock);
    kick();
}

esp_err_t cover_storage_erase_all(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_erase_all(h);
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }
    return err;
}
