/*
 * Auto-door firmware - curtain / blind profile over Matter (Wi-Fi).
 *
 * Start-up order is chosen for safety:
 *   1. NVS, storage task
 *   2. cover controller: motor outputs forced OFF first, then inputs, then the
 *      control task. The cover never moves on its own after boot.
 *   3. Matter node / Window Covering endpoint, then esp_matter::start()
 *   4. first attribute sync, console
 * The local buttons work from step 2 on, with or without network.
 */
#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_console.h>

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

#include "app_config.h"
#include "app_console.h"
#include "app_matter_cover.h"
#include "cover_controller.h"
#include "cover_storage.h"
#include "sdkconfig.h"

static const char *TAG = "app_main";

using namespace esp_matter;
using namespace chip::app::Clusters;

static bool s_controller_running;

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "IP address changed");
        break;
    case chip::DeviceLayer::DeviceEventType::kWiFiConnectivityChange:
        ESP_LOGI(TAG, "Wi-Fi connectivity: %s",
                 event->WiFiConnectivityChange.Result == chip::DeviceLayer::kConnectivity_Established ? "connected"
                                                                                                     : "lost");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGW(TAG, "Commissioning failed, fail safe timer expired");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed");
        if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
            /* Last fabric gone: Wi-Fi credentials are kept, advertise on DNS-SD only. */
            auto &mgr = chip::Server::GetInstance().GetCommissioningWindowManager();
            if (!mgr.IsCommissioningWindowOpen()) {
                CHIP_ERROR err = mgr.OpenBasicCommissioningWindow(
                    chip::System::Clock::Seconds32(CONFIG_COVER_COMMISSIONING_WINDOW_S),
                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                if (err != CHIP_NO_ERROR) {
                    ESP_LOGE(TAG, "Failed to open commissioning window: %" CHIP_ERROR_FORMAT, err.Format());
                }
            }
        }
        break;
    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        break;
    default:
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    /* Identify must never move the motor. Add an LED here if the board has one. */
    ESP_LOGI(TAG, "Identify: type %u, effect %u, variant %u", type, effect_id, effect_variant);
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    /* Window Covering is driven through its command delegate, not through
     * attribute writes. Ignoring the cluster here prevents feedback loops when
     * the application itself updates position / status attributes. */
    return ESP_OK;
}

static void func_button_cb(cover_func_event_t ev, void *arg)
{
    switch (ev) {
    case COVER_FUNC_OPEN_COMMISSIONING:
        app_matter_open_commissioning_window(CONFIG_COVER_COMMISSIONING_WINDOW_S);
        break;
    case COVER_FUNC_RESET_ARMED:
        ESP_LOGW(TAG, "Release the function button to factory reset");
        break;
    case COVER_FUNC_FACTORY_RESET:
        /* Motor already halted by the controller before this callback. */
#if CONFIG_COVER_FACTORY_RESET_CLEARS_CALIBRATION
        cover_storage_erase_all();
#endif
        esp_matter::factory_reset();
        break;
    }
}

static void start_controller()
{
    cover_controller_config_t cfg;
    esp_err_t err = app_config_build(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Invalid hardware configuration: motor control DISABLED (fix menuconfig)");
        return;
    }
    if (cfg.simulation) {
        ESP_LOGW(TAG, "SIMULATION MODE: no motor output is driven. Use 'matter esp cover help'.");
    }
    err = cover_controller_start(&cfg, app_matter_cover_status_cb, func_button_cb, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cover controller start failed: %s - motor control DISABLED", esp_err_to_name(err));
        return;
    }
    s_controller_running = true;
}

extern "C" void app_main()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase (%s)", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(cover_storage_init());

    start_controller();

    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    if (!node) {
        ESP_LOGE(TAG, "Failed to create Matter node");
        return; /* controller and local buttons keep running */
    }

    uint16_t endpoint_id = 0;
    if (app_matter_cover_create(node, &endpoint_id) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create Window Covering endpoint");
        return;
    }

    err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Matter, err:%d. Local control still works.", err);
        return;
    }
    if (s_controller_running) {
        app_matter_cover_matter_started();
    }

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    app_console_register();
    esp_matter::console::init();
#endif
}
