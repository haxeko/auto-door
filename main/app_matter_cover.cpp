/*
 * Window Covering endpoint for the curtain profile.
 *
 * Command path (controller -> motor):
 *   Matter thread: upstream window-covering-server decodes UpOrOpen / DownOrClose /
 *   GoToLiftPercentage, writes TargetPositionLiftPercent100ths and calls our
 *   Delegate::HandleMovement(). We only read the target and post a command to the
 *   cover controller queue (non-blocking). StopMotion -> HandleStopMotion() -> STOP
 *   posted to the FRONT of the queue.
 *
 * Status path (motor -> controller):
 *   control task -> app_matter_cover_status_cb() copies a snapshot and schedules
 *   sync_work() on the Matter thread with PlatformMgr().ScheduleWork(). sync_work()
 *   runs with the Matter stack lock held and writes the attributes.
 *
 * No loop: attribute writes never call the delegate, and the application
 * attribute callback ignores the Window Covering cluster.
 *
 * Target vs current: TargetPositionLiftPercent100ths is what the motor is heading
 * for (the controller request while it is being executed); once stopped it is set
 * equal to the current position (as StopMotion does upstream), or null if unknown.
 * CurrentPositionLiftPercent100ths is the state machine's position: measured only at
 * a limit switch, otherwise a time based estimate (ConfigStatus.LiftEncoderControlled
 * is deliberately left 0), null when unknown.
 */
#include "app_matter_cover.h"

#include <atomic>

#include <esp_log.h>
#include <esp_matter.h>
#include <freertos/FreeRTOS.h>

#include <app-common/zap-generated/attributes/Accessors.h>
#include <app/clusters/window-covering-server/window-covering-server.h>
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <platform/CHIPDeviceLayer.h>

#include "sdkconfig.h"

static const char *TAG = "matter_cover";

using namespace esp_matter;
using namespace chip::app::Clusters;
namespace WC = chip::app::Clusters::WindowCovering;
using chip::Protocols::InteractionModel::Status;

#if CONFIG_COVER_PRODUCT_CURTAIN_CENTRAL
#define COVER_TYPE WC::Type::kDrapery
#define COVER_END_PRODUCT WC::EndProductType::kCentralCurtain
#elif CONFIG_COVER_PRODUCT_CURTAIN_LEFT
#define COVER_TYPE WC::Type::kDrapery
#define COVER_END_PRODUCT WC::EndProductType::kLateralLeftCurtain
#elif CONFIG_COVER_PRODUCT_CURTAIN_RIGHT
#define COVER_TYPE WC::Type::kDrapery
#define COVER_END_PRODUCT WC::EndProductType::kLateralRightCurtain
#elif CONFIG_COVER_PRODUCT_ROLLER_SHUTTER
#define COVER_TYPE WC::Type::kShutter
#define COVER_END_PRODUCT WC::EndProductType::kRollerShutter
#else
#define COVER_TYPE WC::Type::kRollerShade
#define COVER_END_PRODUCT WC::EndProductType::kRollerShade
#endif

static uint16_t s_endpoint = chip::kInvalidEndpointId;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static cover_status_t s_pending;
static bool s_have_pending;
static std::atomic<bool> s_scheduled{false};
static std::atomic<bool> s_matter_started{false};

/* ---------------------------------------------------------------- delegate */

class CoverDelegate : public WC::Delegate {
public:
    CHIP_ERROR HandleMovement(WC::WindowCoveringType type) override
    {
        if (type != WC::WindowCoveringType::Lift) {
            return CHIP_ERROR_NOT_IMPLEMENTED; /* no tilt on this device */
        }
        WC::NPercent100ths target;
        if (WC::Attributes::TargetPositionLiftPercent100ths::Get(mEndpoint, target) != Status::Success ||
            target.IsNull()) {
            return CHIP_ERROR_INVALID_ARGUMENT;
        }
        cover_cmd_t cmd = {.type = COVER_CMD_GOTO, .source = COVER_SRC_MATTER, .target = target.Value()};
        return cover_controller_post(&cmd) == ESP_OK ? CHIP_NO_ERROR : CHIP_ERROR_NO_MEMORY;
    }

    CHIP_ERROR HandleStopMotion() override
    {
        cover_cmd_t cmd = {.type = COVER_CMD_STOP, .source = COVER_SRC_MATTER, .target = 0};
        return cover_controller_post(&cmd) == ESP_OK ? CHIP_NO_ERROR : CHIP_ERROR_NO_MEMORY;
    }
};

static CoverDelegate s_delegate;

/* ------------------------------------------------------------- status sync */

static chip::BitMask<WC::SafetyStatus> safety_bits(const cover_status_t &st)
{
    chip::BitMask<WC::SafetyStatus> b;
    if (st.inputs.obstacle || st.fault == COVER_FAULT_OBSTACLE) {
        b.Set(WC::SafetyStatus::kObstacleDetected);
    }
    if (st.inputs.stop_held) {
        b.Set(WC::SafetyStatus::kStopInput);
    }
    switch (st.fault) {
    case COVER_FAULT_TIMEOUT:
        b.Set(WC::SafetyStatus::kMotorJammed);
        b.Set(WC::SafetyStatus::kPositionFailure);
        break;
    case COVER_FAULT_LIMIT_BOTH:
    case COVER_FAULT_LIMIT_WRONG:
        b.Set(WC::SafetyStatus::kPositionFailure);
        b.Set(WC::SafetyStatus::kHardwareFailure);
        break;
    case COVER_FAULT_DRIVER:
    case COVER_FAULT_OUTPUT:
        b.Set(WC::SafetyStatus::kHardwareFailure);
        break;
    default:
        break;
    }
    if (st.inputs.driver_fault) {
        b.Set(WC::SafetyStatus::kHardwareFailure);
    }
    return b;
}

static void sync_work(intptr_t)
{
    s_scheduled.store(false);
    cover_status_t st;
    portENTER_CRITICAL(&s_lock);
    if (!s_have_pending) {
        portEXIT_CRITICAL(&s_lock);
        return;
    }
    st = s_pending;
    s_have_pending = false;
    portEXIT_CRITICAL(&s_lock);

    const uint16_t ep = s_endpoint;
    WC::NPercent100ths current;
    if (st.pos_quality != COVER_POS_UNKNOWN) {
        current.SetNonNull(st.pos);
    }
    bool active = st.state == COVER_ST_OPENING || st.state == COVER_ST_CLOSING || st.state == COVER_ST_DEADTIME;
    WC::NPercent100ths target = current;
    if (active && st.target_valid) {
        target.SetNonNull(st.target);
    }

    WC::OperationalState op = WC::OperationalState::Stall;
    if (active) {
        op = st.dir == COVER_DIR_OPEN ? WC::OperationalState::MovingUpOrOpen : WC::OperationalState::MovingDownOrClose;
    }

    /* Order matters: current, target, then OperationalStatus so that the upstream
     * PostAttributeChange() bookkeeping is overridden by the real motor state. */
    WC::Attributes::CurrentPositionLiftPercent100ths::Set(ep, current);
    WC::Attributes::TargetPositionLiftPercent100ths::Set(ep, target);
    WC::OperationalStateSet(ep, WC::OperationalStatus::kLift, op);
    WC::Attributes::SafetyStatus::Set(ep, safety_bits(st));
}

static void schedule_sync()
{
    if (!s_matter_started.load() || s_endpoint == chip::kInvalidEndpointId) {
        return;
    }
    bool expected = false;
    if (s_scheduled.compare_exchange_strong(expected, true)) {
        if (chip::DeviceLayer::PlatformMgr().ScheduleWork(sync_work, 0) != CHIP_NO_ERROR) {
            s_scheduled.store(false);
            ESP_LOGW(TAG, "could not schedule attribute sync");
        }
    }
}

void app_matter_cover_status_cb(const cover_status_t *status, void *arg)
{
    (void)arg;
    portENTER_CRITICAL(&s_lock);
    s_pending = *status;
    s_have_pending = true;
    portEXIT_CRITICAL(&s_lock);
    schedule_sync();
}

void app_matter_cover_matter_started()
{
    s_matter_started.store(true);
    /* Overwrite values esp-matter restored from NVS with the real state. */
    cover_status_t st;
    cover_controller_get_status(&st);
    app_matter_cover_status_cb(&st, nullptr);
}

/* ---------------------------------------------------------------- endpoint */

esp_err_t app_matter_cover_create(node_t *node, uint16_t *endpoint_id)
{
    endpoint::window_covering::config_t cfg(chip::to_underlying(COVER_END_PRODUCT));
    cfg.window_covering.type = chip::to_underlying(COVER_TYPE);
    /* Operational + LiftPositionAware. LiftEncoderControlled stays 0: timer based position. */
    cfg.window_covering.config_status =
        chip::to_underlying(WC::ConfigStatus::kOperational) | chip::to_underlying(WC::ConfigStatus::kLiftPositionAware);
    cfg.window_covering.mode = 0;
    cfg.window_covering.feature_flags =
        cluster::window_covering::feature::lift::get_id() | cluster::window_covering::feature::position_aware_lift::get_id();
    cfg.window_covering.features.position_aware_lift.current_position_lift_percent_100ths = nullable<uint16_t>();
    cfg.window_covering.features.position_aware_lift.target_position_lift_percent_100ths = nullable<uint16_t>();
    cfg.window_covering.delegate = &s_delegate;

    endpoint_t *ep = endpoint::window_covering::create(node, &cfg, ENDPOINT_FLAG_NONE, nullptr);
    if (!ep) {
        ESP_LOGE(TAG, "failed to create window covering endpoint");
        return ESP_FAIL;
    }
    cluster_t *cl = cluster::get(ep, WC::Id);
    if (!cl || !cluster::window_covering::attribute::create_safety_status(cl, 0)) {
        ESP_LOGE(TAG, "failed to add SafetyStatus");
        return ESP_FAIL;
    }
    /* CurrentPosition is non-volatile in esp-matter: batch NVS writes while moving. */
    attribute_t *cur = attribute::get(cl, WC::Attributes::CurrentPositionLiftPercent100ths::Id);
    if (cur) {
        attribute::set_deferred_persistence(cur);
    }

    s_endpoint = endpoint::get_id(ep);
    s_delegate.SetEndpoint(s_endpoint);
    *endpoint_id = s_endpoint;
    ESP_LOGI(TAG, "Window Covering endpoint %u (Lift, PositionAwareLift; 0%% = open, 100%% = closed)", s_endpoint);
    return ESP_OK;
}

/* ------------------------------------------------------ commissioning window */

static void open_cw_work(intptr_t arg)
{
    const auto timeout = chip::System::Clock::Seconds32(static_cast<uint32_t>(arg));
    auto &mgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    if (mgr.IsCommissioningWindowOpen()) {
        ESP_LOGI(TAG, "commissioning window already open");
        return;
    }
    bool commissioned = chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;
    /* Not commissioned: BLE + on-network. Commissioned: the device is on Wi-Fi, use DNS-SD. */
    auto adv = commissioned ? chip::CommissioningWindowAdvertisement::kDnssdOnly
                            : chip::CommissioningWindowAdvertisement::kAllSupported;
    CHIP_ERROR err = mgr.OpenBasicCommissioningWindow(timeout, adv);
    if (err != CHIP_NO_ERROR && adv == chip::CommissioningWindowAdvertisement::kAllSupported) {
        /* BLE may have been de-initialised after a previous commissioning. */
        err = mgr.OpenBasicCommissioningWindow(timeout, chip::CommissioningWindowAdvertisement::kDnssdOnly);
    }
    if (err != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "open commissioning window failed: %" CHIP_ERROR_FORMAT, err.Format());
    } else {
        ESP_LOGI(TAG, "commissioning window open for %u s (%s)", static_cast<unsigned>(arg),
                 commissioned ? "on-network" : "BLE + on-network");
    }
}

void app_matter_open_commissioning_window(uint16_t timeout_s)
{
    if (!s_matter_started.load()) {
        ESP_LOGW(TAG, "Matter not started yet");
        return;
    }
    chip::DeviceLayer::PlatformMgr().ScheduleWork(open_cw_work, static_cast<intptr_t>(timeout_s));
}
