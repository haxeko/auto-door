/*
 * Console commands: `matter esp cover <sub-command>`.
 * Useful in simulation mode to drive the state machine and inject faults.
 */
#include "app_console.h"

#include <stdlib.h>
#include <string.h>

#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_console.h>

#include "app_matter_cover.h"
#include "cover_controller.h"
#include "cover_storage.h"
#include "sdkconfig.h"

static const char *TAG = "cover_cli";

static void print_status()
{
    cover_status_t st;
    cover_controller_get_status(&st);
    printf("state      : %s%s\n", cover_state_str(st.state),
           st.cal_phase != COVER_CAL_IDLE ? " (calibrating)" : "");
    printf("fault      : %s\n", cover_fault_str(st.fault));
    if (st.pos_quality == COVER_POS_UNKNOWN) {
        printf("position   : unknown\n");
    } else {
        printf("position   : %u.%02u %% closed (%s) [Matter: 0 = open, 100 = closed]\n", st.pos / 100, st.pos % 100,
               cover_pos_quality_str(st.pos_quality));
    }
    if (st.target_valid) {
        printf("target     : %u.%02u %% closed\n", st.target / 100, st.target % 100);
    } else {
        printf("target     : none\n");
    }
    if (st.calibrated) {
        printf("calibration: open %u ms, close %u ms\n", (unsigned)st.calib.open_ms, (unsigned)st.calib.close_ms);
    } else {
        printf("calibration: none (mid positions disabled)\n");
    }
    printf("inputs     : limit_open=%d limit_closed=%d obstacle=%d driver_fault=%d stop_held=%d output_fault=%d\n",
           st.inputs.limit_open, st.inputs.limit_closed, st.inputs.obstacle, st.inputs.driver_fault,
           st.inputs.stop_held, st.inputs.output_fault);
    sim_plant_t *sim = cover_controller_sim();
    if (sim) {
        printf("[SIM] true position %ld, motor %d, jammed=%d\n", (long)sim->pos, sim->motor, sim->jammed);
    }
}

static esp_err_t post(cover_cmd_type_t type, uint16_t target = 0)
{
    cover_cmd_t c = {.type = type, .source = COVER_SRC_CONSOLE, .target = target};
    return cover_controller_post(&c);
}

static bool parse_bool(const char *s)
{
    return s && (strcmp(s, "1") == 0 || strcmp(s, "on") == 0);
}

static esp_err_t cmd_sim(int argc, char **argv)
{
    sim_plant_t *sim = cover_controller_sim();
    if (!sim) {
        printf("not in simulation mode\n");
        return ESP_ERR_INVALID_STATE;
    }
    if (argc < 2) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!strcmp(argv[0], "obstacle")) {
        sim_plant_set_obstacle(sim, parse_bool(argv[1]));
    } else if (!strcmp(argv[0], "fault")) {
        sim_plant_set_driver_fault(sim, parse_bool(argv[1]));
    } else if (!strcmp(argv[0], "jam")) {
        sim_plant_set_jammed(sim, parse_bool(argv[1]));
    } else if (!strcmp(argv[0], "pos")) {
        sim_plant_set_pos(sim, atoi(argv[1]));
    } else if (!strcmp(argv[0], "limits")) {
        const char *m = argv[1];
        if (!strcmp(m, "auto")) {
            sim_plant_force_limits(sim, -1, -1);
        } else if (!strcmp(m, "both")) {
            sim_plant_force_limits(sim, 1, 1);
        } else if (!strcmp(m, "none")) {
            sim_plant_force_limits(sim, 0, 0);
        } else if (!strcmp(m, "open")) {
            sim_plant_force_limits(sim, 1, 0);
        } else if (!strcmp(m, "closed")) {
            sim_plant_force_limits(sim, 0, 1);
        } else {
            return ESP_ERR_INVALID_ARG;
        }
    } else {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "[SIM] %s %s", argv[0], argv[1]);
    return ESP_OK;
}

static esp_err_t cover_handler(int argc, char **argv)
{
    if (argc < 1 || !strcmp(argv[0], "help")) {
        printf("matter esp cover status\n"
               "matter esp cover open | close | stop\n"
               "matter esp cover goto <0-100>      (percent CLOSED, Matter convention)\n"
               "matter esp cover btn open|close|stop (virtual local button press)\n"
               "matter esp cover cal start          (automatic, needs both limit switches)\n"
               "matter esp cover cal set <open_ms> <close_ms>\n"
               "matter esp cover cal clear\n"
               "matter esp cover commission         (open commissioning window)\n"
               "matter esp cover factory-reset      (stops motor, then Matter factory reset)\n"
               "matter esp cover sim obstacle|fault|jam 0|1\n"
               "matter esp cover sim limits auto|both|none|open|closed\n"
               "matter esp cover sim pos <0-10000>\n");
        return ESP_OK;
    }
    const char *c = argv[0];
    if (!strcmp(c, "status")) {
        print_status();
        return ESP_OK;
    }
    if (!strcmp(c, "open")) {
        return post(COVER_CMD_OPEN);
    }
    if (!strcmp(c, "close")) {
        return post(COVER_CMD_CLOSE);
    }
    if (!strcmp(c, "stop")) {
        return post(COVER_CMD_STOP);
    }
    if (!strcmp(c, "goto") && argc >= 2) {
        float pct = strtof(argv[1], nullptr);
        if (pct < 0 || pct > 100) {
            return ESP_ERR_INVALID_ARG;
        }
        return post(COVER_CMD_GOTO, static_cast<uint16_t>(pct * 100.0f + 0.5f));
    }
    if (!strcmp(c, "btn") && argc >= 2) {
        if (!strcmp(argv[1], "open")) {
            return post(COVER_CMD_BUTTON_OPEN);
        }
        if (!strcmp(argv[1], "close")) {
            return post(COVER_CMD_BUTTON_CLOSE);
        }
        if (!strcmp(argv[1], "stop")) {
            return post(COVER_CMD_STOP);
        }
        return ESP_ERR_INVALID_ARG;
    }
    if (!strcmp(c, "cal") && argc >= 2) {
        if (!strcmp(argv[1], "start")) {
            return post(COVER_CMD_CALIBRATE);
        }
        if (!strcmp(argv[1], "clear")) {
            return cover_controller_set_calibration(nullptr);
        }
        if (!strcmp(argv[1], "set") && argc >= 4) {
            cover_calib_t cal = {.open_ms = (uint32_t)strtoul(argv[2], nullptr, 10),
                                 .close_ms = (uint32_t)strtoul(argv[3], nullptr, 10)};
            return cover_controller_set_calibration(&cal);
        }
        return ESP_ERR_INVALID_ARG;
    }
    if (!strcmp(c, "commission")) {
        app_matter_open_commissioning_window(CONFIG_COVER_COMMISSIONING_WINDOW_S);
        return ESP_OK;
    }
    if (!strcmp(c, "factory-reset")) {
        cover_controller_halt(1000);
#if CONFIG_COVER_FACTORY_RESET_CLEARS_CALIBRATION
        cover_storage_erase_all();
#endif
        return esp_matter::factory_reset();
    }
    if (!strcmp(c, "sim")) {
        return cmd_sim(argc - 1, &argv[1]);
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t app_console_register()
{
    static const esp_matter::console::command_t cmd = {
        .name = "cover",
        .description = "Cover controller. Usage: matter esp cover help",
        .handler = cover_handler,
    };
    return esp_matter::console::add_commands(&cmd, 1);
}
