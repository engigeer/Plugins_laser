/*

  coolant.c - plugin for for handling laser coolant

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io
  Copyright (c) 2025 Engigeer

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if LASER_COOLANT_ENABLE

#include <string.h>
#include <math.h>

#include "grbl/hal.h"
#include "grbl/override.h"
#include "grbl/state_machine.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"

typedef union {
    uint8_t value;
    struct {
        uint8_t enable : 1;
    };
} coolant_options_t;

typedef struct {
    coolant_options_t options;
    float on_delay;
    float off_delay;
    uint8_t coolant_ok_port;
    uint8_t spindle_link;
} laser_coolant_settings_t;

static uint8_t coolant_ok_port;
static bool coolant_on = false, coolant_off_pending = false;
static laser_coolant_settings_t coolant_settings;
static io_port_cfg_t d_in;
static nvs_address_t nvs_address;

static on_spindle_select_ptr on_spindle_select;
static on_report_options_ptr on_report_options;
static on_realtime_report_ptr on_realtime_report;
static coolant_ptrs_t on_coolant_changed;
static spindle_set_state_ptr on_spindle_set_state;

static void coolant_flood_off (void *data)
{
    coolant_state_t mode = hal.coolant.get_state();
    mode.flood = Off;
    gc_state.modal.coolant = mode;

    on_coolant_changed.set_state(mode);
    coolant_off_pending = coolant_on = false;
    sys.report.coolant = On;
}

static void coolant_lost_handler (uint8_t port, bool state)
{
    if(coolant_on){ // && !coolant_off_pending){

        if (coolant_off_pending){
            task_delete(coolant_flood_off, NULL);
            coolant_off_pending = false;
        }

        if(gc_spindle_get(0)->state.on)
            gc_spindle_off();

        system_set_exec_alarm(Alarm_AbortCycle);

        task_add_immediate(coolant_flood_off, NULL);
        task_add_immediate(report_warning, "Coolant system has turned off unexpectedly.");
    }        
}

// Start/stop tube coolant, wait for ok signal on start if delay is configured.
static void coolantSetState (coolant_state_t mode)
{
    bool changed = mode.flood != hal.coolant.get_state().flood || (mode.flood && coolant_off_pending);

    if(changed && !mode.flood) { //Case handles turning off coolant

        if(gc_spindle_get(0)->state.on) {// && state_get() != STATE_HOLD) {
            mode.flood = On;
            gc_state.modal.coolant = mode; 
            task_add_immediate(report_warning, "Coolant system cannot be disabled while laser is running.");
            on_coolant_changed.set_state(mode); //continue handling chain
            sys.report.coolant = On;
            return;
        }
        // TODO: CHANGE HOW THIS IS HANDLED? NOT SURE IF THIS IS BEHAVING AS EXPECTED RIGHT NOW... 25-09-26
        if(coolant_settings.off_delay > 0.0f && !sys.reset_pending) { //
            mode.flood = On;
            gc_state.modal.coolant = mode; 
            coolant_off_pending = task_add_delayed(coolant_flood_off, NULL, (uint32_t)(coolant_settings.off_delay * 60.0f * 1000.0f));
            on_coolant_changed.set_state(mode); //continue handling chain
            sys.report.coolant = On;
            return;
        }

        coolant_on = false;
    }

    on_coolant_changed.set_state(mode); // continue handling chain

    if(changed && mode.flood) {
        if (coolant_off_pending) {
            task_delete(coolant_flood_off, NULL);
            coolant_off_pending = false;
        }
        if(coolant_settings.on_delay > 0.0f && ioport_wait_on_input(Port_Digital, coolant_ok_port, WaitMode_High, coolant_settings.on_delay) != 1) {
            task_add_immediate(coolant_flood_off, NULL);
            sys.cancel = true;

            system_raise_alarm(Alarm_AbortCycle);
            task_add_immediate(report_warning, "Coolant system has failed to start.");

        } else
            coolant_on = true;
    }
}

static void coolant_fail (void *data) {
    gc_spindle_off();
}

static void onSpindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    coolant_state_t mode = hal.coolant.get_state();

    if(coolant_settings.spindle_link && state.on && !mode.flood) {

        mode.flood = On;
        gc_state.modal.coolant = mode; 
        coolant_set_state(mode);
    }

    if (!ABORTED)
        on_spindle_set_state(spindle, state, rpm);    
    else
        task_add_immediate(coolant_fail, NULL);
}

static bool onSpindleSelect (spindle_ptrs_t *spindle)
{
    on_spindle_set_state = spindle->set_state;
    spindle->set_state = onSpindleSetState;

    return on_spindle_select == NULL || on_spindle_select(spindle);
}

static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

static status_code_t set_port (setting_id_t setting, float value)
{
    status_code_t status = Status_SettingDisabled;

    switch(setting) {

        case Setting_LaserCoolantOkPort:
            status = d_in.set_value(&d_in, &coolant_settings.coolant_ok_port, (pin_cap_t){ .irq_mode = IRQ_Mode_Change }, value);
            break;

        default: break;
    }

    return status;
}

static float get_port (setting_id_t setting)
{
    float value = -1.0f;

    switch(setting) {

        case Setting_LaserCoolantOkPort:
            value = d_in.get_value(&d_in, coolant_settings.coolant_ok_port);
            break;

        default: break;
    }

    return value;
}

static const setting_detail_t plugin_settings[] = {
    { Setting_LaserCoolantOnDelay, Group_Coolant, "Laser coolant OK delay", "seconds", Format_Decimal, "#0.0", "0.0", "30.0", Setting_NonCore, &coolant_settings.on_delay, NULL, NULL },
    { Setting_LaserCoolantOkPort, Group_AuxPorts, "Coolant ok port", NULL, Format_Decimal, "-#0", "-1", d_in.port_maxs, Setting_NonCoreFn, set_port, get_port, NULL, { .reboot_required = On } },
    { ((setting_id_t)683), Group_Coolant, "Coolant to spindle enable link", NULL, Format_Bool, NULL, NULL, NULL, Setting_NonCore, &coolant_settings.spindle_link, NULL, NULL }
};

static const setting_descr_t plugin_settings_descr[] = {
    { Setting_LaserCoolantOnDelay, "" },
    { Setting_LaserCoolantOkPort, "Aux port number to use for coolant ok signal." },
    { ((setting_id_t)683), "Link coolant enable signal to spindle enable." }
};

static void coolant_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&coolant_settings, sizeof(laser_coolant_settings_t), true);
}

static void coolant_settings_restore (void)
{
    coolant_settings.on_delay =
    coolant_settings.off_delay = 0.0f;
    coolant_settings.spindle_link = false;

    //coolant_settings.coolant_ok_port = ioport_find_free(Port_Digital, Port_Input, (pin_cap_t){ .claimable = On }, "Coolant ok");

    coolant_settings_save();
}

static void coolant_settings_load (void)
{
    bool ok;

    if(hal.nvs.memcpy_from_nvs((uint8_t *)&coolant_settings, nvs_address, sizeof(laser_coolant_settings_t), true) != NVS_TransferResult_OK)
        coolant_settings_restore();

    coolant_ok_port = coolant_settings.coolant_ok_port;
    xbar_t *portinfo;

    if(!!(portinfo = d_in.claim(&d_in, &coolant_ok_port, "Coolant ok", (pin_cap_t){ .irq_mode = IRQ_Mode_Change })) &&
        ioport_enable_irq(coolant_ok_port, IRQ_Mode_Change, coolant_lost_handler)) {

        on_realtime_report = grbl.on_realtime_report;
        grbl.on_realtime_report = onRealtimeReport;

        memcpy(&on_coolant_changed, &hal.coolant, sizeof(coolant_ptrs_t));
        hal.coolant.set_state = coolantSetState;
    }
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Laser coolant", "0.10-MG");
}

void laser_coolant_init (void)
{
    static setting_details_t setting_details = {
        .settings = plugin_settings,
        .n_settings = sizeof(plugin_settings) / sizeof(setting_detail_t),
        .descriptions = plugin_settings_descr,
        .n_descriptions = sizeof(plugin_settings_descr) / sizeof(setting_descr_t),
        .save = coolant_settings_save,
        .load = coolant_settings_load,
        .restore = coolant_settings_restore,
    };

    if(ioports_cfg(&d_in, Port_Digital, Port_Input)->n_ports && (nvs_address = nvs_alloc(sizeof(laser_coolant_settings_t)))) {

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        on_spindle_select = grbl.on_spindle_select;
        grbl.on_spindle_select = onSpindleSelect;

        settings_register(&setting_details);

    } else
        task_run_on_startup(report_warning, "Laser coolant plugin failed to initialize!");
}

#endif // LASER_COOLANT_ENABLE
