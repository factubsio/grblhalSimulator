/*
  mrbear.c - MR BEAR DO NOT TOUCH
  Multi-zone keepout plugin for grblHAL.

  Zones are axis-aligned 3D boxes (X/Y/Z min/max).
  Each zone has flags controlling which operations are ALLOWED inside it.
  By default everything is blocked.
*/

#include "driver.h"

#if MRBEAR_ENABLE

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/gcode.h"
#include "grbl/system.h"
#include "grbl/motion_control.h"
#include "grbl/settings.h"
#include "grbl/plugins.h"
#include "grbl/task.h"

#ifndef MRBEAR_MAX_ZONES
#define MRBEAR_MAX_ZONES 16
#endif

#define MRBEAR_TOLERANCE 0.5f

/* Per-zone permission flags — if bit is SET, that operation is ALLOWED */
typedef union {
    uint8_t value;
    struct {
        uint8_t allow_gcode      :1,  // G-code moves may enter
                allow_jog        :1,  // Jog moves may enter
                allow_toolchange :1,  // Tool change macros may enter
                enabled          :1,  // Zone is active
                unused           :4;
    };
} zone_flags_t;

typedef struct {
    float x_min, y_min, z_min;
    float x_max, y_max, z_max;
    zone_flags_t flags;
} zone_t;

typedef struct {
    uint8_t count;
    bool global_enabled;
    zone_t zones[MRBEAR_MAX_ZONES];
} mrbear_config_t;

static mrbear_config_t config;
static nvs_address_t nvs_addr;
static bool tc_macro_running = false;

/* Saved function pointers */
static on_report_options_ptr on_report_options = NULL;
static on_realtime_report_ptr on_realtime_report = NULL;
static on_tool_selected_ptr prev_on_tool_selected = NULL;
static on_tool_changed_ptr prev_on_tool_changed = NULL;
static user_mcode_ptrs_t user_mcode = {0}; /* kept for potential future M-code use */

typedef bool (*travel_limits_ptr)(float *target, axes_signals_t axes, bool is_cartesian, work_envelope_t *envelope);
typedef void (*apply_travel_limits_ptr)(float *target, float *position, work_envelope_t *envelope);
static travel_limits_ptr prev_check_travel_limits = NULL;
static apply_travel_limits_ptr prev_apply_travel_limits = NULL;

/* Forward declarations */
static void mrbear_save(void);
static void mrbear_load(void);
static void mrbear_restore(void);

/* ── Zone helpers ─────────────────────────────────────────────────────────── */

static inline void zone_normalize(zone_t *z)
{
    float tmp;
    if (z->x_min > z->x_max) { tmp = z->x_min; z->x_min = z->x_max; z->x_max = tmp; }
    if (z->y_min > z->y_max) { tmp = z->y_min; z->y_min = z->y_max; z->y_max = tmp; }
    if (z->z_min > z->z_max) { tmp = z->z_min; z->z_min = z->z_max; z->z_max = tmp; }
}

static bool point_in_zone(const zone_t *z, float x, float y, float z_val)
{
    return x >= z->x_min && x <= z->x_max &&
           y >= z->y_min && y <= z->y_max &&
           z_val >= z->z_min && z_val <= z->z_max;
}

/* 3D Liang-Barsky: does segment (p0→p1) intersect the box? */
static bool segment_intersects_zone(const zone_t *z, const float *p0, const float *p1)
{
    float t0 = 0.0f, t1 = 1.0f;
    float d[3] = { p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2] };
    float lo[3] = { z->x_min + MRBEAR_TOLERANCE, z->y_min + MRBEAR_TOLERANCE, z->z_min + MRBEAR_TOLERANCE };
    float hi[3] = { z->x_max - MRBEAR_TOLERANCE, z->y_max - MRBEAR_TOLERANCE, z->z_max - MRBEAR_TOLERANCE };

    for (int i = 0; i < 3; i++) {
        float p_neg = -d[i], q_neg = p0[i] - lo[i];
        float p_pos =  d[i], q_pos = hi[i] - p0[i];
        if (p_neg == 0.0f && p_pos == 0.0f) {
            if (q_neg < 0.0f || q_pos < 0.0f) return false;
        } else {
            float t;
            t = q_neg / p_neg;
            if (p_neg < 0.0f) { if (t > t1) return false; if (t > t0) t0 = t; }
            else               { if (t < t0) return false; if (t < t1) t1 = t; }
            t = q_pos / p_pos;
            if (p_pos < 0.0f) { if (t > t1) return false; if (t > t0) t0 = t; }
            else               { if (t < t0) return false; if (t < t1) t1 = t; }
        }
    }
    return t0 < t1;
}

/* Clip segment to stop at zone boundary, returns true if clipped */
static bool clip_to_zone_boundary(const zone_t *z, const float *start, const float *end, float *clipped)
{
    float t0 = 0.0f, t1 = 1.0f;
    float d[3] = { end[0]-start[0], end[1]-start[1], end[2]-start[2] };
    float lo[3] = { z->x_min, z->y_min, z->z_min };
    float hi[3] = { z->x_max, z->y_max, z->z_max };

    for (int i = 0; i < 3; i++) {
        float p_neg = -d[i], q_neg = start[i] - lo[i];
        float p_pos =  d[i], q_pos = hi[i] - start[i];
        if (p_neg == 0.0f && p_pos == 0.0f) {
            if (q_neg < 0.0f || q_pos < 0.0f) return false;
        } else {
            float t;
            t = q_neg / p_neg;
            if (p_neg < 0.0f) { if (t > t1) return false; if (t > t0) t0 = t; }
            else               { if (t < t0) return false; if (t < t1) t1 = t; }
            t = q_pos / p_pos;
            if (p_pos < 0.0f) { if (t > t1) return false; if (t > t0) t0 = t; }
            else               { if (t < t0) return false; if (t < t1) t1 = t; }
        }
    }

    if (t0 > 0.0f) {
        memcpy(clipped, end, sizeof(float) * N_AXIS);
        clipped[X_AXIS] = start[0] + t0 * d[0];
        clipped[Y_AXIS] = start[1] + t0 * d[1];
        clipped[Z_AXIS] = start[2] + t0 * d[2];
        return true;
    }
    return false;
}

/* ── Travel limit hooks ───────────────────────────────────────────────────── */

/* Determine if an operation is blocked by any zone.
   op_flag: the allow_* bit to check (e.g. zone.flags.allow_jog) */
typedef enum { OP_GCODE, OP_JOG, OP_TOOLCHANGE } mrbear_op_t;

static bool op_allowed_in_zone(const zone_t *z, mrbear_op_t op)
{
    if (!z->flags.enabled) return true;
    switch (op) {
        case OP_GCODE:      return z->flags.allow_gcode;
        case OP_JOG:        return z->flags.allow_jog;
        case OP_TOOLCHANGE: return z->flags.allow_toolchange;
    }
    return true;
}

static mrbear_op_t current_op(void)
{
    if (tc_macro_running) return OP_TOOLCHANGE;
    return OP_GCODE; // check_travel_limits is gcode, apply is jog
}

/* check_travel_limits — called for G-code moves. Return false to reject. */
static bool mrbear_check_travel(float *target, axes_signals_t axes, bool is_cartesian, work_envelope_t *envelope)
{
    if (config.global_enabled) {
        float *pos = plan_get_position();
        float p0[3] = { pos ? pos[X_AXIS] : 0, pos ? pos[Y_AXIS] : 0, pos ? pos[Z_AXIS] : 0 };
        float p1[3] = { target[X_AXIS], target[Y_AXIS], target[Z_AXIS] };
        mrbear_op_t op = current_op();

        for (uint8_t i = 0; i < config.count; i++) {
            zone_t *z = &config.zones[i];
            if (!z->flags.enabled || op_allowed_in_zone(z, op)) continue;
            if (point_in_zone(z, p1[0], p1[1], p1[2]) || segment_intersects_zone(z, p0, p1)) {
                report_message("MRBEAR: Move blocked by keepout zone", Message_Warning);
                return false;
            }
        }
    }
    return prev_check_travel_limits ? prev_check_travel_limits(target, axes, is_cartesian, envelope) : true;
}

/* apply_travel_limits — called for jog moves. Clip target to zone boundary. */
static void mrbear_apply_travel(float *target, float *position, work_envelope_t *envelope)
{
    if (config.global_enabled) {
        float p0[3] = { position[X_AXIS], position[Y_AXIS], position[Z_AXIS] };
        float p1[3] = { target[X_AXIS], target[Y_AXIS], target[Z_AXIS] };

        for (uint8_t i = 0; i < config.count; i++) {
            zone_t *z = &config.zones[i];
            if (!z->flags.enabled || op_allowed_in_zone(z, OP_JOG)) continue;

            if (point_in_zone(z, p0[0], p0[1], p0[2])) {
                report_message("MRBEAR: Inside keepout zone — disable zone to jog out", Message_Warning);
                memcpy(target, position, sizeof(float) * N_AXIS);
                return;
            }

            float clipped[N_AXIS];
            if (segment_intersects_zone(z, p0, p1)) {
                if (clip_to_zone_boundary(z, position, target, clipped)) {
                    report_message("MRBEAR: Jog clipped at keepout boundary", Message_Warning);
                    memcpy(target, clipped, sizeof(float) * N_AXIS);
                    /* Update p1 for subsequent zone checks */
                    p1[0] = target[X_AXIS]; p1[1] = target[Y_AXIS]; p1[2] = target[Z_AXIS];
                }
            }
        }
    }
    if (prev_apply_travel_limits)
        prev_apply_travel_limits(target, position, envelope);
}

/* ── Tool change callbacks ────────────────────────────────────────────────── */

static void mrbear_tool_selected(tool_data_t *tool)
{
    tc_macro_running = true;
    if (prev_on_tool_selected) prev_on_tool_selected(tool);
}

static void mrbear_tool_changed(tool_data_t *tool)
{
    tc_macro_running = false;
    if (prev_on_tool_changed) prev_on_tool_changed(tool);
}

/* ── $ command handler ─────────────────────────────────────────────────────── */
/*
   $ZONE                         — list all zones
   $ZONE=n,xmin,ymin,zmin,xmax,ymax,zmax,flags  — set/add zone
   $ZONE-n                       — delete zone n
   $BEAR=1 / $BEAR=0             — global enable/disable

   flags byte: bit0=allow_gcode, bit1=allow_jog, bit2=allow_toolchange, bit3=enabled
*/

static on_unknown_sys_command_ptr prev_on_unknown_sys_command = NULL;

static status_code_t mrbear_sys_command(sys_state_t state, char *line)
{
    char buf[140];

    /* $BEAR=0/1 — global enable */
    if (strncmp(line, "BEAR=", 5) == 0) {
        config.global_enabled = line[5] != '0';
        mrbear_save();
        snprintf(buf, sizeof(buf), "[BEAR:%s]", config.global_enabled ? "enabled" : "disabled");
        hal.stream.write(buf);
        hal.stream.write(ASCII_EOL);
        return Status_OK;
    }

    /* $ZONE-n — delete zone */
    if (strncmp(line, "ZONE-", 5) == 0) {
        uint8_t slot = (uint8_t)atoi(line + 5);
        if (slot >= config.count) return Status_InvalidStatement;
        memmove(&config.zones[slot], &config.zones[slot+1], sizeof(zone_t) * (config.count - slot - 1));
        config.count--;
        memset(&config.zones[config.count], 0, sizeof(zone_t));
        mrbear_save();
        snprintf(buf, sizeof(buf), "[BEAR:zone %d deleted, %d remaining]", slot, config.count);
        hal.stream.write(buf);
        hal.stream.write(ASCII_EOL);
        return Status_OK;
    }

    /* $ZONE=n,xmin,ymin,zmin,xmax,ymax,zmax,flags — set zone */
    if (strncmp(line, "ZONE=", 5) == 0) {
        unsigned slot, flags;
        float xmin, ymin, zmin, xmax, ymax, zmax;
        if (sscanf(line + 5, "%u,%f,%f,%f,%f,%f,%f,%u",
                   &slot, &xmin, &ymin, &zmin, &xmax, &ymax, &zmax, &flags) < 7)
            return Status_InvalidStatement;
        if (slot >= MRBEAR_MAX_ZONES) return Status_InvalidStatement;
        zone_t *z = &config.zones[slot];
        z->x_min = xmin; z->y_min = ymin; z->z_min = zmin;
        z->x_max = xmax; z->y_max = ymax; z->z_max = zmax;
        z->flags.value = (flags < 8) ? (flags | 0x08) : (uint8_t)flags; /* bit3=enabled by default */
        zone_normalize(z);
        if (slot >= config.count) config.count = slot + 1;
        mrbear_save();
        snprintf(buf, sizeof(buf), "[BEAR:zone %d set]", slot);
        hal.stream.write(buf);
        hal.stream.write(ASCII_EOL);
        return Status_OK;
    }

    /* $ZONE — list all */
    if (strcmp(line, "ZONE") == 0) {
        snprintf(buf, sizeof(buf), "[BEAR:%s,%d zones]",
                 config.global_enabled ? "enabled" : "disabled", config.count);
        hal.stream.write(buf);
        hal.stream.write(ASCII_EOL);
        for (uint8_t i = 0; i < config.count; i++) {
            zone_t *z = &config.zones[i];
            snprintf(buf, sizeof(buf),
                "[ZONE:%d|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f|%u]",
                i, z->x_min, z->y_min, z->z_min, z->x_max, z->y_max, z->z_max,
                (unsigned)z->flags.value);
            hal.stream.write(buf);
            hal.stream.write(ASCII_EOL);
        }
        return Status_OK;
    }

    return prev_on_unknown_sys_command ? prev_on_unknown_sys_command(state, line) : Status_Unhandled;
}

/* ── Reporting ────────────────────────────────────────────────────────────── */

static void mrbear_report_options(bool newopt)
{
    if (on_report_options) on_report_options(newopt);
    if (!newopt) report_plugin("MR BEAR DO NOT TOUCH", "0.1.0");
}

static void mrbear_realtime_report(stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if (config.global_enabled) {
        char buf[16] = "|BEAR:";
        char *p = buf + 6;
        if (config.global_enabled) *p++ = 'E';
        float *pos = plan_get_position();
        if (pos) {
            for (uint8_t i = 0; i < config.count; i++) {
                if (config.zones[i].flags.enabled &&
                    point_in_zone(&config.zones[i], pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS])) {
                    *p++ = 'Z';
                    break;
                }
            }
        }
        *p = '\0';
        stream_write(buf);
    }
    if (on_realtime_report) on_realtime_report(stream_write, report);
}

/* ── NVS persistence ──────────────────────────────────────────────────────── */

static void mrbear_save(void)
{
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&config, sizeof(config), true);
}

static void mrbear_restore(void)
{
    memset(&config, 0, sizeof(config));
    config.global_enabled = true;
    mrbear_save();
}

static void mrbear_load(void)
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&config, nvs_addr, sizeof(config), true) != NVS_TransferResult_OK)
        mrbear_restore();

    /* Normalize all zones */
    for (uint8_t i = 0; i < config.count; i++)
        zone_normalize(&config.zones[i]);

    /* Hook into grblHAL — only once */
    if (prev_check_travel_limits == NULL) {
        prev_check_travel_limits = grbl.check_travel_limits;
        grbl.check_travel_limits = mrbear_check_travel;
        prev_apply_travel_limits = grbl.apply_travel_limits;
        grbl.apply_travel_limits = mrbear_apply_travel;

        prev_on_unknown_sys_command = grbl.on_unknown_sys_command;
        grbl.on_unknown_sys_command = mrbear_sys_command;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = mrbear_report_options;
        on_realtime_report = grbl.on_realtime_report;
        grbl.on_realtime_report = mrbear_realtime_report;

        prev_on_tool_selected = grbl.on_tool_selected;
        grbl.on_tool_selected = mrbear_tool_selected;
        prev_on_tool_changed = grbl.on_tool_changed;
        grbl.on_tool_changed = mrbear_tool_changed;
    }
}

/* ── Public init ──────────────────────────────────────────────────────────── */

void mrbear_init(void)
{
    fprintf(stderr, "[mrbear] init\n");

    static setting_details_t settings = {
        .load = mrbear_load,
        .save = mrbear_save,
        .restore = mrbear_restore
    };

    if ((nvs_addr = nvs_alloc(sizeof(config)))) {
        settings_register(&settings);
        fprintf(stderr, "[mrbear] registered, nvs_addr=%d, sizeof=%lu\n", (int)nvs_addr, (unsigned long)sizeof(config));
        report_message("MR BEAR DO NOT TOUCH v0.1.0", Message_Info);
    } else {
        fprintf(stderr, "[mrbear] nvs_alloc FAILED (need %lu bytes)\n", (unsigned long)sizeof(config));
    }
}

#endif // MRBEAR_ENABLE

