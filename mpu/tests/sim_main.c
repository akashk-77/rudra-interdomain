/**
 * @file sim_main.c
 * @brief Host-PC simulation showing how to wire both FSMs together.
 *
 * Build & run:  gcc -Wall -Wextra -o sim sim_main.c rover_fsm.c mech_fsm.c -lm && ./sim
 *
 * On the real rover, replace every "fake_*" function with your hardware code
 * (servo driver, OpenCV bridge, A* module, wheel controller, sensors) and
 * replace the simulated clock with your millis(). The glue between the FSMs
 * (the mech_* wrappers) and the main loop stay exactly the same.
 */
#include <stdio.h>
#include <math.h>
#include "rover_fsm.h"
#include "mech_fsm.h"

/* ------------------------- simulated world ----------------------------- */
static uint32_t g_ms;
static float g_disk, g_disk_tgt, g_disk_spd = 60;
static float g_screw, g_screw_tgt, g_screw_spd = 10;
static int8_t g_drill;
static rover_pose_t g_pose;
static int g_detect_calls, g_drive_ticks, g_align_ticks;

static uint32_t sim_millis(void) { return g_ms; }
static void sim_log(const char *m) { printf("[%6u ms] %s\n", (unsigned)g_ms, m); }

/* Servos move toward their target at their speed (10 ms per tick). */
static void step_world(void)
{
    float dt = 0.010f, d;
    d = g_disk_tgt - g_disk;  g_disk  += fmaxf(-g_disk_spd * dt, fminf(g_disk_spd * dt, d));
    d = g_screw_tgt - g_screw; g_screw += fmaxf(-g_screw_spd * dt, fminf(g_screw_spd * dt, d));
}

/* ----------------------------- mech HAL -------------------------------- */
static void  m_disk_set(float deg, float spd) { g_disk_tgt = deg; g_disk_spd = spd; }
static float m_disk_get(void) { return g_disk; }
static void  m_screw_set(float mm, float spd) { g_screw_tgt = mm; g_screw_spd = spd; }
static float m_screw_get(void) { return g_screw; }
static void  m_drill(int8_t p) { g_drill = p; }
static uint32_t m_load(void) { return g_drill > 0 ? 1200u : 100u; }
static hal_status_t m_soil(uint8_t fn, mech_soil_sample_t *o)
{
    o->moisture_pct = 10.0f + fn; o->ph = 6.5f + 0.1f * fn;
    o->temperature_c = 24.0f; o->conductivity_us_cm = 300.0f + 10.0f * fn;
    return HAL_OK;
}

/* ----------------------------- rover HAL ------------------------------- */
static mech_fsm_t g_mech;   /* the rover talks to the mechanism through these */
static bool r_mech_idle(void)  { return mech_fsm_is_idle(&g_mech); }
static bool r_mech_start(uint8_t i) { return mech_fsm_start_cycle(&g_mech, i); }
static bool r_mech_done(void)  { return mech_fsm_cycle_done(&g_mech); }
static bool r_mech_safe(void)  { return mech_fsm_is_safe_to_move(&g_mech); }
static bool r_mech_err(void)   { return mech_fsm_has_error(&g_mech); }

static hal_status_t r_init(void) { return HAL_OK; }
static hal_status_t r_detect(rover_site_t *o, uint8_t max, uint8_t *n)
{
    if (++g_detect_calls < 3) return HAL_BUSY;      /* vision "takes time" */
    (void)max;
    o[0] = (rover_site_t){2.0f, 1.0f, 0.90f};
    o[1] = (rover_site_t){5.0f, 4.0f, 0.80f};
    o[2] = (rover_site_t){8.0f, 2.0f, 0.75f};
    *n = 3;
    return HAL_OK;
}
static bool r_pose(rover_pose_t *p) { *p = g_pose; return true; }
static hal_status_t r_plan(const rover_pose_t *s, const rover_site_t *g,
                           rover_waypoint_t *path, uint16_t max, uint16_t *len)
{   /* stand-in for A*: straight line in 3 hops */
    (void)max;
    for (int i = 0; i < 3; i++) {
        path[i].x_m = s->x_m + (g->x_m - s->x_m) * (float)(i + 1) / 3.0f;
        path[i].y_m = s->y_m + (g->y_m - s->y_m) * (float)(i + 1) / 3.0f;
    }
    *len = 3;
    return HAL_OK;
}
static hal_status_t r_drive(const rover_waypoint_t *wp)
{
    if (++g_drive_ticks < 50) return HAL_BUSY;
    g_drive_ticks = 0; g_pose.x_m = wp->x_m; g_pose.y_m = wp->y_m;
    return HAL_OK;
}
static hal_status_t r_align(const rover_site_t *s)
{
    (void)s;
    return (++g_align_ticks < 20) ? HAL_BUSY : (g_align_ticks = 0, HAL_OK);
}
static bool r_align_ok(const rover_site_t *s) { (void)s; return true; }
static bool r_still(void) { return true; }
static void r_stop(void) {}

int main(void)
{
    mech_hal_t mh = {
        .millis = sim_millis, .disk_set_target_deg = m_disk_set, .disk_get_deg = m_disk_get,
        .screw_set_target_mm = m_screw_set, .screw_get_mm = m_screw_get,
        .drill_set_pct = m_drill, .drill_load_ma = m_load, .soil_read = m_soil, .log = sim_log
    };
    rover_hal_t rh = {
        .millis = sim_millis, .init = r_init, .vision_detect_sites = r_detect,
        .get_pose = r_pose, .plan_path = r_plan, .drive_to_waypoint = r_drive,
        .align_over_site = r_align, .alignment_ok = r_align_ok, .is_stationary = r_still,
        .stop_drive = r_stop,
        .mech_is_idle = r_mech_idle, .mech_start_cycle = r_mech_start,
        .mech_cycle_done = r_mech_done, .mech_is_safe_to_move = r_mech_safe,
        .mech_has_error = r_mech_err, .log = sim_log
    };
    rover_fsm_t rover;

    mech_fsm_init(&g_mech, &mh);
    rover_fsm_init(&rover, &rh);

    /* Main loop: both FSMs are stepped every 10 ms. */
    while (!rover_fsm_mission_complete(&rover) &&
           rover_fsm_state(&rover) != ROVER_STATE_ERROR && g_ms < 600000u) {
        mech_fsm_update(&g_mech);
        rover_fsm_update(&rover);
        step_world();
        g_ms += 10;
    }

    printf("\nFinal rover state: %s (error=%d) at %u ms\n",
           rover_state_name(rover_fsm_state(&rover)), rover_fsm_error(&rover), (unsigned)g_ms);
    for (uint8_t i = 0; i < MECH_NUM_FUNNELS; i++) {
        const mech_result_t *r = mech_fsm_result(&g_mech, i);
        printf("Funnel %u (%.0f deg): valid=%d moisture=%.1f pH=%.2f\n", i,
               (i + 1) * MECH_FUNNEL_STEP_DEG, r->valid, r->avg.moisture_pct, r->avg.ph);
    }
    return 0;
}
