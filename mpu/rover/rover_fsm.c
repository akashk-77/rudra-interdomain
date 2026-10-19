/**
 * @file rover_fsm.c
 * @brief Implementation of the ROVER state machine. See rover_fsm.h.
 *
 * STRUCTURE
 *   rover_fsm_update() = one big switch on f->state. Each case:
 *     1. checks its timeout,
 *     2. polls the HAL function it depends on,
 *     3. decides the next state (enter_state()).
 *   To add a state: add it to rover_state_t (+ name table), add a case below.
 */
#include "rover_fsm.h"
#include <math.h>
#include <string.h>

/* ------------------------------ helpers -------------------------------- */

static const char *const k_state_names[ROVER_STATE_COUNT] = {
    "INIT", "DETECT_SITES", "VALIDATE_SITES", "SELECT_SITE", "PLAN_PATH",
    "FOLLOW_PATH", "ALIGN", "CONFIRM_STABLE", "START_MECH", "WAIT_MECH",
    "CHECK_SAFE", "WAIT_FINAL", "MISSION_COMPLETE", "ERROR"
};

const char *rover_state_name(rover_state_t s)
{
    return (s < ROVER_STATE_COUNT) ? k_state_names[s] : "?";
}

static void say(const rover_fsm_t *f, const char *msg)
{
    if (f->hal.log) f->hal.log(msg);
}

/** Switch state and run ON-ENTRY bookkeeping. */
static void enter_state(rover_fsm_t *f, rover_state_t s)
{
    f->state = s;
    f->state_entry_ms = f->hal.millis();
    f->stable_active = false;
    if (f->hal.log) {
        char buf[48];
        strncpy(buf, "ROVER -> ", sizeof buf);
        strncat(buf, rover_state_name(s), sizeof buf - strlen(buf) - 1);
        f->hal.log(buf);
    }
}

/** Fatal fault: stop driving and park in ERROR. */
static void fail(rover_fsm_t *f, rover_error_t e)
{
    if (f->hal.stop_drive) f->hal.stop_drive();
    f->error = e;
    enter_state(f, ROVER_STATE_ERROR);
}

static bool timed_out(const rover_fsm_t *f, uint32_t now, uint32_t limit_ms)
{
    return (uint32_t)(now - f->state_entry_ms) > limit_ms;
}

/** A site is acceptable if finite, inside the arena and flat enough. */
static bool site_is_valid(const rover_site_t *s)
{
    if (!isfinite(s->x_m) || !isfinite(s->y_m) || !isfinite(s->flatness_score))
        return false;
    if (s->x_m < ROVER_ARENA_MIN_X_M || s->x_m > ROVER_ARENA_MAX_X_M) return false;
    if (s->y_m < ROVER_ARENA_MIN_Y_M || s->y_m > ROVER_ARENA_MAX_Y_M) return false;
    if (s->flatness_score < ROVER_MIN_FLATNESS) return false;
    return true;
}

/** Replan (A*) after an obstacle/timeout, or give up after too many tries. */
static void replan_or_fail(rover_fsm_t *f)
{
    if (f->hal.stop_drive) f->hal.stop_drive();
    if (++f->replans > ROVER_MAX_REPLANS) fail(f, ROVER_ERR_BLOCKED);
    else enter_state(f, ROVER_STATE_PLAN_PATH);
}

/* ------------------------------- public -------------------------------- */

void rover_fsm_reset(rover_fsm_t *f)
{
    rover_hal_t hal = f->hal;
    memset(f, 0, sizeof *f);
    f->hal = hal;
    f->error = ROVER_ERR_NONE;
    enter_state(f, ROVER_STATE_INIT);
}

void rover_fsm_init(rover_fsm_t *f, const rover_hal_t *hal)
{
    memset(f, 0, sizeof *f);
    f->hal = *hal;
    enter_state(f, ROVER_STATE_INIT);
}

rover_state_t rover_fsm_state(const rover_fsm_t *f) { return f->state; }
rover_error_t rover_fsm_error(const rover_fsm_t *f) { return f->error; }
bool rover_fsm_mission_complete(const rover_fsm_t *f)
{
    return f->state == ROVER_STATE_MISSION_COMPLETE;
}

void rover_fsm_update(rover_fsm_t *f)
{
    const rover_hal_t *h = &f->hal;
    const uint32_t now = h->millis();
    hal_status_t st;

    /* Global safety checks (all states except terminal ones). */
    if (f->state != ROVER_STATE_ERROR && f->state != ROVER_STATE_MISSION_COMPLETE) {
        if (h->estop_active && h->estop_active()) { fail(f, ROVER_ERR_ESTOP); return; }
        if (h->mech_has_error && h->mech_has_error()) { fail(f, ROVER_ERR_MECH_FAULT); return; }
    }

    switch (f->state) {

    /* ---- Initialize all systems (also wait for mechanism to be homed) ---- */
    case ROVER_STATE_INIT:
        if (timed_out(f, now, ROVER_INIT_TIMEOUT_MS)) { fail(f, ROVER_ERR_INIT); break; }
        st = h->init();
        if (st == HAL_FAIL) { fail(f, ROVER_ERR_INIT); break; }
        if (st == HAL_OK && h->mech_is_idle()) {
            f->detect_retries = 0;
            enter_state(f, ROVER_STATE_DETECT_SITES);
        }
        break;

    /* ---- OpenCV detects candidate sites ---- */
    case ROVER_STATE_DETECT_SITES:
        if (timed_out(f, now, ROVER_DETECT_TIMEOUT_MS)) st = HAL_FAIL;
        else st = h->vision_detect_sites(f->candidates, ROVER_MAX_CANDIDATES,
                                         &f->candidate_count);
        if (st == HAL_OK) {
            enter_state(f, ROVER_STATE_VALIDATE_SITES);
        } else if (st == HAL_FAIL) {
            if (++f->detect_retries > ROVER_MAX_DETECT_RETRIES) fail(f, ROVER_ERR_DETECT);
            else enter_state(f, ROVER_STATE_DETECT_SITES);   /* retry */
        }
        break;

    /* ---- Validate: inside arena, flat enough, not duplicates ---- */
    case ROVER_STATE_VALIDATE_SITES: {
        uint8_t n = 0;
        for (uint8_t i = 0; i < f->candidate_count && n < ROVER_NUM_SITES; i++) {
            const rover_site_t *c = &f->candidates[i];
            if (!site_is_valid(c)) continue;
            bool dup = false;
            for (uint8_t j = 0; j < n; j++) {
                float dx = c->x_m - f->sites[j].x_m, dy = c->y_m - f->sites[j].y_m;
                if (sqrtf(dx * dx + dy * dy) < ROVER_MIN_SITE_SEPARATION_M) dup = true;
            }
            if (!dup) f->sites[n++] = *c;   /* order = detection order. EDIT HERE
                                               to sort e.g. nearest-first.   */
        }
        if (n >= ROVER_NUM_SITES) {
            f->site_idx = 0;
            enter_state(f, ROVER_STATE_SELECT_SITE);
        } else if (++f->detect_retries <= ROVER_MAX_DETECT_RETRIES) {
            say(f, "Not enough valid sites, re-detecting");
            enter_state(f, ROVER_STATE_DETECT_SITES);
        } else {
            fail(f, ROVER_ERR_VALIDATE);
        }
        break;
    }

    /* ---- Pick site #site_idx ---- */
    case ROVER_STATE_SELECT_SITE:
        f->replans = 0;
        f->align_retries = 0;
        enter_state(f, ROVER_STATE_PLAN_PATH);
        break;

    /* ---- A* path planning ---- */
    case ROVER_STATE_PLAN_PATH: {
        rover_pose_t pose;
        if (timed_out(f, now, ROVER_PLAN_TIMEOUT_MS)) { fail(f, ROVER_ERR_PLAN); break; }
        if (!h->get_pose(&pose)) break;           /* wait for pose */
        st = h->plan_path(&pose, &f->sites[f->site_idx], f->path,
                          ROVER_MAX_PATH_POINTS, &f->path_len);
        if (st == HAL_OK && f->path_len > 0) {
            f->wp_idx = 0;
            f->wp_start_ms = now;
            enter_state(f, ROVER_STATE_FOLLOW_PATH);
        } else if (st == HAL_FAIL || (st == HAL_OK && f->path_len == 0)) {
            fail(f, ROVER_ERR_PLAN);              /* no route exists */
        }
        break;
    }

    /* ---- Follow waypoints one by one ---- */
    case ROVER_STATE_FOLLOW_PATH:
        if (h->obstacle_detected && h->obstacle_detected()) { replan_or_fail(f); break; }
        if ((uint32_t)(now - f->wp_start_ms) > ROVER_WAYPOINT_TIMEOUT_MS) { replan_or_fail(f); break; }
        st = h->drive_to_waypoint(&f->path[f->wp_idx]);
        if (st == HAL_OK) {
            if (++f->wp_idx >= f->path_len) {
                h->stop_drive();
                enter_state(f, ROVER_STATE_ALIGN);
            } else {
                f->wp_start_ms = now;
            }
        } else if (st == HAL_FAIL) {
            replan_or_fail(f);
        }
        break;

    /* ---- Fine alignment so the drill is right above the site ---- */
    case ROVER_STATE_ALIGN:
        if (timed_out(f, now, ROVER_ALIGN_TIMEOUT_MS)) st = HAL_FAIL;
        else st = h->align_over_site(&f->sites[f->site_idx]);
        if (st == HAL_OK) {
            h->stop_drive();
            enter_state(f, ROVER_STATE_CONFIRM_STABLE);
        } else if (st == HAL_FAIL) {
            h->stop_drive();
            if (++f->align_retries > ROVER_MAX_ALIGN_RETRIES) fail(f, ROVER_ERR_ALIGN);
            else enter_state(f, ROVER_STATE_ALIGN);
        }
        break;

    /* ---- Must be still AND aligned for ROVER_STABLE_TIME_MS ---- */
    case ROVER_STATE_CONFIRM_STABLE:
        if (!h->alignment_ok(&f->sites[f->site_idx])) {
            f->stable_active = false;
            enter_state(f, ROVER_STATE_ALIGN);    /* drifted: re-align */
        } else if (!h->is_stationary()) {
            f->stable_active = false;             /* restart the timer */
        } else if (!f->stable_active) {
            f->stable_active = true;
            f->stable_since_ms = now;
        } else if ((uint32_t)(now - f->stable_since_ms) >= ROVER_STABLE_TIME_MS) {
            enter_state(f, ROVER_STATE_START_MECH);
        }
        break;

    /* ---- Hand over to the MECHANISM FSM ---- */
    case ROVER_STATE_START_MECH:
        if (timed_out(f, now, ROVER_MECH_START_TIMEOUT_MS)) { fail(f, ROVER_ERR_MECH_START); break; }
        if (h->mech_is_idle() && h->mech_start_cycle(f->site_idx))
            enter_state(f, ROVER_STATE_WAIT_MECH);
        break;

    /* ---- Rover stays put while the mechanism drills/discharges ---- */
    case ROVER_STATE_WAIT_MECH:
        if (timed_out(f, now, ROVER_MECH_CYCLE_TIMEOUT_MS)) { fail(f, ROVER_ERR_MECH_TIMEOUT); break; }
        if (h->mech_cycle_done()) enter_state(f, ROVER_STATE_CHECK_SAFE);
        break;

    /* ---- "Is the mechanism safe?" then advance ---- */
    case ROVER_STATE_CHECK_SAFE:
        if (timed_out(f, now, ROVER_SAFE_TIMEOUT_MS)) { fail(f, ROVER_ERR_NOT_SAFE); break; }
        if (!h->mech_is_safe_to_move()) break;
#if ROVER_WAIT_FOR_TEST_BEFORE_MOVING
        if (!h->mech_is_idle()) break;            /* tests still running */
#endif
        if (++f->site_idx >= ROVER_NUM_SITES) enter_state(f, ROVER_STATE_WAIT_FINAL);
        else enter_state(f, ROVER_STATE_SELECT_SITE);
        break;

    /* ---- After last site: let the final soil test finish ---- */
    case ROVER_STATE_WAIT_FINAL:
        if (timed_out(f, now, ROVER_FINAL_TIMEOUT_MS)) { fail(f, ROVER_ERR_TIMEOUT); break; }
        if (h->mech_is_idle()) {
            say(f, "MISSION COMPLETE");
            enter_state(f, ROVER_STATE_MISSION_COMPLETE);
        }
        break;

    case ROVER_STATE_MISSION_COMPLETE:
    case ROVER_STATE_ERROR:
    default:
        break;   /* terminal: call rover_fsm_reset() to restart */
    }
}
