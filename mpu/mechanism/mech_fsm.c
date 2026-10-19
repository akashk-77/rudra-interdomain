/**
 * @file mech_fsm.c
 * @brief Implementation of the MECHANISM state machine. See mech_fsm.h.
 *
 * PATTERN USED IN EVERY STATE
 *   enter_state()  -> ON-ENTRY action: send the motor command ONCE.
 *   mech_fsm_update() -> poll feedback; when the goal is reached (position
 *                        settled / timer elapsed) move to the next state.
 * To change the sequence, edit enter_state() (what to command) and the
 * matching case in update() (when to move on).
 */
#include "mech_fsm.h"
#include <math.h>
#include <string.h>

static const char *const k_names[MECH_STATE_COUNT] = {
    "INIT", "IDLE", "DISK_TO_FUNNEL", "LOWER_DRILL", "DRILLING", "STOP_DRILL",
    "RETRACT_DRILL", "DISCHARGE", "DISK_HOME", "TEST_SOIL", "FAULT_RECOVERY",
    "ERROR"
};

const char *mech_state_name(mech_state_t s) { return (s < MECH_STATE_COUNT) ? k_names[s] : "?"; }

/* ------------------------------ helpers -------------------------------- */

static void say(const mech_fsm_t *f, const char *msg)
{
    if (f->hal.log) f->hal.log(msg);
}

static uint32_t elapsed(const mech_fsm_t *f, uint32_t now)
{
    return (uint32_t)(now - f->state_entry_ms);
}

/** True once |error| <= tol has held continuously for MECH_SETTLE_MS. */
static bool settled(mech_fsm_t *f, float err, float tol, uint32_t now)
{
    if (fabsf(err) > tol) { f->settle_active = false; return false; }
    if (!f->settle_active) { f->settle_active = true; f->settle_since_ms = now; }
    return (uint32_t)(now - f->settle_since_ms) >= MECH_SETTLE_MS;
}

static bool disk_settled(mech_fsm_t *f, uint32_t now)
{
    return settled(f, f->hal.disk_get_deg() - f->disk_target_deg, MECH_DISK_TOL_DEG, now);
}

static bool screw_settled(mech_fsm_t *f, float target_mm, uint32_t now)
{
    return settled(f, f->hal.screw_get_mm() - target_mm, MECH_SCREW_TOL_MM, now);
}

/** Funnel k sits at (k+1)*MECH_FUNNEL_STEP_DEG from home. */
static float funnel_angle_deg(uint8_t site_idx)
{
    return MECH_DISK_HOME_DEG + (float)(site_idx + 1u) * MECH_FUNNEL_STEP_DEG;
}

/** Switch state and perform its ON-ENTRY action (command motors once). */
static void enter_state(mech_fsm_t *f, mech_state_t s)
{
    const mech_hal_t *h = &f->hal;
    f->state = s;
    f->state_entry_ms = h->millis();
    f->settle_active = false;
    f->dwell_active = false;
    f->stall_active = false;

    if (h->log) {
        char buf[48];
        strncpy(buf, "MECH -> ", sizeof buf);
        strncat(buf, mech_state_name(s), sizeof buf - strlen(buf) - 1);
        h->log(buf);
    }

    switch (s) {
    case MECH_STATE_INIT:          /* homing: drill off, screw up, disk home */
        h->drill_set_pct(0);
        h->screw_set_target_mm(MECH_SCREW_TOP_MM, MECH_SCREW_FAST_MM_S);
        f->disk_target_deg = MECH_DISK_HOME_DEG;
        h->disk_set_target_deg(f->disk_target_deg, MECH_DISK_SPEED_DPS);
        break;

    case MECH_STATE_DISK_TO_FUNNEL:  /* after retraction, align THIS funnel */
        f->disk_target_deg = funnel_angle_deg(f->site_idx);
        h->disk_set_target_deg(f->disk_target_deg, MECH_DISK_SPEED_DPS);
        break;

    case MECH_STATE_LOWER_DRILL:     /* hover just above soil, drill still off */
        h->screw_set_target_mm(MECH_APPROACH_MM, MECH_SCREW_FAST_MM_S);
        break;

    case MECH_STATE_DRILLING:        /* spin and feed slowly to full depth */
        h->drill_set_pct(MECH_DRILL_PCT);
        h->screw_set_target_mm(MECH_DRILL_DEPTH_MM, MECH_SCREW_FEED_MM_S);
        break;

    case MECH_STATE_STOP_DRILL:      /* motor off, hold position while it stops */
        h->drill_set_pct(0);
        break;

    case MECH_STATE_RETRACT_DRILL:   /* pull the bit (with soil) back up */
        h->screw_set_target_mm(MECH_SCREW_TOP_MM, MECH_SCREW_FAST_MM_S);
        break;

    case MECH_STATE_DISCHARGE:       /* release the collected soil into the aligned funnel */
        h->drill_set_pct((int8_t)MECH_DISCHARGE_PCT);
        break;

    case MECH_STATE_DISK_HOME:       /* funnel leaves; disk returns to 0 deg */
        h->drill_set_pct(0);
        f->disk_target_deg = MECH_DISK_HOME_DEG;
        h->disk_set_target_deg(f->disk_target_deg, MECH_DISK_SPEED_DPS);
        break;

    case MECH_STATE_TEST_SOIL:       /* reset the averaging accumulator */
        f->cycle_done = true;        /* rover may now leave (drill safe)   */
        f->test_count = 0;
        f->test_last_ms = h->millis() - MECH_TEST_INTERVAL_MS; /* sample at once */
        memset(&f->test_sum, 0, sizeof f->test_sum);
        break;

    case MECH_STATE_FAULT_RECOVERY:  /* make everything safe */
        h->drill_set_pct(0);
        h->screw_set_target_mm(MECH_SCREW_TOP_MM, MECH_SCREW_FAST_MM_S);
        break;

    case MECH_STATE_ERROR:
        h->drill_set_pct(0);
        break;

    case MECH_STATE_IDLE:
    default:
        break;
    }
}

/** Any fault: record the cause and run the safe-recovery sequence. */
static void fault(mech_fsm_t *f, mech_error_t e)
{
    if (f->state == MECH_STATE_FAULT_RECOVERY || f->state == MECH_STATE_ERROR) return;
    f->error = e;
    say(f, "MECH FAULT");
    enter_state(f, MECH_STATE_FAULT_RECOVERY);
}

/* ------------------------------- public -------------------------------- */

void mech_fsm_init(mech_fsm_t *f, const mech_hal_t *hal)
{
    memset(f, 0, sizeof *f);
    f->hal = *hal;
    enter_state(f, MECH_STATE_INIT);
}

bool mech_fsm_start_cycle(mech_fsm_t *f, uint8_t site_idx)
{
    if (f->state != MECH_STATE_IDLE || site_idx >= MECH_NUM_FUNNELS) return false;
    f->site_idx = site_idx;
    f->cycle_done = false;
    /* Begin by lowering the drill; rotate to the funnel only after retraction. */
    enter_state(f, MECH_STATE_LOWER_DRILL);
    return true;
}

void mech_fsm_abort(mech_fsm_t *f) { fault(f, MECH_ERR_ABORTED); }

bool mech_fsm_is_idle(const mech_fsm_t *f)       { return f->state == MECH_STATE_IDLE; }
bool mech_fsm_cycle_done(const mech_fsm_t *f)    { return f->cycle_done; }
bool mech_fsm_has_error(const mech_fsm_t *f)     { return f->state == MECH_STATE_ERROR
                                                       || f->state == MECH_STATE_FAULT_RECOVERY; }
mech_state_t mech_fsm_state(const mech_fsm_t *f) { return f->state; }
mech_error_t mech_fsm_error(const mech_fsm_t *f) { return f->error; }

bool mech_fsm_is_safe_to_move(const mech_fsm_t *f)
{
    /* Drill is off and retracted only in these two states. */
    return f->state == MECH_STATE_IDLE || f->state == MECH_STATE_TEST_SOIL;
}

const mech_result_t *mech_fsm_result(const mech_fsm_t *f, uint8_t funnel)
{
    return (funnel < MECH_NUM_FUNNELS) ? &f->results[funnel] : (const mech_result_t *)0;
}

void mech_fsm_update(mech_fsm_t *f)
{
    const mech_hal_t *h = &f->hal;
    const uint32_t now = h->millis();

    /* Global safety: e-stop in any active state. */
    if (h->estop_active && h->estop_active()) fault(f, MECH_ERR_ESTOP);

    switch (f->state) {

    /* ---- Homing at boot ---- */
    case MECH_STATE_INIT:
        if (elapsed(f, now) > MECH_INIT_TIMEOUT_MS) { fault(f, MECH_ERR_INIT_TIMEOUT); break; }
        if (screw_settled(f, MECH_SCREW_TOP_MM, now) &&
            fabsf(h->disk_get_deg() - MECH_DISK_HOME_DEG) <= MECH_DISK_TOL_DEG)
            enter_state(f, MECH_STATE_IDLE);
        break;

    case MECH_STATE_IDLE:
        break;   /* waiting for mech_fsm_start_cycle() */

    /* ---- Position the retracted sample above the funnel ---- */
    case MECH_STATE_DISK_TO_FUNNEL:
        if (elapsed(f, now) > MECH_DISK_TIMEOUT_MS) { fault(f, MECH_ERR_DISK_TIMEOUT); break; }
        if (disk_settled(f, now)) enter_state(f, MECH_STATE_DISCHARGE);
        break;

    case MECH_STATE_LOWER_DRILL:
        if (elapsed(f, now) > MECH_SCREW_TIMEOUT_MS) { fault(f, MECH_ERR_SCREW_TIMEOUT); break; }
        if (screw_settled(f, MECH_APPROACH_MM, now)) enter_state(f, MECH_STATE_DRILLING);
        break;

    /* ---- Drill: watch for stall, finish at depth + dwell ---- */
    case MECH_STATE_DRILLING:
        if (elapsed(f, now) > MECH_SCREW_TIMEOUT_MS * 2u) { fault(f, MECH_ERR_SCREW_TIMEOUT); break; }

        if (h->drill_load_ma() > MECH_STALL_CURRENT_MA) {
            if (!f->stall_active) { f->stall_active = true; f->stall_since_ms = now; }
            else if ((uint32_t)(now - f->stall_since_ms) > MECH_STALL_TIME_MS) {
                fault(f, MECH_ERR_DRILL_STALL);
                break;
            }
        } else {
            f->stall_active = false;
        }

        if (screw_settled(f, MECH_DRILL_DEPTH_MM, now)) {
            if (!f->dwell_active) { f->dwell_active = true; f->dwell_start_ms = now; }
            else if ((uint32_t)(now - f->dwell_start_ms) >= MECH_DRILL_DWELL_MS)
                enter_state(f, MECH_STATE_STOP_DRILL);
        }
        break;

    case MECH_STATE_STOP_DRILL:
        if (elapsed(f, now) >= MECH_SPINDOWN_MS) enter_state(f, MECH_STATE_RETRACT_DRILL);
        break;

    case MECH_STATE_RETRACT_DRILL:
        if (elapsed(f, now) > MECH_SCREW_TIMEOUT_MS) { fault(f, MECH_ERR_SCREW_TIMEOUT); break; }
        if (screw_settled(f, MECH_SCREW_TOP_MM, now)) enter_state(f, MECH_STATE_DISK_TO_FUNNEL);
        break;

    case MECH_STATE_DISCHARGE:
        if (elapsed(f, now) >= MECH_DISCHARGE_TIME_MS) enter_state(f, MECH_STATE_DISK_HOME);
        break;

    case MECH_STATE_DISK_HOME:
        if (elapsed(f, now) > MECH_DISK_TIMEOUT_MS) { fault(f, MECH_ERR_DISK_TIMEOUT); break; }
        if (disk_settled(f, now)) enter_state(f, MECH_STATE_TEST_SOIL);
        break;

    /* ---- Sample the sensors, average, store in results[site_idx] ---- */
    case MECH_STATE_TEST_SOIL:
        if (elapsed(f, now) > MECH_TEST_TIMEOUT_MS) { fault(f, MECH_ERR_SENSOR); break; }
        if ((uint32_t)(now - f->test_last_ms) >= MECH_TEST_INTERVAL_MS) {
            mech_soil_sample_t s;
            hal_status_t st = h->soil_read(f->site_idx, &s);
            if (st == HAL_FAIL) { fault(f, MECH_ERR_SENSOR); break; }
            if (st == HAL_OK) {
                f->test_sum.moisture_pct        += s.moisture_pct;
                f->test_sum.ph                  += s.ph;
                f->test_sum.temperature_c       += s.temperature_c;
                f->test_sum.conductivity_us_cm  += s.conductivity_us_cm;
                f->test_last_ms = now;
                if (++f->test_count >= MECH_TEST_SAMPLES) {
                    mech_result_t *r = &f->results[f->site_idx];
                    float n = (float)MECH_TEST_SAMPLES;
                    r->avg.moisture_pct       = f->test_sum.moisture_pct / n;
                    r->avg.ph                 = f->test_sum.ph / n;
                    r->avg.temperature_c      = f->test_sum.temperature_c / n;
                    r->avg.conductivity_us_cm = f->test_sum.conductivity_us_cm / n;
                    r->valid = true;
                    enter_state(f, MECH_STATE_IDLE);
                }
            }
        }
        break;

    /* ---- Safe-out after a fault: drill off, screw up, then ERROR ---- */
    case MECH_STATE_FAULT_RECOVERY:
        if (elapsed(f, now) > MECH_SCREW_TIMEOUT_MS || screw_settled(f, MECH_SCREW_TOP_MM, now))
            enter_state(f, MECH_STATE_ERROR);
        break;

    case MECH_STATE_ERROR:
    default:
        break;   /* terminal: re-run mech_fsm_init() after fixing the cause */
    }
}
