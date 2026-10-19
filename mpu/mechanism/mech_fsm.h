/**
 * @file mech_fsm.h
 * @brief MECHANISM state machine: rotary funnel disk + lead-screw drill + soil test.
 *
 * HARDWARE MODEL (servo motors give absolute position feedback, so we never
 * count steps ourselves - we command a target and read the position back):
 *   - Rotary disk servo : angle in degrees. 0 = HOME. Funnels sit at
 *                         90, 180, 270 deg from home.
 *   - Lead-screw servo  : drill height in mm. 0 = fully retracted (top),
 *                         positive = downward into the soil.
 *   - Drill motor       : signed percent (-100..+100).
 *                         +  = drilling direction
 *                         -  = anticlockwise (used to throw soil into funnel)
 *   - Soil sensors      : below the disk, read through soil_read().
 *
 * ONE CYCLE (triggered by mech_fsm_start_cycle(site_idx)):
 *   INIT(homing, once) -> IDLE
 *   IDLE -> DISK_TO_FUNNEL  (funnel angle = (site_idx+1)*90 deg)
 *        -> LOWER_DRILL     (to approach height, drill off)
 *        -> DRILLING        (motor on, feed to depth, stall watchdog)
 *        -> STOP_DRILL      (motor off, wait to spin down)
 *        -> RETRACT_DRILL   (back to top)
 *        -> DISCHARGE       (drill motor anticlockwise -> soil falls in funnel)
 *        -> DISK_HOME       (disk back to 0 deg)   <-- "cycle done" flag set
 *        -> TEST_SOIL       (sample sensors, average, store in results[])
 *        -> IDLE
 *   Any fault/e-stop: -> FAULT_RECOVERY (drill off, retract) -> ERROR
 *
 * Funnel k (k = site index 0,1,2) is therefore at (k+1)*90 degrees and its
 * result is stored in results[k]. This guarantees no sample mixing.
 */
#ifndef MECH_FSM_H
#define MECH_FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "fsm_common.h"

/* ======================= USER-TUNABLE CONFIGURATION ====================== */

#ifndef MECH_NUM_FUNNELS
#define MECH_NUM_FUNNELS            3
#endif
/** Angular spacing between funnels (deg). Funnel k is at (k+1)*STEP. */
#ifndef MECH_FUNNEL_STEP_DEG
#define MECH_FUNNEL_STEP_DEG        90.0f
#endif
#ifndef MECH_DISK_HOME_DEG
#define MECH_DISK_HOME_DEG          0.0f
#endif
/** Rotate the disk at what speed (deg/s) - passed to the HAL. */
#ifndef MECH_DISK_SPEED_DPS
#define MECH_DISK_SPEED_DPS         60.0f
#endif

/* ---- Lead screw (mm, 0 = top) ---- */
#ifndef MECH_SCREW_TOP_MM
#define MECH_SCREW_TOP_MM           0.0f
#endif
/** Height where the bit hovers just above the soil before drilling. */
#ifndef MECH_APPROACH_MM
#define MECH_APPROACH_MM            30.0f
#endif
/** Final drilling depth measured on the same axis as MECH_SCREW_TOP_MM. */
#ifndef MECH_DRILL_DEPTH_MM
#define MECH_DRILL_DEPTH_MM         80.0f
#endif
#ifndef MECH_SCREW_FAST_MM_S
#define MECH_SCREW_FAST_MM_S        10.0f   /**< lower / retract speed        */
#endif
#ifndef MECH_SCREW_FEED_MM_S
#define MECH_SCREW_FEED_MM_S        2.0f    /**< feed speed while cutting     */
#endif

/* ---- Drill motor ---- */
#ifndef MECH_DRILL_PCT
#define MECH_DRILL_PCT              80
#endif
#ifndef MECH_DISCHARGE_PCT
#define MECH_DISCHARGE_PCT          (-60)   /**< negative = anticlockwise     */
#endif
#ifndef MECH_DISCHARGE_TIME_MS
#define MECH_DISCHARGE_TIME_MS      3000u
#endif
#ifndef MECH_DRILL_DWELL_MS
#define MECH_DRILL_DWELL_MS         1500u   /**< spin at depth to fill bit    */
#endif
#ifndef MECH_SPINDOWN_MS
#define MECH_SPINDOWN_MS            800u
#endif
/** Stall watchdog: load above this for STALL_TIME => fault (mA). */
#ifndef MECH_STALL_CURRENT_MA
#define MECH_STALL_CURRENT_MA       2500u
#endif
#ifndef MECH_STALL_TIME_MS
#define MECH_STALL_TIME_MS          500u
#endif

/* ---- Position-reached criteria (from servo feedback) ---- */
#ifndef MECH_DISK_TOL_DEG
#define MECH_DISK_TOL_DEG           1.0f
#endif
#ifndef MECH_SCREW_TOL_MM
#define MECH_SCREW_TOL_MM           0.5f
#endif
/** Position must stay within tolerance this long to count as "reached". */
#ifndef MECH_SETTLE_MS
#define MECH_SETTLE_MS              150u
#endif

/* ---- Soil testing ---- */
#ifndef MECH_TEST_SAMPLES
#define MECH_TEST_SAMPLES           10
#endif
#ifndef MECH_TEST_INTERVAL_MS
#define MECH_TEST_INTERVAL_MS       100u
#endif

/* ---- Timeouts (ms) per motion state ---- */
#ifndef MECH_INIT_TIMEOUT_MS
#define MECH_INIT_TIMEOUT_MS        20000u
#define MECH_DISK_TIMEOUT_MS        10000u
#define MECH_SCREW_TIMEOUT_MS       30000u
#define MECH_TEST_TIMEOUT_MS        15000u
#endif

/* ================================ TYPES ================================= */

typedef enum {
    MECH_STATE_INIT = 0,
    MECH_STATE_IDLE,
    MECH_STATE_DISK_TO_FUNNEL,
    MECH_STATE_LOWER_DRILL,
    MECH_STATE_DRILLING,
    MECH_STATE_STOP_DRILL,
    MECH_STATE_RETRACT_DRILL,
    MECH_STATE_DISCHARGE,
    MECH_STATE_DISK_HOME,
    MECH_STATE_TEST_SOIL,
    MECH_STATE_FAULT_RECOVERY,
    MECH_STATE_ERROR,
    MECH_STATE_COUNT
} mech_state_t;

typedef enum {
    MECH_ERR_NONE = 0,
    MECH_ERR_ESTOP,
    MECH_ERR_INIT_TIMEOUT,
    MECH_ERR_DISK_TIMEOUT,
    MECH_ERR_SCREW_TIMEOUT,
    MECH_ERR_DRILL_STALL,
    MECH_ERR_SENSOR,
    MECH_ERR_ABORTED
} mech_error_t;

/** Edit the fields to match your real sensors. */
typedef struct {
    float moisture_pct;
    float ph;
    float temperature_c;
    float conductivity_us_cm;
} mech_soil_sample_t;

typedef struct {
    bool               valid;      /**< true once this funnel was tested */
    mech_soil_sample_t avg;        /**< average of MECH_TEST_SAMPLES     */
} mech_result_t;

typedef struct {
    /* ---- required ---- */
    uint32_t (*millis)(void);

    /** Command disk target angle (deg); servo handles motion profile. */
    void   (*disk_set_target_deg)(float deg, float speed_dps);
    float  (*disk_get_deg)(void);                 /**< servo feedback */

    /** Command lead screw target (mm) at a speed (mm/s). */
    void   (*screw_set_target_mm)(float mm, float speed_mm_s);
    float  (*screw_get_mm)(void);                 /**< servo feedback */

    /** Drill motor: -100..+100 percent, 0 = off. */
    void   (*drill_set_pct)(int8_t pct);
    /** Drill load/current in mA (for stall detection). */
    uint32_t (*drill_load_ma)(void);

    /** Read ONE sample from the sensor station of `funnel` (0-based). */
    hal_status_t (*soil_read)(uint8_t funnel, mech_soil_sample_t *out);

    /* ---- optional ---- */
    bool   (*estop_active)(void);
    void   (*log)(const char *msg);
} mech_hal_t;

typedef struct {
    mech_hal_t    hal;
    mech_state_t  state;
    mech_error_t  error;
    uint32_t      state_entry_ms;

    uint8_t       site_idx;         /**< funnel being filled this cycle     */
    float         disk_target_deg;
    bool          cycle_done;       /**< drill stowed + disk home           */

    bool          settle_active;
    uint32_t      settle_since_ms;
    uint32_t      stall_since_ms;
    bool          stall_active;
    uint32_t      dwell_start_ms;
    bool          dwell_active;

    uint8_t       test_count;
    uint32_t      test_last_ms;
    mech_soil_sample_t test_sum;

    mech_result_t results[MECH_NUM_FUNNELS];
} mech_fsm_t;

/* ================================== API ================================= */
void         mech_fsm_init(mech_fsm_t *f, const mech_hal_t *hal);
void         mech_fsm_update(mech_fsm_t *f);       /**< call periodically   */

/** Request one drill+collect cycle for site `site_idx`. Only accepted in
 *  IDLE; returns false otherwise. */
bool         mech_fsm_start_cycle(mech_fsm_t *f, uint8_t site_idx);

/** Emergency stop of the cycle: drill off, retract, then ERROR. */
void         mech_fsm_abort(mech_fsm_t *f);

bool         mech_fsm_is_idle(const mech_fsm_t *f);
bool         mech_fsm_cycle_done(const mech_fsm_t *f);
/** Drill off + retracted, disk not under rover motion hazard. */
bool         mech_fsm_is_safe_to_move(const mech_fsm_t *f);
bool         mech_fsm_has_error(const mech_fsm_t *f);
mech_state_t mech_fsm_state(const mech_fsm_t *f);
mech_error_t mech_fsm_error(const mech_fsm_t *f);
const mech_result_t *mech_fsm_result(const mech_fsm_t *f, uint8_t funnel);
const char  *mech_state_name(mech_state_t s);

#endif /* MECH_FSM_H */
