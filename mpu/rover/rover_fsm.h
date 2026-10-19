/**
 * @file rover_fsm.h
 * @brief ROVER state machine: mission sequencing + navigation (A*) + alignment.
 *
 * RESPONSIBILITIES
 *   - Initialise, ask OpenCV for candidate drill sites, validate them.
 *   - For each site: plan a path (A*), follow it, align the drill axis above
 *     the site, confirm the rover is stable, hand over to the MECHANISM FSM,
 *     wait until the mechanism is safe, then go to the next site.
 *   - Declare the mission complete after ROVER_NUM_SITES sites.
 *
 * HOW IT TALKS TO THE OTHER FSM
 *   The rover FSM never touches the drill/disk. It only uses the four
 *   "mech_*" callbacks in rover_hal_t (see below). In main() you wire those
 *   to the functions in mech_fsm.h (sim_main.c shows exactly how).
 *
 * HOW TO USE
 *   1. Fill a rover_hal_t with your functions (set unused optional ones NULL).
 *   2. rover_fsm_init(&rover, &hal);
 *   3. Call rover_fsm_update(&rover) periodically (e.g. every 10-50 ms).
 *
 * STATE FLOW
 *   INIT -> DETECT_SITES -> VALIDATE_SITES -> SELECT_SITE -> PLAN_PATH
 *        -> FOLLOW_PATH -> ALIGN -> CONFIRM_STABLE -> START_MECH
 *        -> WAIT_MECH -> CHECK_SAFE -> (more sites? SELECT_SITE : WAIT_FINAL)
 *        -> MISSION_COMPLETE            (any state -> ERROR on fatal fault)
 */
#ifndef ROVER_FSM_H
#define ROVER_FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "fsm_common.h"

/* ======================= USER-TUNABLE CONFIGURATION ======================
 * Each can be overridden with -DNAME=value on the compiler command line.   */

/** Number of drill sites in the mission (= number of funnels used). */
#ifndef ROVER_NUM_SITES
#define ROVER_NUM_SITES            3
#endif
/** Max candidate sites OpenCV may return (>= ROVER_NUM_SITES). */
#ifndef ROVER_MAX_CANDIDATES
#define ROVER_MAX_CANDIDATES       8
#endif
/** Max waypoints A* may return for one trip. */
#ifndef ROVER_MAX_PATH_POINTS
#define ROVER_MAX_PATH_POINTS      128
#endif

/* ---- Site validation (metres, arena frame) ---- */
#ifndef ROVER_ARENA_MIN_X_M
#define ROVER_ARENA_MIN_X_M        0.0f
#define ROVER_ARENA_MAX_X_M        10.0f
#define ROVER_ARENA_MIN_Y_M        0.0f
#define ROVER_ARENA_MAX_Y_M        10.0f
#endif
/** Minimum flatness score (0..1) from OpenCV for a site to be "suitable". */
#ifndef ROVER_MIN_FLATNESS
#define ROVER_MIN_FLATNESS         0.60f
#endif
/** Two sites closer than this are treated as the same spot (m). */
#ifndef ROVER_MIN_SITE_SEPARATION_M
#define ROVER_MIN_SITE_SEPARATION_M 0.50f
#endif

/* ---- Timeouts (ms) and retry limits ---- */
#ifndef ROVER_INIT_TIMEOUT_MS
#define ROVER_INIT_TIMEOUT_MS      10000u
#define ROVER_DETECT_TIMEOUT_MS    5000u
#define ROVER_PLAN_TIMEOUT_MS      3000u
#define ROVER_WAYPOINT_TIMEOUT_MS  20000u  /**< max time to reach ONE waypoint */
#define ROVER_ALIGN_TIMEOUT_MS     15000u
#define ROVER_MECH_START_TIMEOUT_MS 10000u
#define ROVER_MECH_CYCLE_TIMEOUT_MS 180000u /**< whole drill+discharge cycle  */
#define ROVER_SAFE_TIMEOUT_MS      5000u
#define ROVER_FINAL_TIMEOUT_MS     60000u
#define ROVER_MAX_DETECT_RETRIES   3
#define ROVER_MAX_REPLANS          3       /**< replans per site before ERROR  */
#define ROVER_MAX_ALIGN_RETRIES    2
#endif
/** Rover must be stationary AND aligned continuously for this long (ms). */
#ifndef ROVER_STABLE_TIME_MS
#define ROVER_STABLE_TIME_MS       1500u
#endif
/** 1 = rover waits for soil TESTS to finish before driving to next site.
 *  0 = rover may leave once the drill is retracted & disk is home (tests
 *      continue in the background inside the mechanism FSM). */
#ifndef ROVER_WAIT_FOR_TEST_BEFORE_MOVING
#define ROVER_WAIT_FOR_TEST_BEFORE_MOVING 0
#endif

/* ================================ TYPES ================================= */

typedef struct { float x_m, y_m; float flatness_score; } rover_site_t;
typedef struct { float x_m, y_m; } rover_waypoint_t;
typedef struct { float x_m, y_m, heading_rad; } rover_pose_t;

typedef enum {
    ROVER_STATE_INIT = 0,
    ROVER_STATE_DETECT_SITES,
    ROVER_STATE_VALIDATE_SITES,
    ROVER_STATE_SELECT_SITE,
    ROVER_STATE_PLAN_PATH,
    ROVER_STATE_FOLLOW_PATH,
    ROVER_STATE_ALIGN,
    ROVER_STATE_CONFIRM_STABLE,
    ROVER_STATE_START_MECH,
    ROVER_STATE_WAIT_MECH,
    ROVER_STATE_CHECK_SAFE,
    ROVER_STATE_WAIT_FINAL,
    ROVER_STATE_MISSION_COMPLETE,
    ROVER_STATE_ERROR,
    ROVER_STATE_COUNT
} rover_state_t;

typedef enum {
    ROVER_ERR_NONE = 0,
    ROVER_ERR_ESTOP,
    ROVER_ERR_INIT,
    ROVER_ERR_DETECT,          /**< vision failed / not enough sites       */
    ROVER_ERR_VALIDATE,        /**< fewer than ROVER_NUM_SITES valid sites */
    ROVER_ERR_PLAN,            /**< A* could not find a path               */
    ROVER_ERR_BLOCKED,         /**< too many replans (obstacles/timeouts)  */
    ROVER_ERR_ALIGN,
    ROVER_ERR_MECH_START,
    ROVER_ERR_MECH_FAULT,      /**< mechanism FSM reported an error        */
    ROVER_ERR_MECH_TIMEOUT,
    ROVER_ERR_NOT_SAFE,
    ROVER_ERR_TIMEOUT          /**< generic state timeout                  */
} rover_error_t;

/**
 * Hardware / software abstraction. Fill these in for your platform.
 * Functions returning hal_status_t are polled until not HAL_BUSY.
 */
typedef struct {
    /* ---- required ---- */
    uint32_t     (*millis)(void);                       /**< ms tick counter */

    /** One-time/boot init of drive, IMU, camera, ... (may take time). */
    hal_status_t (*init)(void);

    /** Run OpenCV site detection. Write up to `max` sites into `out`, set
     *  *count. Return HAL_OK when a result is ready. */
    hal_status_t (*vision_detect_sites)(rover_site_t *out, uint8_t max,
                                        uint8_t *count);

    /** Current pose estimate (odometry/visual). Return false if unavailable. */
    bool         (*get_pose)(rover_pose_t *pose);

    /** A*: plan from `start` to `goal`. Fill path[0..*len-1]. */
    hal_status_t (*plan_path)(const rover_pose_t *start,
                              const rover_site_t *goal,
                              rover_waypoint_t *path, uint16_t max_len,
                              uint16_t *len);

    /** Drive toward one waypoint. HAL_OK = reached, HAL_FAIL = cannot. */
    hal_status_t (*drive_to_waypoint)(const rover_waypoint_t *wp);

    /** Fine visual-servo alignment of the drill axis over `site`.
     *  HAL_OK = within tolerance. */
    hal_status_t (*align_over_site)(const rover_site_t *site);

    /** True when alignment is currently within tolerance. */
    bool         (*alignment_ok)(const rover_site_t *site);

    /** True when wheels are stopped / velocity ~ 0 (IMU + encoders). */
    bool         (*is_stationary)(void);

    /** Stop all drive motors immediately. */
    void         (*stop_drive)(void);

    /* ---- link to the MECHANISM FSM (wire to mech_fsm.h functions) ---- */
    bool         (*mech_is_idle)(void);                 /**< ready for cycle */
    bool         (*mech_start_cycle)(uint8_t site_idx); /**< begin drilling  */
    bool         (*mech_cycle_done)(void);              /**< drill stowed, disk home */
    bool         (*mech_is_safe_to_move)(void);
    bool         (*mech_has_error)(void);

    /* ---- optional (may be NULL) ---- */
    bool         (*obstacle_detected)(void);            /**< lidar/ultrasonic */
    bool         (*estop_active)(void);
    void         (*log)(const char *msg);
} rover_hal_t;

typedef struct {
    rover_hal_t   hal;
    rover_state_t state;
    rover_error_t error;
    uint32_t      state_entry_ms;

    rover_site_t  candidates[ROVER_MAX_CANDIDATES];
    uint8_t       candidate_count;
    rover_site_t  sites[ROVER_NUM_SITES];   /**< validated, mission order     */
    uint8_t       site_idx;                 /**< current site (0-based)       */

    rover_waypoint_t path[ROVER_MAX_PATH_POINTS];
    uint16_t      path_len;
    uint16_t      wp_idx;
    uint32_t      wp_start_ms;

    uint8_t       detect_retries;
    uint8_t       replans;
    uint8_t       align_retries;
    bool          stable_active;
    uint32_t      stable_since_ms;
} rover_fsm_t;

/* ================================== API ================================= */
void          rover_fsm_init(rover_fsm_t *f, const rover_hal_t *hal);
void          rover_fsm_update(rover_fsm_t *f);   /**< call periodically */
void          rover_fsm_reset(rover_fsm_t *f);    /**< restart mission   */
rover_state_t rover_fsm_state(const rover_fsm_t *f);
rover_error_t rover_fsm_error(const rover_fsm_t *f);
bool          rover_fsm_mission_complete(const rover_fsm_t *f);
const char   *rover_state_name(rover_state_t s);

#endif /* ROVER_FSM_H */
