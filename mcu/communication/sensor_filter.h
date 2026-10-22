/*
 * sensor_filter.h - per-sensor validation ("failsafes") + 1-D Kalman filter.
 *
 * One sf_channel_t per sensor (soil moisture, temperature, pH).  Feed it each
 * new reading with sf_update(); call sf_poll() when no reading arrived.  It
 * returns the filtered value, its uncertainty, and a quality state, so
 * downstream code never has to guess whether a number can be trusted.
 *
 * The filter models the quantity as a slowly drifting value (random walk):
 *     predict:  P += Q * dt              (uncertainty grows with time)
 *     update :  K = P/(P+R);  x += K*(z-x);  P = (1-K)*P
 * dt comes from real timestamps, so irregular sampling is handled.
 *
 * Failsafes, in the order they are applied to each reading:
 *   1. sensor-reported error, NaN/inf, outside the plausible range  -> rejected
 *   2. optional rate-of-change limit                                -> rejected
 *   3. Kalman innovation gate (statistical outlier)                 -> rejected
 *   4. stuck-at detection (value frozen for N samples)              -> INVALID
 *   5. no accepted reading for stale_ms                             -> STALE
 * A reading rejected by 2 or 3 many times in a row is accepted as a genuine
 * level shift (e.g. the probe was just pushed into soil) and the filter
 * re-initialises (SF_F_RESET).  Call sf_reset() on each new site/probe
 * insertion so old data never smooths into the new measurement.
 *
 * Missing data is NEVER reported as zero: before the first accepted reading
 * quality is SF_UNKNOWN and value is NaN.
 *
 * The input is expected to be the CALIBRATED value.  Calibration (raw ADC ->
 * physical units) is a separate step that does not exist yet; until then,
 * ranges and noise numbers in the config are placeholders.
 *
 * Not thread-safe.  Status: unit-tested in simulation only.
 */
#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SF_UNKNOWN = 0,   /* no accepted reading yet (or just reset) */
    SF_VALID   = 1,
    SF_STALE   = 2,   /* last good value is older than stale_ms */
    SF_INVALID = 3    /* repeated bad readings, or sensor frozen */
} sf_quality_t;

enum {                /* sf_sample_t.flags bits: what happened on the latest input */
    SF_F_OUT_OF_RANGE = 1u << 0,
    SF_F_RATE         = 1u << 1,
    SF_F_OUTLIER      = 1u << 2,
    SF_F_STUCK        = 1u << 3,
    SF_F_SENSOR_ERR   = 1u << 4,
    SF_F_NOT_FINITE   = 1u << 5,
    SF_F_STALE        = 1u << 6,
    SF_F_RESET        = 1u << 7   /* level shift accepted, filter re-initialised */
};

typedef struct {
    float    min_value, max_value;      /* plausible range of the calibrated value (PLACEHOLDER) */
    float    max_rate_per_s;            /* 0 = disabled */
    float    process_noise_per_s;       /* Q: variance added per second (PLACEHOLDER) */
    float    measurement_var;           /* R: sensor noise variance (PLACEHOLDER) */
    float    initial_var;               /* P after (re)initialisation */
    float    gate_sigma;                /* innovation gate in std-devs; 0 = disabled */
    uint8_t  max_consecutive_rejects;   /* then accept as level shift */
    uint8_t  invalid_after;             /* consecutive unusable readings before SF_INVALID */
    uint8_t  stuck_samples;             /* 0 = disabled (keep 0 for low-resolution sensors) */
    float    stuck_epsilon;
    uint32_t stale_ms;
} sf_config_t;

typedef struct {
    bool         has_value;   /* false until a reading has been accepted */
    float        value;       /* filtered; NaN if !has_value */
    float        sigma;       /* 1-sigma uncertainty of value; NaN if !has_value */
    float        raw;         /* most recent raw input; NaN if none */
    sf_quality_t quality;
    uint8_t      flags;       /* SF_F_* from the latest input (SF_F_STALE added by poll) */
    uint32_t     t_ms;        /* time of the last accepted reading */
} sf_sample_t;

typedef struct {
    sf_config_t cfg;
    bool        init, has_accept, has_raw;
    float       x, P, last_accept_raw, last_raw, stuck_ref;
    uint32_t    t_pred_ms, t_accept_ms;
    uint8_t     reject_streak, bad_streak, stuck_count;
    bool        stuck_active;
    uint8_t     flags;
} sf_channel_t;

void        sf_config_default(sf_config_t *cfg);   /* generic placeholders; set real ones per sensor */
void        sf_init(sf_channel_t *ch, const sf_config_t *cfg);
void        sf_reset(sf_channel_t *ch);            /* new site / probe insertion */
sf_sample_t sf_update(sf_channel_t *ch, uint32_t now_ms, float raw, bool sensor_ok);
sf_sample_t sf_poll(sf_channel_t *ch, uint32_t now_ms);   /* no new reading */

#ifdef __cplusplus
}
#endif
#endif
