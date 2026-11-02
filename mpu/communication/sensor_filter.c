/* sensor_filter.c - see sensor_filter.h. */
#include "sensor_filter.h"

#include <math.h>
#include <string.h>

void sf_config_default(sf_config_t *c)
{
    memset(c, 0, sizeof(*c));
    c->min_value = -1e9f;               /* PLACEHOLDERS: set per sensor */
    c->max_value = 1e9f;
    c->max_rate_per_s = 0.0f;
    c->process_noise_per_s = 0.01f;
    c->measurement_var = 1.0f;
    c->initial_var = 1.0f;
    c->gate_sigma = 4.0f;
    c->max_consecutive_rejects = 5;
    c->invalid_after = 3;
    c->stuck_samples = 0;
    c->stuck_epsilon = 0.0f;
    c->stale_ms = 10000u;
}

void sf_reset(sf_channel_t *ch)
{
    const sf_config_t cfg = ch->cfg;
    memset(ch, 0, sizeof(*ch));
    ch->cfg = cfg;
}

void sf_init(sf_channel_t *ch, const sf_config_t *cfg)
{
    memset(ch, 0, sizeof(*ch));
    ch->cfg = *cfg;
}

static void predict(sf_channel_t *ch, uint32_t now_ms)
{
    if (!ch->init) return;
    const float dt = (float)(uint32_t)(now_ms - ch->t_pred_ms) / 1000.0f;   /* wrap-safe */
    ch->P += ch->cfg.process_noise_per_s * dt;
    ch->t_pred_ms = now_ms;
}

static sf_sample_t make_sample(const sf_channel_t *ch, uint32_t now_ms)
{
    sf_sample_t s;
    s.has_value = ch->init;
    s.value = ch->init ? ch->x : NAN;
    s.sigma = ch->init ? sqrtf(ch->P) : NAN;
    s.raw = ch->has_raw ? ch->last_raw : NAN;
    s.flags = ch->flags;
    s.t_ms = ch->t_accept_ms;
    if (!ch->init) {
        s.quality = SF_UNKNOWN;
    } else if (ch->stuck_active || ch->bad_streak >= ch->cfg.invalid_after) {
        s.quality = SF_INVALID;
    } else if ((uint32_t)(now_ms - ch->t_accept_ms) > ch->cfg.stale_ms) {
        s.quality = SF_STALE;
        s.flags |= SF_F_STALE;
    } else {
        s.quality = SF_VALID;
    }
    return s;
}

static void accept(sf_channel_t *ch, uint32_t now_ms, float raw, bool level_shift)
{
    if (!ch->init || level_shift) {
        ch->x = raw;
        ch->P = ch->cfg.initial_var;
        ch->init = true;
        ch->t_pred_ms = now_ms;
        if (level_shift) ch->flags |= SF_F_RESET;
    } else {
        const float S = ch->P + ch->cfg.measurement_var;
        const float K = ch->P / S;
        ch->x += K * (raw - ch->x);
        ch->P = (1.0f - K) * ch->P;
    }
    ch->has_accept = true;
    ch->t_accept_ms = now_ms;
    ch->last_accept_raw = raw;
    ch->reject_streak = 0;
    ch->bad_streak = 0;
}

sf_sample_t sf_update(sf_channel_t *ch, uint32_t now_ms, float raw, bool sensor_ok)
{
    const sf_config_t *c = &ch->cfg;
    ch->flags = 0;
    predict(ch, now_ms);

    /* 1. hard failures: the number carries no information */
    if (!sensor_ok) {
        ch->flags |= SF_F_SENSOR_ERR;
    } else if (!isfinite(raw)) {
        ch->flags |= SF_F_NOT_FINITE;
    } else {
        ch->last_raw = raw;
        ch->has_raw = true;
        if (raw < c->min_value || raw > c->max_value) ch->flags |= SF_F_OUT_OF_RANGE;
    }
    if (ch->flags != 0) {
        if (ch->bad_streak < 255) ch->bad_streak++;
        return make_sample(ch, now_ms);
    }

    /* 4. stuck-at detection (on plausible, finite readings) */
    if (c->stuck_samples > 0) {
        if (ch->stuck_count > 0 && fabsf(raw - ch->stuck_ref) <= c->stuck_epsilon) {
            if (ch->stuck_count < 255) ch->stuck_count++;
        } else {
            ch->stuck_count = 1;
            ch->stuck_ref = raw;
        }
        ch->stuck_active = (ch->stuck_count >= c->stuck_samples);
        if (ch->stuck_active) {
            ch->flags |= SF_F_STUCK;
            return make_sample(ch, now_ms);
        }
    }

    /* 2./3. plausible but suspicious: rate limit, then innovation gate */
    bool suspicious = false;
    if (ch->init && c->max_rate_per_s > 0.0f && ch->has_accept) {
        const float dt = (float)(uint32_t)(now_ms - ch->t_accept_ms) / 1000.0f;
        if (dt > 0.0f && fabsf(raw - ch->last_accept_raw) / dt > c->max_rate_per_s) {
            ch->flags |= SF_F_RATE;
            suspicious = true;
        }
    }
    if (!suspicious && ch->init && c->gate_sigma > 0.0f) {
        const float y = raw - ch->x;
        const float S = ch->P + c->measurement_var;
        if (y * y > c->gate_sigma * c->gate_sigma * S) {
            ch->flags |= SF_F_OUTLIER;
            suspicious = true;
        }
    }
    if (suspicious) {
        ch->bad_streak = 0;                  /* a plausible number, just surprising */
        if (ch->reject_streak < 255) ch->reject_streak++;
        if (ch->reject_streak > c->max_consecutive_rejects) {
            accept(ch, now_ms, raw, true);   /* it keeps saying the same thing: believe it */
        }
        return make_sample(ch, now_ms);
    }

    accept(ch, now_ms, raw, false);
    return make_sample(ch, now_ms);
}

sf_sample_t sf_poll(sf_channel_t *ch, uint32_t now_ms)
{
    predict(ch, now_ms);
    return make_sample(ch, now_ms);
}
