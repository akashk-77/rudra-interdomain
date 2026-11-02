/* test_telemetry.c - simulation-only tests for sensor_filter, telemetry_packet, link_queue.
 * Build and run:  make test */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "link_queue.h"
#include "sensor_filter.h"
#include "telemetry_packet.h"

static int g_checks, g_failed;
#define CHECK(cond) do { g_checks++; if (!(cond)) { g_failed++; \
    printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define NEAR(a, b, tol) CHECK(fabsf((float)(a) - (float)(b)) <= (float)(tol))

/* placeholder numbers, chosen only to exercise the logic */
static sf_config_t cfg_basic(void)
{
    sf_config_t c; sf_config_default(&c);
    c.min_value = 0.0f; c.max_value = 50.0f;
    c.process_noise_per_s = 0.001f; c.measurement_var = 0.25f; c.initial_var = 1.0f;
    c.gate_sigma = 4.0f; c.max_consecutive_rejects = 3; c.invalid_after = 3;
    c.stale_ms = 5000u;
    return c;
}

static uint32_t g_rng = 12345u;
static float noise(float amp)           /* deterministic uniform noise in [-amp, amp] */
{
    g_rng = g_rng * 1664525u + 1013904223u;
    return amp * (((float)(g_rng >> 8) / 8388608.0f) - 1.0f);
}

/* ------------------------------------------------------------ sensor filter */
static void test_unknown_before_first_reading(void)
{
    sf_config_t c = cfg_basic(); sf_channel_t ch; sf_init(&ch, &c);
    sf_sample_t s = sf_poll(&ch, 1000);
    CHECK(s.quality == SF_UNKNOWN); CHECK(!s.has_value);
    CHECK(isnan(s.value)); CHECK(isnan(s.sigma));          /* never zero */
}

static void test_first_reading_initialises(void)
{
    sf_config_t c = cfg_basic(); sf_channel_t ch; sf_init(&ch, &c);
    sf_sample_t s = sf_update(&ch, 1000, 22.0f, true);
    CHECK(s.quality == SF_VALID); CHECK(s.has_value); NEAR(s.value, 22.0f, 1e-6); CHECK(s.flags == 0);
}

static void test_kalman_smooths_noise(void)
{
    sf_config_t c = cfg_basic(); c.measurement_var = 0.33f; sf_channel_t ch; sf_init(&ch, &c);
    float raw_err = 0, filt_err = 0; int n = 0; sf_sample_t s = {0};
    for (int i = 0; i < 400; i++) {
        const float z = 25.0f + noise(1.0f);
        s = sf_update(&ch, 1000u + 1000u * (uint32_t)i, z, true);
        if (i >= 300) { raw_err += fabsf(z - 25.0f); filt_err += fabsf(s.value - 25.0f); n++; }
    }
    CHECK(filt_err / n < 0.5f * (raw_err / n));            /* clearly less noisy than raw */
    CHECK(s.sigma < 0.3f);                                 /* and the filter knows it */
}

static void test_out_of_range_rejected_then_invalid_then_recovers(void)
{
    sf_config_t c = cfg_basic(); sf_channel_t ch; sf_init(&ch, &c);
    sf_update(&ch, 1000, 22.0f, true);
    sf_sample_t s = sf_update(&ch, 2000, 200.0f, true);
    CHECK(s.flags & SF_F_OUT_OF_RANGE); NEAR(s.value, 22.0f, 0.01); CHECK(s.quality == SF_VALID);
    s = sf_update(&ch, 3000, 200.0f, true);
    CHECK(s.quality == SF_VALID);
    s = sf_update(&ch, 4000, 200.0f, true);
    CHECK(s.quality == SF_INVALID);                        /* 3 unusable readings in a row */
    s = sf_update(&ch, 5000, 22.1f, true);
    CHECK(s.quality == SF_VALID); CHECK(s.flags == 0);
}

static void test_sensor_error_and_nan_are_never_turned_into_values(void)
{
    sf_config_t c = cfg_basic(); sf_channel_t ch; sf_init(&ch, &c);
    sf_update(&ch, 1000, 22.0f, true);
    sf_sample_t s = sf_update(&ch, 2000, 0.0f, false);     /* driver says "read failed" */
    CHECK(s.flags & SF_F_SENSOR_ERR); NEAR(s.value, 22.0f, 0.01);
    s = sf_update(&ch, 3000, NAN, true);
    CHECK(s.flags & SF_F_NOT_FINITE); NEAR(s.value, 22.0f, 0.01);
    sf_channel_t fresh; sf_init(&fresh, &c);
    s = sf_update(&fresh, 1000, 0.0f, false);              /* error before any good reading */
    CHECK(s.quality == SF_UNKNOWN); CHECK(!s.has_value);
}

static void test_ds18b20_disconnect_and_power_on_values(void)
{
    sf_config_t c = cfg_basic(); c.min_value = -55.0f; c.max_value = 125.0f;   /* sensor's rated range */
    sf_channel_t ch; sf_init(&ch, &c);
    for (int i = 0; i < 10; i++) sf_update(&ch, 1000u + 1000u * (uint32_t)i, 22.0f + noise(0.1f), true);
    sf_sample_t s = sf_update(&ch, 20000, -127.0f, true);   /* typical "disconnected" code */
    CHECK(s.flags & SF_F_OUT_OF_RANGE);
    s = sf_update(&ch, 21000, 85.0f, true);                 /* DS18B20 power-on-reset value: in range! */
    CHECK(s.flags & SF_F_OUTLIER);                          /* caught by the statistical gate */
    NEAR(s.value, 22.0f, 0.5);
}

static void test_single_spike_is_rejected_as_outlier(void)
{
    sf_config_t c = cfg_basic(); sf_channel_t ch; sf_init(&ch, &c);
    for (int i = 0; i < 20; i++) sf_update(&ch, 1000u + 1000u * (uint32_t)i, 20.0f + noise(0.2f), true);
    sf_sample_t s = sf_update(&ch, 21000, 45.0f, true);      /* 1 s after the last good reading */
    CHECK(s.flags & SF_F_OUTLIER); NEAR(s.value, 20.0f, 0.5); CHECK(s.quality == SF_VALID);
    s = sf_update(&ch, 22000, 20.1f, true);
    CHECK(s.flags == 0);                                    /* back to normal immediately */
}

static void test_sustained_level_shift_is_accepted(void)
{
    sf_config_t c = cfg_basic(); sf_channel_t ch; sf_init(&ch, &c);
    for (int i = 0; i < 20; i++) sf_update(&ch, 1000u + 1000u * (uint32_t)i, 20.0f, true);
    sf_sample_t s;
    for (int i = 0; i < 3; i++) {                           /* 3 rejects allowed */
        s = sf_update(&ch, 30000u + 1000u * (uint32_t)i, 40.0f, true);
        CHECK(s.flags & SF_F_OUTLIER); NEAR(s.value, 20.0f, 0.5);
    }
    s = sf_update(&ch, 33000, 40.0f, true);                 /* 4th time: it is real */
    CHECK(s.flags & SF_F_RESET); NEAR(s.value, 40.0f, 1e-6); CHECK(s.quality == SF_VALID);
}

static void test_rate_limit(void)
{
    sf_config_t c = cfg_basic(); c.max_rate_per_s = 1.0f; c.gate_sigma = 0.0f;
    sf_channel_t ch; sf_init(&ch, &c);
    sf_update(&ch, 1000, 20.0f, true);
    sf_sample_t s = sf_update(&ch, 2000, 25.0f, true);      /* 5 units in 1 s */
    CHECK(s.flags & SF_F_RATE); NEAR(s.value, 20.0f, 0.01);
    s = sf_update(&ch, 3000, 20.5f, true);
    CHECK(s.flags == 0);
}

static void test_stuck_sensor_becomes_invalid_and_recovers(void)
{
    sf_config_t c = cfg_basic(); c.stuck_samples = 5; c.stuck_epsilon = 0.0f;
    sf_channel_t ch; sf_init(&ch, &c);
    sf_sample_t s = {0};
    for (int i = 0; i < 5; i++) s = sf_update(&ch, 1000u + 1000u * (uint32_t)i, 30.0f, true);
    CHECK(s.quality == SF_INVALID); CHECK(s.flags & SF_F_STUCK);
    s = sf_update(&ch, 7000, 30.2f, true);
    CHECK(s.quality == SF_VALID); CHECK(!(s.flags & SF_F_STUCK));
}

static void test_stale_after_timeout_then_fresh_reading(void)
{
    sf_config_t c = cfg_basic(); sf_channel_t ch; sf_init(&ch, &c);
    sf_update(&ch, 0, 22.0f, true);
    CHECK(sf_poll(&ch, 4000).quality == SF_VALID);
    sf_sample_t s = sf_poll(&ch, 6000);
    CHECK(s.quality == SF_STALE); CHECK(s.flags & SF_F_STALE); CHECK(s.has_value);   /* last value, labelled stale */
    s = sf_update(&ch, 7000, 22.2f, true);
    CHECK(s.quality == SF_VALID);
}

static void test_reset_clears_everything(void)
{
    sf_config_t c = cfg_basic(); sf_channel_t ch; sf_init(&ch, &c);
    for (int i = 0; i < 10; i++) sf_update(&ch, 1000u + 1000u * (uint32_t)i, 20.0f, true);
    sf_reset(&ch);
    sf_sample_t s = sf_poll(&ch, 20000);
    CHECK(s.quality == SF_UNKNOWN); CHECK(isnan(s.value));
    s = sf_update(&ch, 21000, 40.0f, true);                 /* new site: no smoothing with old data */
    NEAR(s.value, 40.0f, 1e-6); CHECK(s.flags == 0); CHECK(s.quality == SF_VALID);
}

static void test_uncertainty_grows_without_data(void)
{
    sf_config_t c = cfg_basic(); sf_channel_t ch; sf_init(&ch, &c);
    sf_sample_t s = sf_update(&ch, 0, 22.0f, true);
    for (int i = 1; i < 50; i++) s = sf_update(&ch, 1000u * (uint32_t)i, 22.0f, true);
    const float before = s.sigma;
    s = sf_poll(&ch, 50000u + 1000000u);                    /* ~1000 s of silence */
    CHECK(s.sigma > before * 2.0f);
}

static void test_clock_wraparound(void)
{
    sf_config_t c = cfg_basic(); sf_channel_t ch; sf_init(&ch, &c);
    const uint32_t t0 = 0xFFFFFF00u;
    sf_update(&ch, t0, 22.0f, true);
    CHECK(sf_poll(&ch, t0 + 1000u).quality == SF_VALID);    /* counter has wrapped past zero */
    CHECK(sf_poll(&ch, t0 + 6000u).quality == SF_STALE);
}

/* ------------------------------------------------------------------- packet */
static tp_packet_t sample_packet(void)
{
    tp_packet_t p; memset(&p, 0, sizeof(p));
    p.type = TP_TYPE_TELEMETRY; p.node_id = 1; p.seq = 513; p.t_s = 3600; p.mission_state = 8; p.site_id = 2;
    p.n_records = 3;
    p.rec[0] = (tp_record_t){ 0, SF_VALID, 0, true, 31.457f, 0.25f };
    p.rec[1] = (tp_record_t){ 1, SF_VALID, SF_F_OUTLIER, true, -3.5f, 0.05f };
    p.rec[2] = (tp_record_t){ 2, SF_UNKNOWN, 0, false, 0.0f, 0.0f };
    return p;
}

static void test_crc_reference_value(void)
{
    CHECK(tp_crc16((const uint8_t *)"123456789", 9) == 0x29B1u);   /* standard CRC-16/CCITT-FALSE check */
}

static void test_packet_roundtrip(void)
{
    tp_packet_t p = sample_packet(), q; uint8_t buf[TP_MAX_LEN]; size_t n = 0;
    CHECK(tp_pack(&p, buf, sizeof(buf), &n) == TP_OK);
    CHECK(n == TP_HEADER_LEN + 3 * TP_RECORD_LEN + TP_CRC_LEN);
    CHECK(tp_unpack(buf, n, &q) == TP_OK);
    CHECK(q.node_id == 1 && q.seq == 513 && q.t_s == 3600 && q.mission_state == 8 && q.site_id == 2);
    CHECK(q.n_records == 3);
    NEAR(q.rec[0].value, 31.457f, 0.001); NEAR(q.rec[0].sigma, 0.25f, 0.001);
    NEAR(q.rec[1].value, -3.5f, 0.001);   CHECK(q.rec[1].flags == SF_F_OUTLIER);
}

static void test_missing_value_is_not_zero_after_roundtrip(void)
{
    tp_packet_t p = sample_packet(), q; uint8_t buf[TP_MAX_LEN]; size_t n = 0;
    tp_pack(&p, buf, sizeof(buf), &n); tp_unpack(buf, n, &q);
    CHECK(!q.rec[2].has_value); CHECK(isnan(q.rec[2].value)); CHECK(isnan(q.rec[2].sigma));
    CHECK(q.rec[2].quality == SF_UNKNOWN);
}

static void test_value_edge_cases(void)
{
    tp_packet_t p; memset(&p, 0, sizeof(p)); p.type = TP_TYPE_TELEMETRY; p.n_records = 2;
    p.rec[0] = (tp_record_t){ 0, SF_VALID, 0, true, 1e12f, 1e9f };      /* saturates, never wraps */
    p.rec[1] = (tp_record_t){ 1, SF_VALID, 0, true, -1e12f, NAN };
    uint8_t buf[TP_MAX_LEN]; size_t n; tp_packet_t q;
    CHECK(tp_pack(&p, buf, sizeof(buf), &n) == TP_OK); CHECK(tp_unpack(buf, n, &q) == TP_OK);
    CHECK(q.rec[0].has_value && q.rec[0].value > 2.0e6f);
    CHECK(q.rec[1].has_value && q.rec[1].value < -2.0e6f); CHECK(isnan(q.rec[1].sigma));
}

static void test_corruption_is_detected(void)
{
    tp_packet_t p = sample_packet(), q; uint8_t buf[TP_MAX_LEN]; size_t n;
    tp_pack(&p, buf, sizeof(buf), &n);
    for (size_t i = 0; i < n; i++) {                        /* flip any single bit anywhere */
        uint8_t copy[TP_MAX_LEN]; memcpy(copy, buf, n); copy[i] ^= 0x10u;
        CHECK(tp_unpack(copy, n, &q) != TP_OK);
    }
}

static void test_bad_lengths_and_versions(void)
{
    tp_packet_t p = sample_packet(), q; uint8_t buf[TP_MAX_LEN]; size_t n;
    tp_pack(&p, buf, sizeof(buf), &n);
    CHECK(tp_unpack(buf, 5, &q) == TP_ERR_SHORT);
    CHECK(tp_unpack(buf, n - 1, &q) == TP_ERR_LENGTH);
    uint8_t v2[TP_MAX_LEN]; memcpy(v2, buf, n);
    v2[0] = (uint8_t)((2 << 4) | TP_TYPE_TELEMETRY);
    uint16_t crc = tp_crc16(v2, n - 2); v2[n - 2] = (uint8_t)crc; v2[n - 1] = (uint8_t)(crc >> 8);
    CHECK(tp_unpack(v2, n, &q) == TP_ERR_VERSION);          /* valid CRC but unknown version */
    CHECK(tp_pack(&p, buf, 10, &n) == TP_ERR_BUF_SMALL);
    p.n_records = TP_MAX_RECORDS + 1;
    CHECK(tp_pack(&p, buf, sizeof(buf), &n) == TP_ERR_TOO_MANY);
}

static void test_heartbeat_packet(void)
{
    tp_packet_t p; memset(&p, 0, sizeof(p)); p.type = TP_TYPE_HEARTBEAT; p.node_id = 7; p.seq = 65535;
    uint8_t buf[TP_MAX_LEN]; size_t n; tp_packet_t q;
    CHECK(tp_pack(&p, buf, sizeof(buf), &n) == TP_OK); CHECK(n == TP_HEADER_LEN + TP_CRC_LEN);
    CHECK(tp_unpack(buf, n, &q) == TP_OK); CHECK(q.type == TP_TYPE_HEARTBEAT && q.seq == 65535 && q.n_records == 0);
}

/* -------------------------------------------------------- queue and tracker */
static void test_queue_fifo_and_drop_oldest(void)
{
    txq_t q; txq_init(&q);
    for (uint8_t i = 0; i < TXQ_SLOTS + 3; i++) { uint8_t d[3] = { i, i, i }; CHECK(txq_push(&q, d, 3)); }
    CHECK(txq_count(&q) == TXQ_SLOTS); CHECK(q.dropped == 3);
    const uint8_t *d; size_t len;
    CHECK(txq_peek(&q, &d, &len)); CHECK(len == 3 && d[0] == 3);       /* 0,1,2 were dropped */
    for (int i = 0; i < TXQ_SLOTS; i++) txq_pop(&q);
    CHECK(txq_count(&q) == 0); CHECK(!txq_peek(&q, &d, &len));
    txq_pop(&q);                                                        /* popping empty is harmless */
    CHECK(!txq_push(&q, d, 0)); CHECK(!txq_push(&q, (const uint8_t *)"x", TP_MAX_LEN + 1));
}

static void test_rx_tracker(void)
{
    rx_tracker_t t; rx_init(&t);
    CHECK(!rx_link_up(&t, 0, 1000));                                    /* nothing received yet */
    CHECK(rx_accept(&t, 10, 100) == RX_NEW);
    CHECK(rx_accept(&t, 10, 200) == RX_DUPLICATE);
    CHECK(rx_accept(&t, 13, 300) == RX_NEW); CHECK(t.lost == 2);        /* 11 and 12 missing */
    CHECK(rx_accept(&t, 12, 400) == RX_OLD);
    CHECK(rx_link_up(&t, 1000, 1000)); CHECK(!rx_link_up(&t, 1301, 1000));
    CHECK(t.received == 2 && t.duplicates == 1 && t.late == 1);
}

static void test_rx_tracker_sequence_wraparound(void)
{
    rx_tracker_t t; rx_init(&t);
    rx_accept(&t, 65534, 0);
    CHECK(rx_accept(&t, 65535, 1) == RX_NEW);
    CHECK(rx_accept(&t, 1, 2) == RX_NEW); CHECK(t.lost == 1);           /* 0 missing across the wrap */
}

/* ---------------------------------------------------------------- end to end */
static void test_end_to_end_sensor_to_base_station(void)
{
    sf_config_t c = cfg_basic(); sf_channel_t ch; sf_init(&ch, &c);
    txq_t q; txq_init(&q); rx_tracker_t rx; rx_init(&rx);
    uint16_t seq = 0; float last_value = NAN;

    for (int i = 0; i < 30; i++) {                          /* rover side */
        const uint32_t now = 1000u * (uint32_t)(i + 1);
        const float z = (i == 12) ? 200.0f : 22.0f + noise(0.1f);        /* one bad reading */
        sf_sample_t s = sf_update(&ch, now, z, true);
        tp_packet_t p; memset(&p, 0, sizeof(p));
        p.type = TP_TYPE_TELEMETRY; p.node_id = 1; p.seq = seq++; p.t_s = now / 1000u; p.n_records = 1;
        p.rec[0] = (tp_record_t){ 0, (uint8_t)s.quality, s.flags, s.has_value, s.value, s.sigma };
        uint8_t buf[TP_MAX_LEN]; size_t n;
        CHECK(tp_pack(&p, buf, sizeof(buf), &n) == TP_OK);
        CHECK(txq_push(&q, buf, n));
    }
    int delivered = 0;                                      /* "radio": drop every 5th packet */
    for (int i = 0; txq_count(&q) > 0; i++) {
        const uint8_t *d; size_t n; txq_peek(&q, &d, &n);
        if (i % 5 != 4) {
            tp_packet_t u; CHECK(tp_unpack(d, n, &u) == TP_OK);
            CHECK(rx_accept(&rx, u.seq, 1000u * (uint32_t)i) == RX_NEW);
            if (u.rec[0].has_value) last_value = u.rec[0].value;
            CHECK(u.rec[0].quality == SF_VALID);
            NEAR(u.rec[0].value, 22.0f, 0.5);               /* the 200.0 spike never reaches the dashboard */
            delivered++;
        }
        txq_pop(&q);
    }
    CHECK(delivered == 24); CHECK(rx.lost == 5);            /* 5 dropped, but the last one is not detectable yet */
    NEAR(last_value, 22.0f, 0.5);
}

int main(void)
{
    typedef void (*fn)(void);
    static const struct { const char *n; fn f; } tests[] = {
#define T(x) { #x, x }
        T(test_unknown_before_first_reading), T(test_first_reading_initialises),
        T(test_kalman_smooths_noise), T(test_out_of_range_rejected_then_invalid_then_recovers),
        T(test_sensor_error_and_nan_are_never_turned_into_values),
        T(test_ds18b20_disconnect_and_power_on_values), T(test_single_spike_is_rejected_as_outlier),
        T(test_sustained_level_shift_is_accepted), T(test_rate_limit),
        T(test_stuck_sensor_becomes_invalid_and_recovers), T(test_stale_after_timeout_then_fresh_reading),
        T(test_reset_clears_everything), T(test_uncertainty_grows_without_data), T(test_clock_wraparound),
        T(test_crc_reference_value), T(test_packet_roundtrip), T(test_missing_value_is_not_zero_after_roundtrip),
        T(test_value_edge_cases), T(test_corruption_is_detected), T(test_bad_lengths_and_versions),
        T(test_heartbeat_packet), T(test_queue_fifo_and_drop_oldest), T(test_rx_tracker),
        T(test_rx_tracker_sequence_wraparound), T(test_end_to_end_sensor_to_base_station),
#undef T
    };
    const size_t n = sizeof(tests) / sizeof(tests[0]);
    int bad = 0;
    for (size_t i = 0; i < n; i++) {
        const int before = g_failed;
        tests[i].f();
        const int ok = (g_failed == before);
        printf("[%s] %s\n", ok ? " OK " : "FAIL", tests[i].n);
        if (!ok) bad++;
    }
    printf("\n%zu tests, %d failed, %d checks\n", n, bad, g_checks);
    return bad ? 1 : 0;
}
