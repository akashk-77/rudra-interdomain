/* test_lora_airtime.c - simulation-only tests.  Build/run: make test */
#include <math.h>
#include <stdio.h>

#include "lora_airtime.h"
#include "telemetry_packet.h"

static int g_checks, g_failed;
#define CHECK(c) do { g_checks++; if (!(c)) { g_failed++; printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); } } while (0)
#define NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

static lora_params_t sf(int s) { lora_params_t p = lora_params_default(); p.sf = (uint8_t)s; return p; }

static void test_reference_values(void)
{
    /* 10-byte payload, 125 kHz, CR 4/5, 8 preamble, explicit header, CRC: Semtech calculator defaults */
    NEAR(lora_airtime_ms(&(lora_params_t){7, 125000u, 1, 8, true, true}, 10), 41.22, 0.01);
    lora_params_t p12 = sf(12);
    NEAR(lora_airtime_ms(&p12, 10), 991.23, 0.01);
    lora_params_t p7 = sf(7);
    NEAR(lora_airtime_ms(&p7, 20), 56.6, 0.05);
    NEAR(lora_airtime_ms(&p12, 20), 1318.9, 0.1);
}

static void test_monotonic(void)
{
    for (int s = 7; s < 12; s++) {
        lora_params_t a = sf(s), b = sf(s + 1);
        CHECK(lora_airtime_ms(&a, 40) < lora_airtime_ms(&b, 40));         /* higher SF = slower */
    }
    lora_params_t p = sf(9);
    CHECK(lora_airtime_ms(&p, 20) < lora_airtime_ms(&p, 85));             /* bigger packet = longer */
    lora_params_t wide = sf(9); wide.bandwidth_hz = 250000u;
    CHECK(lora_airtime_ms(&wide, 40) < lora_airtime_ms(&p, 40));          /* wider band = faster */
}

static void test_invalid_input(void)
{
    lora_params_t p = sf(7);
    CHECK(lora_airtime_ms(NULL, 10) == 0.0);
    CHECK(lora_airtime_ms(&p, 256) == 0.0);
    p.sf = 13; CHECK(lora_airtime_ms(&p, 10) == 0.0);
}

static void test_our_packet_sizes(void)
{
    CHECK(TP_MAX_LEN == 85);                                              /* fits LoRa's 255-byte limit */
    lora_params_t p = sf(10);
    CHECK(lora_airtime_ms(&p, TP_MAX_LEN) < 1000.0);
}

static void test_duty_cycle_limiter(void)
{
    dc_limiter_t l; dc_init(&l, 0.01);                                    /* 1 %: example value only */
    CHECK(dc_can_send(&l, 0));
    dc_on_sent(&l, 1000u, 1000.0);                                        /* 1 s on air */
    CHECK(!dc_can_send(&l, 1000u + 98000u));
    CHECK(dc_can_send(&l, 1000u + 100000u));                              /* 1 s + 99 s off */
}

static void test_duty_limiter_wraparound_and_no_limit(void)
{
    dc_limiter_t l; dc_init(&l, 0.1);
    dc_on_sent(&l, 0xFFFFFF00u, 500.0);                                   /* next time wraps past zero */
    CHECK(!dc_can_send(&l, 0xFFFFFF00u + 1000u));
    CHECK(dc_can_send(&l, 0xFFFFFF00u + 6000u));
    dc_limiter_t free_l; dc_init(&free_l, 1.0);
    dc_on_sent(&free_l, 100u, 500.0);
    CHECK(dc_can_send(&free_l, 600u));                                    /* 100 %: only airtime itself */
}

int main(void)
{
    typedef void (*fn)(void);
    static const struct { const char *n; fn f; } t[] = {
        {"reference_values", test_reference_values}, {"monotonic", test_monotonic},
        {"invalid_input", test_invalid_input}, {"our_packet_sizes", test_our_packet_sizes},
        {"duty_cycle_limiter", test_duty_cycle_limiter},
        {"duty_limiter_wraparound_and_no_limit", test_duty_limiter_wraparound_and_no_limit},
    };
    int bad = 0;
    for (size_t i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
        const int before = g_failed; t[i].f();
        const int ok = (g_failed == before);
        printf("[%s] %s\n", ok ? " OK " : "FAIL", t[i].n);
        if (!ok) bad++;
    }
    printf("\n%zu tests, %d failed, %d checks\n", sizeof(t) / sizeof(t[0]), bad, g_checks);
    return bad ? 1 : 0;
}
