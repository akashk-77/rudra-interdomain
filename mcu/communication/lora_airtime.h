/*
 * lora_airtime.h - how long a LoRa packet occupies the air, and a duty-cycle limiter.
 *
 * Airtime matters because it sets how often the rover may transmit (regulations can
 * limit the fraction of time a transmitter may be on air) and how much battery each
 * message costs.  Formula: the standard LoRa time-on-air calculation (Semtech SX127x
 * documentation).  Hardware-independent; no radio driver is involved.
 *
 * Status: unit-tested in simulation only.  Check the result against Semtech's online
 * calculator for your exact module settings.
 */
#ifndef LORA_AIRTIME_H
#define LORA_AIRTIME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  sf;               /* spreading factor 6..12 */
    uint32_t bandwidth_hz;     /* 125000, 250000, 500000 */
    uint8_t  coding_rate;      /* 1..4  meaning 4/5 .. 4/8 */
    uint16_t preamble_symbols; /* typically 8 */
    bool     explicit_header;  /* true for normal packets */
    bool     crc_on;
} lora_params_t;

lora_params_t lora_params_default(void);   /* SF7, 125 kHz, CR 4/5, 8 preamble, header, CRC */
double lora_airtime_ms(const lora_params_t *p, size_t payload_len);   /* 0 on invalid input */

/* Duty-cycle limiter: after sending, wait airtime*(1/duty - 1) before the next send.
 * duty_fraction 0.01 = 1 %; 1.0 = no limit.  The limit to use is a regulatory value for
 * your frequency band and country: look it up, do not guess. */
typedef struct {
    double   duty_fraction;
    bool     has_next;
    uint32_t next_allowed_ms;
} dc_limiter_t;

void dc_init(dc_limiter_t *l, double duty_fraction);
bool dc_can_send(const dc_limiter_t *l, uint32_t now_ms);          /* wrap-safe */
void dc_on_sent(dc_limiter_t *l, uint32_t now_ms, double airtime_ms);

#ifdef __cplusplus
}
#endif
#endif
