/* lora_airtime.c - see lora_airtime.h. */
#include "lora_airtime.h"

#include <math.h>

lora_params_t lora_params_default(void)
{
    lora_params_t p = { 7, 125000u, 1, 8, true, true };
    return p;
}

double lora_airtime_ms(const lora_params_t *p, size_t payload_len)
{
    if (p == NULL || p->sf < 6 || p->sf > 12 || p->bandwidth_hz == 0 ||
        p->coding_rate < 1 || p->coding_rate > 4 || payload_len > 255) return 0.0;

    const double sf = p->sf;
    const double t_sym_ms = pow(2.0, sf) / (double)p->bandwidth_hz * 1000.0;
    const int de = (t_sym_ms > 16.0) ? 1 : 0;            /* low data rate optimisation */
    const int ih = p->explicit_header ? 0 : 1;
    const double num = 8.0 * (double)payload_len - 4.0 * sf + 28.0 +
                       (p->crc_on ? 16.0 : 0.0) - 20.0 * ih;
    const double den = 4.0 * (sf - 2.0 * de);
    double groups = ceil(num / den);
    if (groups < 0.0) groups = 0.0;
    const double payload_symbols = 8.0 + groups * (p->coding_rate + 4.0);
    return ((double)p->preamble_symbols + 4.25 + payload_symbols) * t_sym_ms;
}

void dc_init(dc_limiter_t *l, double duty_fraction)
{
    l->duty_fraction = (duty_fraction > 0.0 && duty_fraction <= 1.0) ? duty_fraction : 1.0;
    l->has_next = false;
    l->next_allowed_ms = 0;
}

bool dc_can_send(const dc_limiter_t *l, uint32_t now_ms)
{
    if (!l->has_next) return true;
    return (int32_t)(now_ms - l->next_allowed_ms) >= 0;          /* wrap-safe */
}

void dc_on_sent(dc_limiter_t *l, uint32_t now_ms, double airtime_ms)
{
    const double off_ms = airtime_ms * (1.0 / l->duty_fraction - 1.0);
    l->next_allowed_ms = now_ms + (uint32_t)ceil(airtime_ms + off_ms);
    l->has_next = true;
}
