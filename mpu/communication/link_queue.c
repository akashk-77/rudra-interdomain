/* link_queue.c - see link_queue.h. */
#include "link_queue.h"

#include <string.h>

void txq_init(txq_t *q) { memset(q, 0, sizeof(*q)); }

bool txq_push(txq_t *q, const uint8_t *data, size_t len)
{
    if (q == NULL || data == NULL || len == 0 || len > TP_MAX_LEN) return false;
    if (q->count == TXQ_SLOTS) {                       /* full: drop the oldest */
        q->head = (uint8_t)((q->head + 1u) % TXQ_SLOTS);
        q->count--;
        q->dropped++;
    }
    txq_slot_t *s = &q->slot[(q->head + q->count) % TXQ_SLOTS];
    s->len = (uint8_t)len;
    memcpy(s->data, data, len);
    q->count++;
    return true;
}

bool txq_peek(const txq_t *q, const uint8_t **data, size_t *len)
{
    if (q->count == 0) return false;
    *data = q->slot[q->head].data;
    *len = q->slot[q->head].len;
    return true;
}

void txq_pop(txq_t *q)
{
    if (q->count == 0) return;
    q->head = (uint8_t)((q->head + 1u) % TXQ_SLOTS);
    q->count--;
}

size_t txq_count(const txq_t *q) { return q->count; }

void rx_init(rx_tracker_t *t) { memset(t, 0, sizeof(*t)); }

rx_result_t rx_accept(rx_tracker_t *t, uint16_t seq, uint32_t now_ms)
{
    if (!t->seen) {
        t->seen = true;
        t->last_seq = seq;
        t->last_rx_ms = now_ms;
        t->received++;
        return RX_NEW;
    }
    const uint16_t diff = (uint16_t)(seq - t->last_seq);       /* wrap-safe */
    if (diff == 0) { t->duplicates++; return RX_DUPLICATE; }
    if (diff >= 0x8000u) { t->late++; return RX_OLD; }         /* older than the newest seen */
    t->lost += (uint32_t)(diff - 1u);
    t->last_seq = seq;
    t->last_rx_ms = now_ms;
    t->received++;
    return RX_NEW;
}

bool rx_link_up(const rx_tracker_t *t, uint32_t now_ms, uint32_t timeout_ms)
{
    return t->seen && (uint32_t)(now_ms - t->last_rx_ms) <= timeout_ms;
}
