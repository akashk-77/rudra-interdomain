/*
 * link_queue.h - the two small pieces of the radio link that do not depend on
 * which radio is chosen:
 *
 *  txq_t        rover side: FIFO of packets waiting to be sent.  If the link is
 *               down the rover keeps queueing; when full, the OLDEST packet is
 *               dropped (and counted) so the newest data always gets through.
 *               This is a RAM buffer for short outages.  The full-resolution
 *               record belongs in the rover's local log/database.
 *
 *  rx_tracker_t base-station side: tells new packets from duplicates and late
 *               ones, counts lost packets from sequence-number gaps, and says
 *               whether the link is up.  The dashboard must show "link lost" /
 *               data age from this instead of presenting old values as live.
 */
#ifndef LINK_QUEUE_H
#define LINK_QUEUE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "telemetry_packet.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TXQ_SLOTS 32

typedef struct { uint8_t len; uint8_t data[TP_MAX_LEN]; } txq_slot_t;

typedef struct {
    txq_slot_t slot[TXQ_SLOTS];
    uint8_t    head;       /* index of the oldest packet */
    uint8_t    count;
    uint32_t   dropped;    /* packets discarded because the queue was full */
} txq_t;

void   txq_init(txq_t *q);
bool   txq_push(txq_t *q, const uint8_t *data, size_t len);        /* false if len invalid */
bool   txq_peek(const txq_t *q, const uint8_t **data, size_t *len);/* oldest, not removed */
void   txq_pop(txq_t *q);                                          /* call after a confirmed send */
size_t txq_count(const txq_t *q);

typedef enum { RX_NEW = 0, RX_DUPLICATE, RX_OLD } rx_result_t;

typedef struct {
    bool     seen;
    uint16_t last_seq;
    uint32_t last_rx_ms;
    uint32_t received, lost, duplicates, late;
} rx_tracker_t;

void        rx_init(rx_tracker_t *t);
rx_result_t rx_accept(rx_tracker_t *t, uint16_t seq, uint32_t now_ms);
bool        rx_link_up(const rx_tracker_t *t, uint32_t now_ms, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
#endif
