/*
 * telemetry_packet.h - compact binary packet for the rover -> base station radio link.
 *
 * Small on purpose: long-range links (e.g. LoRa) carry only tens of bytes per
 * message.  The radio itself is NOT chosen yet; this only defines the bytes.
 * All integers little-endian.  Packet = header + records + CRC16.
 *
 *   offset size field
 *   0      1    version(high nibble)=1 | type(low nibble)  1=telemetry 2=heartbeat
 *   1      1    node_id
 *   2      2    seq            wraps at 65536; the base station detects gaps/duplicates
 *   4      4    t_s            seconds since the rover booted
 *   8      1    mission_state  (value of the mission state machine's state enum)
 *   9      1    site_id
 *   10     1    n_records      0..TP_MAX_RECORDS
 *   11     9*n  records:  u8 channel | u8 quality | u8 flags |
 *                         i32 value*1000 (INT32_MIN = no value) |
 *                         u16 sigma*1000 (0xFFFF = unknown)
 *   end-2  2    CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over everything before it
 *
 * quality uses sensor_filter's sf_quality_t values; flags are its SF_F_* bits.
 * Values are the FILTERED stream (after failsafes and Kalman filter).  Raw data
 * stays in the rover's local log.
 */
#ifndef TELEMETRY_PACKET_H
#define TELEMETRY_PACKET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TP_VERSION      1
#define TP_MAX_RECORDS  8
#define TP_HEADER_LEN   11
#define TP_RECORD_LEN   9
#define TP_CRC_LEN      2
#define TP_MAX_LEN      (TP_HEADER_LEN + TP_MAX_RECORDS * TP_RECORD_LEN + TP_CRC_LEN)

typedef enum { TP_TYPE_TELEMETRY = 1, TP_TYPE_HEARTBEAT = 2 } tp_type_t;

typedef struct {
    uint8_t channel;
    uint8_t quality;
    uint8_t flags;
    bool    has_value;
    float   value;      /* ignored if !has_value */
    float   sigma;      /* negative or NaN = unknown */
} tp_record_t;

typedef struct {
    uint8_t     type;
    uint8_t     node_id;
    uint16_t    seq;
    uint32_t    t_s;
    uint8_t     mission_state;
    uint8_t     site_id;
    uint8_t     n_records;
    tp_record_t rec[TP_MAX_RECORDS];
} tp_packet_t;

typedef enum {
    TP_OK = 0,
    TP_ERR_ARG,
    TP_ERR_BUF_SMALL,
    TP_ERR_TOO_MANY,
    TP_ERR_SHORT,
    TP_ERR_LENGTH,
    TP_ERR_CRC,
    TP_ERR_VERSION
} tp_result_t;

uint16_t    tp_crc16(const uint8_t *data, size_t len);
tp_result_t tp_pack(const tp_packet_t *p, uint8_t *buf, size_t cap, size_t *out_len);
tp_result_t tp_unpack(const uint8_t *buf, size_t len, tp_packet_t *out);
const char *tp_result_name(tp_result_t r);

#ifdef __cplusplus
}
#endif
#endif
