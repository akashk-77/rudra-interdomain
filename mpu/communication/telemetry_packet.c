/* telemetry_packet.c - see telemetry_packet.h. */
#include "telemetry_packet.h"

#include <math.h>
#include <string.h>

#define NO_VALUE   INT32_MIN
#define NO_SIGMA   0xFFFFu

uint16_t tp_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t encode_value(float v)
{
    const double scaled = (double)v * 1000.0;
    if (scaled >= 2147483647.0)  return INT32_MAX;
    if (scaled <= -2147483647.0) return -INT32_MAX;        /* never collide with NO_VALUE */
    return (int32_t)lround(scaled);
}

static uint16_t encode_sigma(float s)
{
    if (!(s >= 0.0f) || !isfinite(s)) return NO_SIGMA;     /* also catches NaN */
    const double scaled = (double)s * 1000.0;
    return (scaled >= 65534.0) ? 65534u : (uint16_t)lround(scaled);
}

tp_result_t tp_pack(const tp_packet_t *p, uint8_t *buf, size_t cap, size_t *out_len)
{
    if (p == NULL || buf == NULL || out_len == NULL) return TP_ERR_ARG;
    if (p->n_records > TP_MAX_RECORDS) return TP_ERR_TOO_MANY;
    if (p->type != TP_TYPE_TELEMETRY && p->type != TP_TYPE_HEARTBEAT) return TP_ERR_ARG;
    const size_t need = TP_HEADER_LEN + (size_t)p->n_records * TP_RECORD_LEN + TP_CRC_LEN;
    if (cap < need) return TP_ERR_BUF_SMALL;

    buf[0] = (uint8_t)((TP_VERSION << 4) | (p->type & 0x0Fu));
    buf[1] = p->node_id;
    put16(&buf[2], p->seq);
    put32(&buf[4], p->t_s);
    buf[8] = p->mission_state;
    buf[9] = p->site_id;
    buf[10] = p->n_records;
    uint8_t *r = &buf[TP_HEADER_LEN];
    for (uint8_t i = 0; i < p->n_records; i++, r += TP_RECORD_LEN) {
        const tp_record_t *rec = &p->rec[i];
        r[0] = rec->channel;
        r[1] = rec->quality;
        r[2] = rec->flags;
        put32(&r[3], (uint32_t)(rec->has_value ? encode_value(rec->value) : NO_VALUE));
        put16(&r[7], encode_sigma(rec->has_value ? rec->sigma : NAN));
    }
    put16(r, tp_crc16(buf, need - TP_CRC_LEN));
    *out_len = need;
    return TP_OK;
}

tp_result_t tp_unpack(const uint8_t *buf, size_t len, tp_packet_t *out)
{
    if (buf == NULL || out == NULL) return TP_ERR_ARG;
    if (len < TP_HEADER_LEN + TP_CRC_LEN) return TP_ERR_SHORT;
    const uint8_t n = buf[10];
    if (n > TP_MAX_RECORDS) return TP_ERR_TOO_MANY;
    const size_t need = TP_HEADER_LEN + (size_t)n * TP_RECORD_LEN + TP_CRC_LEN;
    if (len != need) return TP_ERR_LENGTH;
    if (get16(&buf[need - TP_CRC_LEN]) != tp_crc16(buf, need - TP_CRC_LEN)) return TP_ERR_CRC;
    if ((buf[0] >> 4) != TP_VERSION) return TP_ERR_VERSION;
    const uint8_t type = buf[0] & 0x0Fu;
    if (type != TP_TYPE_TELEMETRY && type != TP_TYPE_HEARTBEAT) return TP_ERR_ARG;

    memset(out, 0, sizeof(*out));
    out->type = type;
    out->node_id = buf[1];
    out->seq = get16(&buf[2]);
    out->t_s = get32(&buf[4]);
    out->mission_state = buf[8];
    out->site_id = buf[9];
    out->n_records = n;
    const uint8_t *r = &buf[TP_HEADER_LEN];
    for (uint8_t i = 0; i < n; i++, r += TP_RECORD_LEN) {
        tp_record_t *rec = &out->rec[i];
        rec->channel = r[0];
        rec->quality = r[1];
        rec->flags = r[2];
        const int32_t v = (int32_t)get32(&r[3]);
        rec->has_value = (v != NO_VALUE);
        rec->value = rec->has_value ? (float)v / 1000.0f : NAN;
        const uint16_t s = get16(&r[7]);
        rec->sigma = (s == NO_SIGMA) ? NAN : (float)s / 1000.0f;
    }
    return TP_OK;
}

const char *tp_result_name(tp_result_t r)
{
    switch (r) {
    case TP_OK:           return "ok";
    case TP_ERR_ARG:      return "bad argument";
    case TP_ERR_BUF_SMALL:return "buffer too small";
    case TP_ERR_TOO_MANY: return "too many records";
    case TP_ERR_SHORT:    return "packet too short";
    case TP_ERR_LENGTH:   return "length mismatch";
    case TP_ERR_CRC:      return "CRC mismatch";
    case TP_ERR_VERSION:  return "unsupported version";
    }
    return "?";
}
