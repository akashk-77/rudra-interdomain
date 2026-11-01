/* gen_sample_packet.c - prints one known packet as hex (used to cross-check the Python decoder). */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "telemetry_packet.h"
int main(void)
{
    tp_packet_t p; memset(&p, 0, sizeof(p));
    p.type = TP_TYPE_TELEMETRY; p.node_id = 1; p.seq = 513; p.t_s = 3600; p.mission_state = 8; p.site_id = 2;
    p.n_records = 3;
    p.rec[0] = (tp_record_t){ 0, 1, 0, true, 31.457f, 0.25f };
    p.rec[1] = (tp_record_t){ 1, 1, 4, true, -3.5f, 0.05f };
    p.rec[2] = (tp_record_t){ 2, 0, 0, false, 0.0f, 0.0f };
    uint8_t buf[TP_MAX_LEN]; size_t n = 0;
    if (tp_pack(&p, buf, sizeof(buf), &n) != TP_OK) return 1;
    for (size_t i = 0; i < n; i++) printf("%02x", buf[i]);
    printf("\n");
    return 0;
}
