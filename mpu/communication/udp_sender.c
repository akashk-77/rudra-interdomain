/* udp_sender.c - runs on the Pi. Simulates 3 sensors, filters them, packs a
 * telemetry packet, queues it, and sends it over UDP to the laptop.
 *
 * build: gcc -o udp_sender udp_sender.c telemetry_packet.c sensor_filter.c link_queue.c -lm
 * run:   ./udp_sender <laptop-ip> [port]
 */
#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "link_queue.h"
#include "sensor_filter.h"
#include "telemetry_packet.h"

#define NODE_ID   1
#define N_CH      3
#define PERIOD_US 1000000   /* 1 packet per second */

static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

static float frand(void) { return (float)rand() / (float)RAND_MAX * 2.0f - 1.0f; }

/* fake sensor readings. Replace with real sensor reads later. */
static float read_sensor(int ch, unsigned tick)
{
    switch (ch) {
    case 0:  return 30.0f + 2.0f * frand();                          /* soil moisture % */
    case 1:  return ((tick % 25) == 24) ? 45.0f                      /* temperature C, with a spike */
                                         : 24.0f + 0.5f * frand();
    default: return 6.5f + 0.1f * frand();                           /* pH */
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <laptop-ip> [port]\n", argv[0]);
        return 1;
    }
    const int port = (argc > 2) ? atoi(argv[2]) : 5005;

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return 1; }
    struct sockaddr_in dst = {0};
    dst.sin_family = AF_INET;
    dst.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, argv[1], &dst.sin_addr) != 1) {
        fprintf(stderr, "bad IP address: %s\n", argv[1]);
        return 1;
    }

    /* one filter per sensor; ranges and noise are placeholders until calibrated */
    sf_channel_t ch[N_CH];
    sf_config_t cfg;
    const float mn[N_CH] = { 0.0f, -10.0f, 0.0f };
    const float mx[N_CH] = { 100.0f, 60.0f, 14.0f };
    const float rv[N_CH] = { 1.5f, 0.1f, 0.005f };
    for (int i = 0; i < N_CH; i++) {
        sf_config_default(&cfg);
        cfg.min_value = mn[i];
        cfg.max_value = mx[i];
        cfg.measurement_var = rv[i];
        cfg.initial_var = rv[i];
        sf_init(&ch[i], &cfg);
    }

    txq_t q;
    txq_init(&q);
    uint16_t seq = 0;
    const uint32_t t0 = now_ms();

    printf("sending to %s:%d, Ctrl+C to stop\n", argv[1], port);
    for (unsigned tick = 0;; tick++) {
        const uint32_t t = now_ms();
        tp_packet_t p = {0};
        p.type = TP_TYPE_TELEMETRY;
        p.node_id = NODE_ID;
        p.seq = seq++;
        p.t_s = (t - t0) / 1000u;
        p.mission_state = 1;
        p.site_id = 1;
        p.n_records = N_CH;
        for (int i = 0; i < N_CH; i++) {
            sf_sample_t s = sf_update(&ch[i], t, read_sensor(i, tick), true);
            p.rec[i].channel = (uint8_t)i;
            p.rec[i].quality = (uint8_t)s.quality;
            p.rec[i].flags = s.flags;
            p.rec[i].has_value = s.has_value;
            p.rec[i].value = s.value;
            p.rec[i].sigma = s.sigma;
        }

        uint8_t buf[TP_MAX_LEN];
        size_t len = 0;
        tp_result_t r = tp_pack(&p, buf, sizeof(buf), &len);
        if (r != TP_OK) {
            fprintf(stderr, "pack failed: %s\n", tp_result_name(r));
        } else {
            txq_push(&q, buf, len);
        }

        /* drain the queue; pop only after a successful send */
        const uint8_t *d;
        size_t dl;
        while (txq_peek(&q, &d, &dl)) {
            if (sendto(sock, d, dl, 0, (struct sockaddr *)&dst, sizeof(dst)) < 0) {
                perror("sendto (will retry)");
                break;
            }
            txq_pop(&q);
        }
        printf("sent seq=%u len=%zu queued=%zu dropped=%u\n",
               (unsigned)p.seq, len, txq_count(&q), (unsigned)q.dropped);
        usleep(PERIOD_US);
    }
}
