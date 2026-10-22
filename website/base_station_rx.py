"""base_station_rx.py - base-station side: receive telemetry over UDP, check it, and report.

    python3 base_station_rx.py --port 9000 [--link-timeout 5] [--max-packets N] [--json]

For every datagram: verify length + CRC (damaged packets are dropped and counted), drop
duplicates and late packets using the sequence number, count lost packets from gaps, and
print the records.  If nothing arrives for --link-timeout seconds it reports LINK LOST, so
a dashboard never shows old values as if they were live.  --json prints one JSON object per
packet (feed these to your website backend / database).
"""
import argparse
import json
import socket
import time

from decode_packet import decode


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--link-timeout", type=float, default=5.0)
    ap.add_argument("--max-packets", type=int, default=0, help="exit after N accepted packets (testing)")
    ap.add_argument("--idle-exit", type=float, default=0.0, help="exit after this many idle seconds (testing)")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((a.bind, a.port))
    sock.settimeout(0.5)
    last_seq = None
    stats = {"received": 0, "lost": 0, "duplicates": 0, "late": 0, "damaged": 0}
    last_rx = None
    link_up = False
    idle_start = time.monotonic()

    while True:
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            now = time.monotonic()
            if last_rx is not None and link_up and now - last_rx > a.link_timeout:
                link_up = False
                print(f"LINK LOST (no packet for {a.link_timeout:.0f} s)", flush=True)
            if a.idle_exit and now - idle_start > a.idle_exit:
                break
            continue
        idle_start = time.monotonic()
        try:
            pkt = decode(data)
        except ValueError as e:
            stats["damaged"] += 1
            print(f"dropped damaged packet from {addr[0]}: {e}", flush=True)
            continue
        seq = pkt["seq"]
        if last_seq is not None:
            diff = (seq - last_seq) & 0xFFFF
            if diff == 0:
                stats["duplicates"] += 1
                continue
            if diff >= 0x8000:
                stats["late"] += 1
                continue
            stats["lost"] += diff - 1
        last_seq = seq
        stats["received"] += 1
        last_rx = time.monotonic()
        if not link_up:
            link_up = True
            print("LINK UP", flush=True)
        if a.json:
            print(json.dumps(pkt), flush=True)
        else:
            parts = [f"ch{r['channel']}={'--' if r['value'] is None else format(r['value'], '.3f')}"
                     f"[{r['quality']}{'/' + ','.join(r['flags']) if r['flags'] else ''}]" for r in pkt["records"]]
            print(f"seq {seq:5d} site {pkt['site']}  " + "  ".join(parts), flush=True)
        if a.max_packets and stats["received"] >= a.max_packets:
            break
    print("stats:", stats, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
