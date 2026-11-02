"""udp_listener.py - runs on the laptop (base station).

Receives telemetry datagrams from the Pi, decodes them with decode_packet.py,
and tracks lost / duplicate packets from the sequence numbers.

Put this file in the same folder as decode_packet.py.
run: python3 udp_listener.py [port]
"""
import socket
import sys
import time

from decode_packet import decode

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5005
NAMES = {0: "moisture", 1: "temp", 2: "pH"}

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))
print(f"listening on UDP port {PORT}, Ctrl+C to stop")

last_seq = None
received = lost = dups = late = 0

try:
    while True:
        data, addr = sock.recvfrom(256)
        try:
            pkt = decode(data)
        except ValueError as e:
            print(f"bad packet from {addr[0]}: {e}")
            continue

        seq = pkt["seq"]
        if last_seq is not None:
            diff = (seq - last_seq) & 0xFFFF
            if diff == 0:
                dups += 1
                print(f"duplicate seq={seq}")
                continue
            if diff >= 0x8000:
                late += 1
                print(f"late packet seq={seq}")
                continue
            lost += diff - 1
        last_seq = seq
        received += 1

        parts = []
        for r in pkt["records"]:
            name = NAMES.get(r["channel"], f"ch{r['channel']}")
            val = "--" if r["value"] is None else f"{r['value']:.3f}"
            flags = ("," + "|".join(r["flags"])) if r["flags"] else ""
            parts.append(f"{name}={val} [{r['quality']}{flags}]")
        print(f"{time.strftime('%H:%M:%S')} from {addr[0]} seq={seq} t={pkt['t_s']}s  "
              + "  ".join(parts)
              + f"   (rx={received} lost={lost} dup={dups})")
except KeyboardInterrupt:
    print(f"\nreceived={received} lost={lost} duplicates={dups} late={late}")
