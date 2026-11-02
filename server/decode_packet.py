"""decode_packet.py - decodes the rover's binary telemetry packet (see telemetry_packet.h).

Pure standard library.  Use decode() from the base-station/website backend:
    from decode_packet import decode
    pkt = decode(datagram_bytes)      # raises ValueError if damaged or unknown
CLI:  python3 decode_packet.py <hex>        or    echo <hex> | python3 decode_packet.py
"""
import json
import struct
import sys

VERSION = 1
HEADER = struct.Struct("<BBHIBBB")      # ver|type, node, seq, t_s, mission_state, site, n_records
RECORD = struct.Struct("<BBBiH")        # channel, quality, flags, value*1000, sigma*1000
CRC_LEN = 2
NO_VALUE = -2**31
NO_SIGMA = 0xFFFF
QUALITY = {0: "UNKNOWN", 1: "VALID", 2: "STALE", 3: "INVALID"}
FLAG_BITS = [(0, "OUT_OF_RANGE"), (1, "RATE"), (2, "OUTLIER"), (3, "STUCK"),
             (4, "SENSOR_ERR"), (5, "NOT_FINITE"), (6, "STALE"), (7, "RESET")]


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def decode(buf: bytes) -> dict:
    if len(buf) < HEADER.size + CRC_LEN:
        raise ValueError("packet too short")
    ver_type, node, seq, t_s, state, site, n = HEADER.unpack_from(buf, 0)
    need = HEADER.size + n * RECORD.size + CRC_LEN
    if n > 8 or len(buf) != need:
        raise ValueError("length mismatch")
    if struct.unpack_from("<H", buf, need - CRC_LEN)[0] != crc16(buf[:need - CRC_LEN]):
        raise ValueError("CRC mismatch")
    if ver_type >> 4 != VERSION:
        raise ValueError("unsupported version")
    records = []
    for i in range(n):
        ch, q, flags, v, s = RECORD.unpack_from(buf, HEADER.size + i * RECORD.size)
        records.append({
            "channel": ch,
            "quality": QUALITY.get(q, f"?{q}"),
            "flags": [name for bit, name in FLAG_BITS if flags & (1 << bit)],
            "value": None if v == NO_VALUE else v / 1000.0,      # None, never 0, when missing
            "sigma": None if s == NO_SIGMA else s / 1000.0,
        })
    return {"type": "telemetry" if ver_type & 0x0F == 1 else "heartbeat", "node": node, "seq": seq,
            "t_s": t_s, "mission_state": state, "site": site, "records": records}


def _expect_sample(p: dict) -> None:
    """Checks the packet produced by gen_sample_packet.c."""
    assert (p["node"], p["seq"], p["t_s"], p["mission_state"], p["site"]) == (1, 513, 3600, 8, 2), p
    r = p["records"]
    assert abs(r[0]["value"] - 31.457) < 1e-9 and abs(r[0]["sigma"] - 0.25) < 1e-9 and r[0]["quality"] == "VALID"
    assert r[1]["value"] == -3.5 and r[1]["flags"] == ["OUTLIER"]
    assert r[2]["value"] is None and r[2]["sigma"] is None and r[2]["quality"] == "UNKNOWN"
    assert crc16(b"123456789") == 0x29B1


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    hex_in = args[0] if args else sys.stdin.read().strip()
    pkt = decode(bytes.fromhex(hex_in))
    if "--expect-sample" in sys.argv:
        _expect_sample(pkt)
        print("python decoder matches the C packet: OK")
    else:
        print(json.dumps(pkt, indent=2))
