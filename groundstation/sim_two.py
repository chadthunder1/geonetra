#!/usr/bin/env python3
"""
Two-aircraft simulator. Speaks the exact wire format both C5s emit, so
the ground station cannot tell the difference.

    python3 gs_server.py     # terminal 1
    python3 sim_two.py       # terminal 2
    -> http://localhost:8000/
"""
import socket, json, math, time, random, argparse

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--gas-event", action="store_true", help="ramp CH4 through the alarm")
ap.add_argument("--preheat", type=int, default=15)
ap.add_argument("--only", choices=["gas", "lidar"], help="simulate one aircraft only")
a = ap.parse_args()

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
DST = (a.host, 9000)
send = lambda o: s.sendto((json.dumps(o) + "\n").encode(), DST)
B64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"

def pack(bins):
    out = []
    for i in range(0, 360, 3):
        v = (bins[i] << 16) | (bins[i+1] << 8) | bins[i+2]
        out += [B64[(v >> 18) & 63], B64[(v >> 12) & 63],
                B64[(v >> 6) & 63], B64[v & 63]]
    return "".join(out)

def room(yaw):
    """3.2 x 2.4 m room with a pillar 0.9 m out at bearing 40 deg."""
    bins = [255] * 360
    for d in range(360):
        th = math.radians(d + yaw)
        c, sn = math.cos(th), math.sin(th)
        best = 9.9
        for half, comp in ((1.6, c), (1.2, sn)):
            if abs(comp) > 1e-3:
                dist = half / abs(comp)
                if 0 < dist < best:
                    best = dist
        if abs(((d - 40 + 180) % 360) - 180) < 9:
            best = min(best, 0.9)
        best += random.gauss(0, 0.012)
        bins[d] = min(254, max(2, int(best * 1000 / 32)))
    return bins

WHICH = [a.only] if a.only else ["gas", "lidar"]
t0 = time.time()
print(f"simulating {', '.join(WHICH)} -> {DST}   ctrl-C to stop")
for w in WHICH:
    send({"t": "evt", "id": w, "kind": "BOOT", "sev": 0, "msg": "simulator started"})

seq, announced = 0, set()
try:
    while True:
        t = time.time() - t0
        seq += 1
        pre = max(0, int(a.preheat - t))
        cal = pre == 0

        for w in WHICH:
            phase = 0 if w == "gas" else 1.7
            yaw = (t * (8 if w == "gas" else 14)) % 360
            alt = 0.35 + 0.28 * (0.5 + 0.5 * math.sin(t * 0.5 + phase))
            roll = 2.2 * math.sin(t * 1.3 + phase)
            pitch = 1.6 * math.sin(t * 0.9 + phase)

            send({"t": "tel", "id": w, "seq": seq, "ms": int(t * 1000),
                  "st": "PREHEAT" if (w == "gas" and pre) else "READY",
                  "roll": round(roll, 2), "pitch": round(pitch, 2), "yaw": round(yaw, 1),
                  "gx": round(roll * 2, 1), "gy": round(pitch * 2, 1), "gz": 8.0,
                  "ax": 0.03, "ay": -0.02, "az": 1.0,
                  "alt": round(alt, 3), "altOk": 1,
                  "baroAlt": round(alt + random.gauss(0, 0.25), 2),
                  "tempC": 29.4, "presPa": 100380,
                  "hdg": round(yaw, 1), "magOk": 1,
                  "vbat": round((12.4 if w == "gas" else 12.1) - t * 0.004, 2),
                  "pre": pre if w == "gas" else 0, "imuOk": 1})

            if w == "gas":
                ch4 = 380 + 40 * math.sin(t / 4)
                co = 3 + 1.2 * math.sin(t / 3)
                if a.gas_event and t > 25:
                    ch4 += (t - 25) ** 2 * 22
                warn = 2 if (ch4 > 10000 or co > 200) else 1 if (ch4 > 5000 or co > 35) else 0
                send({"t": "gas", "id": w, "v4": 0.71, "v7": 0.55,
                      "rs4": 12.4, "rs7": 24.1,
                      "r04": 13.9 if cal else 0, "r07": 26.8 if cal else 0,
                      "ch4": round(ch4 if cal else 0),
                      "co": round(co if cal else 0, 1),
                      "warn": warn, "cal": cal})
                if cal and w not in announced:
                    announced.add(w)
                    send({"t": "evt", "id": w, "kind": "GAS_CAL", "sev": 0,
                          "msg": "R0 captured from clean air"})

            elif seq % 2 == 0:
                bins = room(yaw)
                sec = [min([b * 32 / 1000 for b in bins[i*30:(i+1)*30] if b < 255] or [0])
                       for i in range(12)]
                near = min([v for v in sec if v] or [0])
                send({"t": "lid", "id": w, "ok": 1, "near": round(near, 2), "nb": 40,
                      "s": [round(v, 2) for v in sec], "pkts": seq * 45})
                send({"t": "scan", "id": w, "res": 1, "unit": 32,
                      "z": round(alt, 3), "yaw": round(yaw, 1),
                      "roll": round(roll, 1), "pitch": round(pitch, 1),
                      "st": "BENCH", "d": pack(bins)})

            if seq % 10 == 0:
                rtt = random.randint(1800, 4200)
                send({"t": "link", "id": w,
                      "role": "ap" if w == "gas" else "sta",
                      "ap": "GEONETRA_5G", "ch": 36, "band": "5GHz",
                      "rssi": -47 if w == "gas" else -58,
                      "ip": "192.168.4.1" if w == "gas" else "192.168.4.3",
                      "pi": "192.168.4.2",
                      "tx": seq * 3, "rx": seq // 5, "drop": 0,
                      "rttUs": rtt, "cDistKm": round(299.792458 * rtt / 2000, 1),
                      "note": "software RTT is link health, not range"})

        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nstopped")
