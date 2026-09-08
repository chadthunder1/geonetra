#!/usr/bin/env python3
"""
GeoNetra - two-aircraft ground station for Raspberry Pi 5
----------------------------------------------------------
Ingests UDP telemetry from both link boards on 9000, keeps one state
block per aircraft, pushes to the dashboard over a websocket, and relays
commands back on 9001.

Commands can be addressed:  ALL:ESTOP  /  GAS:MOTORTEST  /  LIDAR:CALGYRO
The firmware ignores anything not addressed to it.

    pip install fastapi uvicorn websockets --break-system-packages
    python3 gs_server.py
    -> http://<pi-address>:8000/
"""

import asyncio, json, socket, time, argparse, collections, pathlib
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse
import uvicorn

HERE = pathlib.Path(__file__).parent
DRONES = ("gas", "lidar")

# ---------------------------------------------------------------- state ----

def blank():
    return {"tel": {}, "gas": {}, "lid": {}, "link": {}, "scan": {},
            "last_rx": 0.0, "pkts": 0,
            "hist": collections.deque(maxlen=300)}

class State:
    def __init__(self):
        self.d = {k: blank() for k in DRONES}
        self.events = collections.deque(maxlen=80)
        self.peers = {}          # drone id -> (ip, port) of its link board

    def snapshot(self):
        now = time.time()
        out = {}
        for k, v in self.d.items():
            out[k] = {
                "tel": v["tel"], "gas": v["gas"], "lid": v["lid"],
                "link": v["link"], "scan": v["scan"],
                "online": (now - v["last_rx"]) < 2.0,
                "pkts": v["pkts"],
                "hist": list(v["hist"]),
            }
        return {"drones": out,
                "events": list(self.events),
                "peers": {k: a[0] for k, a in self.peers.items()}}

S = State()
clients: set[WebSocket] = set()

# ------------------------------------------------------------- udp side ----

_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

def send_to(drone_id: str, text: str) -> bool:
    """Send one command to one aircraft's link board."""
    addr = S.peers.get(drone_id)
    if not addr:
        return False
    _sock.sendto(text.encode(), (addr[0], 9001))
    return True

def send_all(text: str) -> int:
    return sum(1 for d in list(S.peers) if send_to(d, text))

class Ingest(asyncio.DatagramProtocol):
    def connection_made(self, transport):
        print("[udp] listening on 0.0.0.0:9000")

    def datagram_received(self, data, addr):
        for line in data.decode("utf-8", "replace").splitlines():
            line = line.strip()
            if not line.startswith("{"):
                continue
            try:
                m = json.loads(line)
            except json.JSONDecodeError:
                continue
            self.handle(m, addr)

    def handle(self, m, addr):
        did = m.get("id")
        if did not in S.d:
            return

        if S.peers.get(did) != addr:
            S.peers[did] = addr
            print(f"[udp] {did} link board at {addr[0]}")
            send_to(did, "HELLO")

        v = S.d[did]
        v["last_rx"] = time.time()
        v["pkts"] += 1
        t = m.get("t")

        if t == "ping":
            # reply immediately, in software - this is the RTT being measured
            send_to(did, f"PONG {m.get('us', 0)}")
        elif t == "tel":
            v["tel"] = m
            v["hist"].append({
                "ts": round(time.time(), 2),
                "alt": m.get("alt", 0), "roll": m.get("roll", 0),
                "pitch": m.get("pitch", 0), "vbat": m.get("vbat", 0),
                "ch4": v["gas"].get("ch4", 0), "co": v["gas"].get("co", 0),
                "near": v["lid"].get("near", 0),
            })
        elif t == "gas":  v["gas"] = m
        elif t == "lid":  v["lid"] = m
        elif t == "link": v["link"] = m
        elif t == "scan": v["scan"] = m
        elif t == "evt":
            m["ts"] = time.strftime("%H:%M:%S")
            S.events.appendleft(m)
            print(f"[evt] {did}: {m.get('kind')} - {m.get('msg')}")

# ------------------------------------------------------------- web side ----

app = FastAPI()
ALLOWED = {"MOTORTEST", "CALGYRO", "ZEROALT", "ESTOP",
           "CLEAR", "SKIPPREHEAT", "BUZZ", "SCAN"}

@app.get("/", response_class=HTMLResponse)
async def index():
    return (HERE / "dashboard.html").read_text()

@app.get("/api/state")
async def api_state():
    return JSONResponse(S.snapshot())

@app.post("/api/cmd/{target}/{cmd}")
async def api_cmd(target: str, cmd: str):
    cmd, target = cmd.upper(), target.lower()
    if cmd not in ALLOWED:
        return JSONResponse({"ok": False, "err": "unknown command"}, 400)
    if target not in ("all", *DRONES):
        return JSONResponse({"ok": False, "err": "unknown target"}, 400)

    wire = f"{target.upper()}:{cmd}"
    reps = 3 if cmd == "ESTOP" else 1          # UDP is lossy; estop goes thrice
    sent = 0
    for _ in range(reps):
        sent = send_all(wire) if target == "all" else int(send_to(target, wire))
    return JSONResponse({"ok": sent > 0, "cmd": wire, "reached": sent,
                         "err": None if sent else "no link board seen yet"})

@app.websocket("/ws")
async def ws(sock: WebSocket):
    await sock.accept()
    clients.add(sock)
    try:
        await sock.send_text(json.dumps(S.snapshot()))
        while True:
            await sock.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(sock)

async def pusher():
    while True:
        await asyncio.sleep(0.1)
        if not clients:
            continue
        payload = json.dumps(S.snapshot())
        for c in list(clients):
            try:
                await c.send_text(payload)
            except Exception:
                clients.discard(c)

@app.on_event("startup")
async def startup():
    loop = asyncio.get_running_loop()
    await loop.create_datagram_endpoint(
        Ingest, local_addr=("0.0.0.0", 9000), allow_broadcast=True)
    loop.create_task(pusher())

# ------------------------------------------------------------------ main ---

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8000)
    a = ap.parse_args()
    print("GeoNetra ground station - gas + lidar")
    print(f"  dashboard  http://0.0.0.0:{a.port}/")
    print( "  telemetry  udp/9000    commands  udp/9001")
    uvicorn.run(app, host="0.0.0.0", port=a.port, log_level="warning")
