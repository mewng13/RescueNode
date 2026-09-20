#!/usr/bin/env python3
"""Live sensor dashboard for the esp32s3 node, served on localhost.

    python3 tools/dashboard.py              # read the board over USB serial
    python3 tools/dashboard.py --demo       # fake data, no board needed
    python3 tools/dashboard.py -p /dev/cu.usbmodem101 --http-port 8080

Then open http://localhost:8000. Uses only the stdlib (no pyserial).

It parses the lines src/main.cpp prints:
    Volume_Level:12345
    Temp: 24.0 C   Humidity: 55.0 %
    Raw: 1234   Volts: 1.23 V   GAS
    Accel X:   0.01   Y:  -0.02   Z:   1.00  (g)
    Gyro X:     0.3   Y:    -0.1   Z:     0.0  (dps)
    Quat w: 0.9998 x: -0.0100 y: 0.0050 z: 0.0000
    IMU status: samples 12345  i2c_err 0  gyro_clipped 0
A JSON line such as {"temp": 24.1, "volume": 812} works too.

The serial port can only have one reader: close PlatformIO's serial monitor
before starting this, and stop this (Ctrl+C) before uploading firmware.
"""

import argparse
import fcntl
import glob
import json
import math
import os
import random
import re
import sys
import termios
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HTML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dashboard.html")

NUM = r"(-?\d+(?:\.\d+)?|-?nan|-?inf)"
VOLUME = re.compile(r"Volume_Level:\s*(-?\d+)")
TEMP = re.compile(rf"Temp:\s*{NUM}\s*C\s+Humidity:\s*{NUM}", re.I)
GAS = re.compile(r"Raw:\s*(\d+)\s+Volts:\s*([\d.]+)\s*V\s+(GAS|clean)", re.I)
ACCEL = re.compile(rf"Accel X:\s*{NUM}\s+Y:\s*{NUM}\s+Z:\s*{NUM}", re.I)
GYRO = re.compile(rf"Gyro X:\s*{NUM}\s+Y:\s*{NUM}\s+Z:\s*{NUM}", re.I)
# The board fuses gyro + accel itself and sends the result; see imuTask in src/main.cpp
QUAT = re.compile(rf"Quat w:\s*{NUM}\s+x:\s*{NUM}\s+y:\s*{NUM}\s+z:\s*{NUM}", re.I)
WHO_AM_I = re.compile(r"WHO_AM_I = 0x([0-9A-Fa-f]{2})")
# gyro_clipped > 0 means the gyro hit full scale and the fused angle is short
STATUS = re.compile(r"IMU status:\s+samples\s+(\d+)\s+i2c_err\s+(\d+)\s+gyro_clipped\s+(\d+)")


def num(text):
    """float, or None for nan/inf (the DHT prints nan when a read fails)."""
    v = float(text)
    return v if math.isfinite(v) else None


class Hub:
    """Latest readings, shared between the reader thread and browser streams."""

    def __init__(self):
        self.cond = threading.Condition()
        self.version = 0
        # One entry per node id. The gateway reports itself too, so a single
        # board on a cable is just a mesh of one.
        self.state = {"status": "starting", "port": None, "detail": None,
                      "nodes": {}, "lines": 0, "i2c_errors": 0}
        self.gateway_id = None

    def publish(self, values=None, node="0", **fields):
        with self.cond:
            now = time.time()
            if values:
                n = self.state["nodes"].setdefault(node, {"values": {}, "updated": {}})
                for key, value in values.items():
                    n["values"][key] = value
                    n["updated"][key] = now
                n["last"] = now
            self.state.update(fields)
            self.version += 1
            self.cond.notify_all()

    def local(self):
        """Which node the plain-text lines describe: the one on the cable."""
        return self.gateway_id or "0"

    def count_line(self):
        with self.cond:
            self.state["lines"] += 1

    def wait(self, seen_version, timeout):
        with self.cond:
            self.cond.wait_for(lambda: self.version != seen_version, timeout)
            return self.version, json.dumps(self.state)


def parse_line(hub, line):
    line = line.strip()
    if not line:
        return
    hub.count_line()

    if "i2cWriteReadNonStop returned Error" in line:
        hub.publish(i2c_errors=hub.state["i2c_errors"] + 1)
        return

    if line.startswith("{"):
        try:
            obj = json.loads(line)
        except ValueError:
            return
        if not isinstance(obj, dict):
            return
        node = str(obj.pop("node", 0))
        if obj.get("gateway"):
            hub.gateway_id = node
        values = {}
        for k, v in obj.items():
            if v is None or isinstance(v, bool):
                values[k] = v          # null means "the sensor failed", keep it
            elif isinstance(v, (int, float)):
                values[k] = v if math.isfinite(v) else None
        hub.publish(values=values, node=node)
        return

    if m := QUAT.search(line):
        hub.publish(values={"qw": num(m[1]), "qx": num(m[2]),
                            "qy": num(m[3]), "qz": num(m[4])}, node=hub.local())
    elif m := VOLUME.search(line):
        hub.publish(values={"volume": int(m[1])}, node=hub.local())
    elif m := TEMP.search(line):
        hub.publish(values={"temp": num(m[1]), "humidity": num(m[2])}, node=hub.local())
    elif m := GAS.search(line):
        hub.publish(values={"gas_raw": int(m[1]), "gas_volts": float(m[2]),
                            "gas": m[3].upper() == "GAS"}, node=hub.local())
    elif m := ACCEL.search(line):
        hub.publish(values={"ax": num(m[1]), "ay": num(m[2]), "az": num(m[3])}, node=hub.local())
    elif m := GYRO.search(line):
        hub.publish(values={"gx": num(m[1]), "gy": num(m[2]), "gz": num(m[3])}, node=hub.local())
    elif m := STATUS.search(line):
        hub.publish(values={"imu_samples": int(m[1]), "imu_i2c_err": int(m[2]),
                            "gyro_clipped": int(m[3])}, node=hub.local())
    elif m := WHO_AM_I.search(line):
        hub.publish(values={"who_am_i": int(m[1], 16)}, node=hub.local())


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*")) or sorted(glob.glob("/dev/cu.wchusbserial*"))
    return ports[0] if ports else None


def open_raw(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    attrs[0] = attrs[1] = attrs[3] = 0          # raw: iflag, oflag, lflag
    attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[4] = attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 5                 # 0.5 s read timeout
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def serial_loop(hub, port_arg):
    """Read the board forever, reconnecting when it's unplugged or re-flashed."""
    while True:
        port = port_arg or find_port()
        if not port or not os.path.exists(port):
            if hub.state["status"] != "no_board":
                hub.publish(status="no_board", port=port_arg, detail=None)
            time.sleep(1)
            continue
        try:
            fd = open_raw(port)
        except OSError as e:
            hub.publish(status="error", port=port, detail=str(e))
            time.sleep(2)
            continue
        try:
            # PlatformIO's monitor and esptool take this same lock
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(fd)
            if hub.state["status"] != "busy":
                hub.publish(status="busy", port=port, detail=None)
            time.sleep(2)
            continue

        hub.publish(status="connected", port=port, detail=None)
        print(f"reading {port}")
        pending = b""
        try:
            while os.path.exists(port):
                chunk = os.read(fd, 4096)
                if not chunk:
                    continue
                *lines, pending = (pending + chunk).split(b"\n")
                for raw in lines:
                    parse_line(hub, raw.decode(errors="replace"))
                pending = pending[-4096:]
        except OSError:
            pass
        finally:
            os.close(fd)
        print(f"lost {port}, waiting for the board")


def demo_loop(hub):
    """A fake three-node mesh, pushed through the real JSON parser.

    Node 1 is the gateway on the cable and reports fast; 2 and 3 arrive over the
    radio at a lower rate and a hop away, like the real thing.
    """
    hub.publish(status="demo", port=None, detail=None)
    nodes = [{"id": 1, "gateway": True,  "hops": 0, "hz": 50, "temp": 24.0, "hum": 52.0, "gas": 900.0},
             {"id": 2, "gateway": False, "hops": 1, "hz": 10, "temp": 19.5, "hum": 71.0, "gas": 2100.0},
             {"id": 3, "gateway": False, "hops": 2, "hz": 10, "temp": 27.8, "hum": 44.0, "gas": 640.0}]
    for n in nodes:
        n.update(seq=0, next=0.0, phase=random.uniform(0, 6.28))

    t0 = time.time()
    while True:
        now = time.time()
        for n in nodes:
            if now < n["next"]:
                continue
            n["next"] = now + 1.0 / n["hz"]
            n["seq"] += 1
            t = (now - t0) + n["phase"]

            loud = random.random() < 0.02
            vol = int(abs(random.gauss(3500, 1200)) +
                      (random.uniform(25000, 55000) if loud else 0))

            jolt = random.gauss(0, 0.25) if random.random() < 0.01 else 0
            ax = 0.02 * math.sin(t * 0.7) + random.gauss(0, 0.008) + jolt
            ay = -0.01 + random.gauss(0, 0.008) + jolt / 2
            az = 1.0 + random.gauss(0, 0.01) - abs(jolt)

            roll, pitch = math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az))
            yaw = t * 0.15 * (1 if n["id"] % 2 else -1)
            cr, sr = math.cos(roll / 2), math.sin(roll / 2)
            cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
            cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)

            n["temp"] += random.gauss(0, 0.05)
            n["hum"] = min(95, max(20, n["hum"] + random.gauss(0, 0.3)))
            n["gas"] = min(4095, max(200, n["gas"] + random.gauss(0, 25)))

            parse_line(hub, json.dumps({
                "node": n["id"], "seq": n["seq"], "hops": n["hops"],
                "up": int((now - t0) * 1000),
                "qw": round(cy * cp * cr + sy * sp * sr, 4),
                "qx": round(cy * cp * sr - sy * sp * cr, 4),
                "qy": round(cy * sp * cr + sy * cp * sr, 4),
                "qz": round(sy * cp * cr - cy * sp * sr, 4),
                "ax": round(ax, 3), "ay": round(ay, 3), "az": round(az, 3),
                "gx": round(math.degrees(random.gauss(0, 0.05)), 1),
                "gy": round(math.degrees(random.gauss(0, 0.05)), 1),
                "gz": round(8.6 if n["id"] % 2 else -8.6, 1),
                "volume": vol, "gas_raw": int(n["gas"]),
                "gas_volts": round(n["gas"] / 4095 * 3.3, 2),
                "gas": n["gas"] > 1500,
                "temp": round(n["temp"], 1), "humidity": int(n["hum"]),
                "still": abs(jolt) < 0.01, "clipped": False,
                "gateway": n["gateway"],
            }))
        time.sleep(0.01)


def make_handler(hub):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                with open(HTML_PATH, "rb") as f:   # re-read so edits show on refresh
                    body = f.read()
                self._send(200, "text/html; charset=utf-8", body)
            elif path == "/state":
                self._send(200, "application/json", hub.wait(-1, 0)[1].encode())
            elif path == "/events":
                self._stream()
            else:
                self._send(404, "text/plain", b"not found")

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seen = -1
            try:
                while True:
                    version, payload = hub.wait(seen, 15)
                    if version == seen:
                        self.wfile.write(b": keepalive\n\n")
                    else:
                        self.wfile.write(f"data: {payload}\n\n".encode())
                        seen = version
                    self.wfile.flush()
                    time.sleep(0.01)   # cap at ~100 updates/s per browser tab
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *args):
            pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default=None, help="serial port (default: auto-detect)")
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--demo", action="store_true", help="generate fake data instead of reading serial")
    args = ap.parse_args()

    hub = Hub()
    source = (lambda: demo_loop(hub)) if args.demo else (lambda: serial_loop(hub, args.port))
    threading.Thread(target=source, daemon=True).start()

    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.http_port), make_handler(hub))
    except OSError as e:
        sys.exit(f"Can't listen on port {args.http_port} ({e.strerror}). Try --http-port 8080.")
    server.daemon_threads = True
    print(f"dashboard: http://localhost:{args.http_port}   (Ctrl+C to stop)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
