#!/usr/bin/env python3
"""Record from the INMP441 via src/mic_dump.cpp and play it back.

    python3 tools/mic_listen.py              # 5 s, save + play
    python3 tools/mic_listen.py -t 10        # 10 s
    python3 tools/mic_listen.py -o take2.wav

Uses only the stdlib (no pyserial) and macOS's built-in afplay.
"""

import argparse
import glob
import os
import subprocess
import sys
import termios
import time
import wave

SAMPLE_RATE = 16000
BYTES_PER_SAMPLE = 2


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*")) or sorted(glob.glob("/dev/cu.wchusbserial*"))
    if not ports:
        sys.exit("No ESP32 serial port found. Plug the board into the USB (native) port.")
    return ports[0]


def open_raw(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    # Raw mode: no echo, no CR/LF translation, no XON/XOFF -- 0x11/0x13 bytes in
    # the audio would otherwise be eaten as flow control.
    attrs[0] = attrs[1] = attrs[3] = 0          # iflag, oflag, lflag
    attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 5                 # 0.5 s read timeout
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default=None)
    ap.add_argument("-t", "--seconds", type=float, default=5.0)
    ap.add_argument("-o", "--out", default="mic.wav")
    ap.add_argument("--no-play", action="store_true")
    args = ap.parse_args()

    port = args.port or find_port()
    want = int(SAMPLE_RATE * args.seconds) * BYTES_PER_SAMPLE
    print(f"port {port}  ->  {args.seconds:g} s ({want} bytes)")

    fd = open_raw(port)
    try:
        time.sleep(0.3)
        termios.tcflush(fd, termios.TCIFLUSH)   # drop boot chatter
        os.write(fd, b"r")

        buf = bytearray()
        deadline = time.time() + args.seconds + 5
        while len(buf) < want and time.time() < deadline:
            chunk = os.read(fd, 4096)
            if chunk:
                buf += chunk
                print(f"\r  {len(buf)*100//want:3d}%", end="", flush=True)
        print()
    finally:
        os.close(fd)

    if not buf:
        sys.exit("Got no data. Is mic_dump.cpp flashed (pio run -e mic -t upload)?")
    if len(buf) < want:
        print(f"warning: short read, {len(buf)}/{want} bytes")

    buf = buf[: len(buf) // BYTES_PER_SAMPLE * BYTES_PER_SAMPLE]
    with wave.open(args.out, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(BYTES_PER_SAMPLE)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(bytes(buf))

    peak = max(abs(int.from_bytes(buf[i:i+2], "little", signed=True))
               for i in range(0, len(buf), 2))
    print(f"wrote {args.out}  peak {peak}/32767")
    if peak < 100:
        print("  near-silent -- check wiring, or flip MIC_CHANNEL / raise GAIN_SHIFT in mic_dump.cpp")

    if not args.no_play:
        subprocess.run(["afplay", args.out])


if __name__ == "__main__":
    main()
