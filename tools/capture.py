"""Capture ESP32 serial output to logs/ for a fixed time.

Usage: python tools/capture.py [--port COM8] [--seconds 40] [--no-reset] [--send "c 3" ...]
Commands given with --send are written to the board after --send-delay seconds.
A command written as "@<seconds> <cmd>" is sent at that time instead, e.g. --send "@40 w 05 96 00".
"""
import argparse
import datetime
import pathlib
import sys
import time

import serial

ap = argparse.ArgumentParser()
ap.add_argument("--port", default="COM8")
ap.add_argument("--seconds", type=float, default=40)
ap.add_argument("--no-reset", action="store_true")
ap.add_argument("--send", action="append", default=[])
ap.add_argument("--send-delay", type=float, default=12)
args = ap.parse_args()

logdir = pathlib.Path(__file__).resolve().parent.parent / "logs"
logdir.mkdir(exist_ok=True)
logfile = logdir / f"capture-{datetime.datetime.now():%Y%m%d-%H%M%S}.log"

def open_port():
    s = serial.Serial(None, 115200, timeout=0.2)
    s.port = args.port
    s.dtr = False  # set before open so opening the port does not reset the chip
    s.rts = False
    s.open()
    return s


ser = open_port()
if not args.no_reset:
    # ESP32 USB Serial/JTAG: toggling RTS with DTR low resets the chip.
    ser.dtr = False
    ser.rts = True
    time.sleep(0.1)
    ser.rts = False
    ser.close()
    for _ in range(30):  # USB re-enumerates after reset
        time.sleep(0.2)
        try:
            ser = open_port()
            break
        except serial.SerialException:
            pass

start = time.time()
pending = []
for cmd in args.send:
    if cmd.startswith("@"):
        at, cmd = cmd[1:].split(" ", 1)
        pending.append((float(at), cmd))
    else:
        pending.append((args.send_delay, cmd))
pending.sort(key=lambda p: p[0])
with open(logfile, "w", encoding="utf-8") as f:
    buf = b""
    while time.time() - start < args.seconds:
        if pending and time.time() - start > pending[0][0]:
            cmd = pending.pop(0)[1]
            ser.write((cmd + "\n").encode())
            f.write(f">>> {cmd}\n")
            print(f">>> {cmd}")
        buf += ser.read(4096)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = f"{time.time() - start:7.2f} {line.decode(errors='replace').rstrip()}"
            f.write(text + "\n")
            print(text)
        sys.stdout.flush()

print(f"Saved to {logfile}")
