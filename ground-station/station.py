#!/usr/bin/env python3
"""Celestium CanSat ground station.

Receives the JSON telemetry packets sent by the CanSat over LoRa (via a
second Pico + RFM96W running a serial bridge, or any USB serial adapter)
and provides the two rule-required deliverables in real time:
  - altitude vs time
  - temperature vs altitude
Everything is also appended to a timestamped CSV for post-processing.

Usage:
    python3 station.py /dev/tty.usbmodem* [--csv flight.csv]
"""
import argparse
import csv
import json
import signal
import sys
import time
from datetime import datetime

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial missing: python3 -m pip install pyserial")

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    PLOT = True
except ImportError:
    PLOT = False  # still log to CSV without plotting

FIELDS = ["t", "T", "P", "A", "uv", "uvi", "ir", "lux", "co2", "o3",
          "la", "lo", "ga", "s", "v"]


def open_csv(path):
    fh = open(path, "a", newline="")
    w = csv.writer(fh)
    if fh.tell() == 0:
        w.writerow(FIELDS)
    return fh, w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--csv", default=f"flight_{datetime.now():%Y%m%d_%H%M%S}.csv")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=2)
    fh, writer = open_csv(args.csv)
    print(f"[*] listening on {args.port} -> {args.csv}")

    rows = {"t": [], "A": [], "T": []}
    last_pkt = 0.0

    if PLOT:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
        fig.suptitle("Celestium CanSat — live telemetry")
        ax1.set_xlabel("time (s)"); ax1.set_ylabel("altitude (m)")
        ax1.set_title("altitude vs time")
        ax2.set_xlabel("temperature (°C)"); ax2.set_ylabel("altitude (m)")
        ax2.set_title("temperature vs altitude")
        line1, = ax1.plot([], [], lw=2)
        line2, = ax2.plot([], [], ".")
        ax1.grid(); ax2.grid()

        def update(_):
            line1.set_data(rows["t"], rows["A"])
            line2.set_data(rows["T"], rows["A"])
            for ax, xs in ((ax1, rows["t"]), (ax2, rows["T"])):
                if xs:
                    ax.relim(); ax.autoscale_view()
            return line1, line2

        ani = animation.FuncAnimation(fig, update, interval=1000, cache_frame_data=False)
        signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
        plt.show(block=False)

    buf = b""
    while True:
        buf += ser.read(ser.in_waiting or 64)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            try:
                pkt = json.loads(line.decode().strip())
            except (ValueError, UnicodeDecodeError):
                continue
            if not isinstance(pkt, dict) or "t" not in pkt:
                continue
            writer.writerow([pkt.get(k, "") for k in FIELDS])
            fh.flush()
            rows["t"].append(pkt.get("t", 0) / 1000.0)
            rows["A"].append(pkt.get("A", float("nan")))
            rows["T"].append(pkt.get("T", float("nan")))
            last_pkt = time.time()
            print(f"[{last_pkt:%H:%M:%S}] t={pkt.get('t')}s alt={pkt.get('A')}m "
                  f"T={pkt.get('T')}C co2={pkt.get('co2')} uvi={pkt.get('uvi')}")
        if PLOT:
            plt.pause(0.2)


if __name__ == "__main__":
    main()
