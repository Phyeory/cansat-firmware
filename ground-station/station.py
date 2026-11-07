#!/usr/bin/env python3
"""Celestium CanSat ground station.

Receives the JSON telemetry packets sent by the CanSat over LoRa via the
receiver bridge (ground-station/lora_receiver) on USB serial, and provides
the two rule-required deliverables in real time:
  - altitude vs time
  - temperature vs altitude
Everything is also appended to a timestamped CSV for post-processing.
Lost packets are counted from the "n" sequence number.

Usage:
    python3 station.py /dev/tty.usbmodem*        (macOS / Linux)
    python3 station.py COM5                      (Windows)
    python3 station.py --list                    (show serial ports)
"""
import argparse
import csv
import json
import sys
import time
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial missing: python3 -m pip install pyserial")

try:
    import matplotlib.pyplot as plt
    PLOT = True
except ImportError:
    PLOT = False

FIELDS = ["rx_time", "n", "t", "T", "P", "A", "uv",
          "co2", "o3", "la", "lo", "ga", "s",
          "ax", "ay", "az", "gx", "gy", "gz", "rh", "rad",
          "v", "h", "rssi", "snr"]

HEALTH_BITS = ["BMP", "LTR", "TSL", "SCD", "GPSdata", "GPSfix", "SD",
               "BMI", "MLX", "SHT", "RAD"]


def health_str(h):
    try:
        h = int(h)
    except (TypeError, ValueError):
        return "?"
    return " ".join(name if h & (1 << i) else name.lower() + "!"
                    for i, name in enumerate(HEALTH_BITS))


def open_csv(path):
    fh = open(path, "a", newline="")
    w = csv.writer(fh)
    if fh.tell() == 0:
        w.writerow(FIELDS)
    return fh, w


def open_serial(port, baud):
    while True:
        try:
            return serial.Serial(port, baud, timeout=0.2)
        except serial.SerialException as e:
            print(f"[!] {e}  retrying in 2 s")
            time.sleep(2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?")
    ap.add_argument("--csv", default=f"flight_{datetime.now():%Y%m%d_%H%M%S}.csv")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true", help="list serial ports")
    args = ap.parse_args()

    if args.list or not args.port:
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  {p.description}")
        if not args.port:
            sys.exit("give a port, e.g. python3 station.py COM5")
        return

    ser = open_serial(args.port, args.baud)
    fh, writer = open_csv(args.csv)
    print(f"[*] listening on {args.port} -> {args.csv}")

    rows = {"t": [], "A": [], "T": []}
    first_n = last_n = None
    received = 0

    if PLOT:
        plt.ion()
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))
        fig.suptitle("Celestium CanSat: live telemetry")
        ax1.set_xlabel("time (s)"); ax1.set_ylabel("altitude (m)")
        ax1.set_title("altitude vs time")
        ax2.set_xlabel("temperature (°C)"); ax2.set_ylabel("altitude (m)")
        ax2.set_title("temperature vs altitude")
        line1, = ax1.plot([], [], lw=2)
        line2, = ax2.plot([], [], ".")
        ax1.grid(); ax2.grid()
        plt.show(block=False)

    buf = b""
    last_draw = 0.0
    try:
        while True:
            try:
                buf += ser.read(ser.in_waiting or 1)
            except serial.SerialException:
                print("[!] serial lost, reconnecting")
                ser.close()
                ser = open_serial(args.port, args.baud)
                continue

            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode(errors="replace").strip()
                if not text:
                    continue
                if text.startswith("#"):
                    print(f"    {text}")
                    continue
                try:
                    pkt = json.loads(text)
                except ValueError:
                    continue
                if not isinstance(pkt, dict) or "t" not in pkt:
                    continue

                now = datetime.now()
                pkt["rx_time"] = now.isoformat(timespec="milliseconds")
                writer.writerow([pkt.get(k, "") for k in FIELDS])
                fh.flush()

                n = pkt.get("n")
                received += 1
                if isinstance(n, int):
                    if first_n is None:
                        first_n = n
                    if last_n is not None and n > last_n + 1:
                        print(f"[!] lost {n - last_n - 1} packet(s)")
                    last_n = n
                    expected = last_n - first_n + 1
                    loss = 100.0 * (1 - received / expected) if expected > 0 else 0.0
                else:
                    loss = 0.0

                rows["t"].append(pkt.get("t", 0) / 1000.0)
                rows["A"].append(pkt.get("A", float("nan")))
                rows["T"].append(pkt.get("T", float("nan")))

                print(f"[{now:%H:%M:%S}] #{n} alt={pkt.get('A')}m T={pkt.get('T')}C "
                      f"P={pkt.get('P')}hPa co2={pkt.get('co2')} az={pkt.get('az')} "
                      f"rh={pkt.get('rh')} rad={pkt.get('rad')} "
                      f"sats={pkt.get('s')} V={pkt.get('v')} rssi={pkt.get('rssi')} "
                      f"snr={pkt.get('snr')} loss={loss:.0f}%  [{health_str(pkt.get('h'))}]")

            if PLOT and time.time() - last_draw > 0.5:
                last_draw = time.time()
                if rows["t"]:
                    line1.set_data(rows["t"], rows["A"])
                    line2.set_data(rows["T"], rows["A"])
                    for ax in (ax1, ax2):
                        ax.relim(); ax.autoscale_view()
                plt.pause(0.01)
    except KeyboardInterrupt:
        print(f"\n[*] stopped. {received} packets saved to {args.csv}")
    finally:
        fh.close()


if __name__ == "__main__":
    main()
