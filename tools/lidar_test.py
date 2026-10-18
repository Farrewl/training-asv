"""tools/lidar_test.py — live test RPLIDAR (2D) atau raw hex dump.

Mode:
  live      → plot 2D (matplotlib) berbasis polar, auto-stop pakai Enter
  raw       → baca scan murni tanpa plot, dump sample hex untuk identifikasi
  sim       → uji plotting pakai simulator shapes.sim_scan

Pakai:
  python tools/lidar_test.py --live --port /dev/ttyUSB0
  python tools/lidar_test.py --raw  --port /dev/ttyUSB0 --n 100
  python tools/lidar_test.py --sim
"""

import argparse
import sys
import time

from lidar import rplidar
from lidar import shapes


def mode_sim(n=600):
    import matplotlib.pyplot as plt
    world = [(1400, 300, 140), (1400, -320, 140), (-1600, 1200, 600)]
    angles, dists = shapes.sim_scan(world, n_beams=n)
    xy = shapes.polar_to_xy(angles, dists)
    fig, ax = plt.subplots()
    ax.scatter(xy[:, 0], xy[:, 1], s=2)
    ax.axis('equal')
    ax.grid(True)
    ax.set_title("SIM: scan 2D")
    plt.show()


def mode_raw(port, baud, n):
    try:
        from serial import Serial
    except ImportError:
        print("pyserial tidak ada")
        sys.exit(1)
    ser = Serial(port, baudrate=baud, timeout=5)
    try:
        l = rplidar.RPLidar(port).attach(ser)
        l.set_motor(True)
        l.start_scan()
        buf = b""
        print("baca %d sample 5-byte (hex): tekan Ctrl-C" % n)
        for i in range(n):
            chunk = ser.read(64)
            buf += chunk
            samples, buf = rplidar.parse_scan_stream(buf)
            for s in samples:
                start, ang, dist, q = s
                if dist > 0:
                    print(i, start, "ang=%.2f" % ang, "dist=%.1f" % dist,
                          "q=%d" % q, "hex:" +
                          samples[0] if False else "")  # placeholder
                break  # cetak satu per 64 byte
            # kesederhanaan: print tiap putaran tidak di-otomasi
        l.stop()
    finally:
        ser.close()


def mode_live(port, baud):
    import matplotlib.pyplot as plt
    try:
        l = rplidar.RPLidar(port, baud=baud)
        l.open()
    except Exception as e:
        print(e)
        sys.exit(1)
    try:
        l.set_motor(True)
        l.start_scan()
        print("Live plot — tekan Enter di terminal untuk berhenti")
        plt.ion()
        fig, ax = plt.subplots()
        ax.axis('equal')
        ax.set_title("LIDAR 2D")
        ax.grid(True)
        sc = None
        while True:
            for scan in l.iter_scans():
                angles = [s[0] for s in scan]
                dists = [s[1] for s in scan]
                xy = shapes.polar_to_xy(angles, dists)
                if sc is None:
                    sc = ax.scatter(xy[:, 0], xy[:, 1], s=2)
                else:
                    sc.set_offsets(xy)
                ax.relim()
                ax.autoscale_view(True, True, True)
                plt.pause(0.03)
                break  # refresh tiap frame sederhana
    except KeyboardInterrupt:
        print("berhenti manual")
    finally:
        try:
            l.stop()
        except Exception:
            pass
        try:
            l.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description="Lidar test (RPLIDAR)")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--live", action="store_true")
    g.add_argument("--raw", action="store_true")
    g.add_argument("--sim", action="store_true")
    ap.add_argument("--n", type=int, default=200)
    args = ap.parse_args()
    if args.sim:
        mode_sim(n=args.n)
        return
    if args.raw:
        mode_raw(args.port, args.baud, args.n)
        return
    mode_live(args.port, args.baud)


if __name__ == "__main__":
    main()
