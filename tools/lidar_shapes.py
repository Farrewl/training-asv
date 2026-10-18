"""tools/lidar_shapes.py — deteksi bentuk langsung dari scan lidar (cluster + buoys + gate).

Mode:
  sim → demo tanpa hardware (sama seperti shapes._selftest)
  live → baca lidar + plot + cetak daftar objek & gate tiap iterasi

Pakai:
  python tools/lidar_shapes.py --sim
  python tools/lidar_shapes.py --live --port /dev/ttyUSB0
"""

import argparse
import sys
import time

import matplotlib.pyplot as plt
import numpy as np

from lidar import rplidar, shapes


def mode_sim():
    world = [(1400, 350, 150), (1400, -350, 150), (-2000, 1500, 600)]
    angles, dists = shapes.sim_scan(world)
    objects, gates = shapes.detect(angles, dists)
    print("OBJEK:")
    for o in objects:
        print(" ", o)
    print("\nGATE:")
    for g in gates:
        print(" ", g)
    # plot
    xy = shapes.polar_to_xy(angles, dists)
    fig, ax = plt.subplots()
    ax.scatter(xy[:, 0], xy[:, 1], s=2, alpha=0.6)
    for o in objects:
        cx, cy = o["center"]
        ax.add_patch(plt.Circle((cx, cy), o["diameter_mm"] / 2.0,
                     fill=False, color="red" if o["kind"] == "buoy" else
                     "orange", linewidth=1.5))
    for g in gates:
        cx, cy = g["center"]
        ax.text(cx, cy, "GATE %.0fmm" % g["width_mm"], ha="center")
    ax.axis("equal")
    ax.grid(True)
    plt.show()


def mode_live(port, baud):
    try:
        l = rplidar.RPLidar(port, baud=baud).open()
    except Exception as e:
        print("ERROR:", e)
        sys.exit(1)
    try:
        l.set_motor(True)
        l.start_scan()
        print("Live — Ctrl-C untuk berhenti")
        for scan in l.iter_scans():
            angles = [s[0] for s in scan]
            dists = [s[1] for s in scan]
            if not angles:
                continue
            objects, gates = shapes.detect(angles, dists)
            buoys = [o for o in objects if o["kind"] == "buoy"]
            obs = [o for o in objects if o["kind"] == "obstacle"]
            print("[%d buoys, %d obstacles, %d gates]" %
                  (len(buoys), len(obs), len(gates)))
            for b in buoys:
                print("  BUOY @r=%.0fmm dir=%.0f\u00b0" %
                      (b["range_mm"], b["bearing_deg"]))
            for g in gates:
                print("  GATE @r=%.0fmm dir=%.0f\u00b0 w=%.0fmm" %
                      (g["range_mm"], g["bearing_deg"], g["width_mm"]))
            # plot cepat
            xy = shapes.polar_to_xy(angles, dists)
            plt.cla()
            plt.scatter(xy[:, 0], xy[:, 1], s=1, alpha=0.5)
            for b in buoys:
                cx, cy = b["center"]
                plt.gca().add_patch(plt.Circle((cx, cy), b["diameter_mm"] / 2.0,
                                               fill=False, color="blue",
                                               linewidth=1.2))
            for o in obs:
                cx, cy = o["center"]
                plt.gca().add_patch(plt.Circle((cx, cy), o["diameter_mm"] / 2.0,
                                               fill=False, color="red",
                                               linewidth=1.0))
            plt.axis("equal")
            plt.pause(0.15)
    except KeyboardInterrupt:
        print("stop")
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
    ap = argparse.ArgumentParser(description="Lidar shape detection")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--sim", action="store_true")
    g.add_argument("--live", action="store_true")
    args = ap.parse_args()
    if args.sim:
        mode_sim()
        return
    mode_live(args.port, args.baud)


if __name__ == "__main__":
    main()
