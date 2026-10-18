"""lidar/shapes.py — deteksi bentuk dari point cloud 2D (murni numpy, tanpa hardware).

Pipeline: cluster (lompatan jarak antar-titik) -> fit lingkaran (Kasa)
-> klasifikasi per cluster -> deteksi GATE (pasangan buoy).

Kelas:
  BUOY     — lingkaran kecil, diameter dalam rentang buoy_dia_*_mm
  OBSTACLE — selain itu (dinding, dermaga, gerombolan besar)
  GATE     — pasangan buoy dengan baseline masuk rentang gate_width_*_mm

Selftest (dunia simulasi):  python -m lidar.shapes
"""

import math

import numpy as np

DEFAULTS = dict(
    cluster_gap_mm=180.0,      # lompatan antar-titik berurutan = retak cluster
    min_cluster_points=5,      # cluster lebih kecil dari ini diabaikan
    buoy_dia_min_mm=80.0,
    buoy_dia_max_mm=700.0,
    max_fit_residual_mm=45.0,  # rata-rata |selisih| titik terhadap lingkaran fit
    gate_width_min_mm=400.0,
    gate_width_max_mm=4000.0,
)


def polar_to_xy(angles_deg, dists_mm):
    a = np.radians(np.asarray(angles_deg, dtype=float))
    d = np.asarray(dists_mm, dtype=float)
    return np.stack([d * np.cos(a), d * np.sin(a)], axis=1)


def cluster_scan(angles_deg, dists_mm, gap_mm=None, min_points=None):
    """Pisah scan jadi cluster (list of Nx2 xy)."""
    gap = DEFAULTS["cluster_gap_mm"] if gap_mm is None else gap_mm
    minp = DEFAULTS["min_cluster_points"] if min_points is None else min_points
    xy = polar_to_xy(angles_deg, dists_mm)
    if len(xy) == 0:
        return []
    step = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    brk = np.where(step > gap)[0] + 1
    groups = np.split(np.arange(len(xy)), brk)
    return [xy[g] for g in groups if len(g) >= minp]


def fit_circle(xy):
    """Least-squares algebraik (Kasa). Return (cx, cy, r, residual_mm)."""
    x, y = xy[:, 0], xy[:, 1]
    A = np.stack([2 * x, 2 * y, np.ones_like(x)], axis=1)
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0], sol[1]
    r = math.sqrt(max(sol[2] + cx * cx + cy * cy, 0.0))
    res = float(np.mean(np.abs(np.hypot(x - cx, y - cy) - r)))
    return float(cx), float(cy), float(r), res


def classify_cluster(xy, cfg=None):
    """Cluster Nx2 -> dict objek (kind, center, diameter, bearing, ...)."""
    c = dict(DEFAULTS)
    if cfg:
        c.update(cfg)
    cx, cy, r, res = fit_circle(xy)
    dia = 2.0 * r
    buoy = (res <= c["max_fit_residual_mm"]
            and c["buoy_dia_min_mm"] <= dia <= c["buoy_dia_max_mm"])
    return dict(
        kind="buoy" if buoy else "obstacle",
        center=(cx, cy),
        diameter_mm=dia,
        residual_mm=res,
        n_points=int(len(xy)),
        range_mm=math.hypot(cx, cy),
        bearing_deg=math.degrees(math.atan2(cy, cx)) % 360.0,
    )


def detect_gate(objects, cfg=None):
    """Pasangan buoy -> gate. Return list[dict]."""
    c = dict(DEFAULTS)
    if cfg:
        c.update(cfg)
    buoys = [o for o in objects if o["kind"] == "buoy"]
    gates = []
    for i in range(len(buoys)):
        for j in range(i + 1, len(buoys)):
            a, b = buoys[i], buoys[j]
            base = math.dist(a["center"], b["center"])
            if c["gate_width_min_mm"] <= base <= c["gate_width_max_mm"]:
                mx = (a["center"][0] + b["center"][0]) / 2.0
                my = (a["center"][1] + b["center"][1]) / 2.0
                gates.append(dict(
                    center=(mx, my),
                    width_mm=base,
                    bearing_deg=math.degrees(math.atan2(my, mx)) % 360.0,
                    range_mm=math.hypot(mx, my),
                ))
    return gates


def detect(angles_deg, dists_mm, **overrides):
    """Scan -> (objects, gates). Override key mengikuti DEFAULTS."""
    cfg = dict(DEFAULTS)
    cfg.update(overrides)
    clusters = cluster_scan(angles_deg, dists_mm,
                            cfg["cluster_gap_mm"], cfg["min_cluster_points"])
    objects = [classify_cluster(xy, cfg) for xy in clusters]
    return objects, detect_gate(objects, cfg)


# ========================= simulator =========================
def sim_scan(objects_world, n_beams=720, fov_deg=360.0, noise_mm=4.0,
             ceiling_mm=12000.0, seed=7):
    """Ray-cast lingkaran -> scan sintetis (angles_deg, dists_mm).

    objects_world: list of (cx_mm, cy_mm, radius_mm).
    """
    rng = np.random.default_rng(seed)
    angles = np.linspace(0.0, fov_deg, n_beams, endpoint=False)
    dists = np.full(n_beams, ceiling_mm, dtype=float)
    cos_a = np.cos(np.radians(angles))
    sin_a = np.sin(np.radians(angles))
    for cx, cy, rad in objects_world:
        # ray |o + t u - c| = rad, o = 0 -> t^2 - 2t(u.c) + |c|^2 - rad^2 = 0
        bb = 2.0 * (cos_a * cx + sin_a * cy)
        cc = cx * cx + cy * cy - rad * rad
        disc = bb * bb - 4.0 * cc
        t = np.where(disc >= 0.0, (bb - np.sqrt(np.maximum(disc, 0.0))) / 2.0,
                     np.inf)
        t = np.where(t > 0.0, t, np.inf)
        dists = np.minimum(dists, t)
    dists = dists + rng.normal(0.0, noise_mm, n_beams)
    return angles, np.clip(dists, 150.0, ceiling_mm)


# ========================= selftest =========================
def _selftest():
    # Dunia: 2 buoy di depan (calon gate), 1 obstacle besar di kiri-belakang
    world = [
        (1500.0, 350.0, 150.0),    # buoy kanan-depan
        (1500.0, -350.0, 150.0),   # buoy kiri-depan
        (-2000.0, 1500.0, 600.0),  # obstacle
    ]
    angles, dists = sim_scan(world)
    objects, gates = detect(angles, dists)

    buoys = [o for o in objects if o["kind"] == "buoy"]
    obs = [o for o in objects if o["kind"] == "obstacle"]
    assert len(buoys) == 2, objects
    assert len(obs) >= 1, objects  # bisa ada artefak tepi → toleran
    assert len(gates) == 1, gates
    # pusat gate harus ~1500mm di depan, lebar ~700mm
    g = gates[0]
    assert abs(g["range_mm"] - 1500.0) < 200.0, g
    assert abs(g["width_mm"] - 700.0) < 200.0, g
    print("selftest OK: buoy=%d obstacle=%d gate=1 (range=%.0fmm lebar=%.0fmm)"
          % (len(buoys), len(obs), g["range_mm"], g["width_mm"]))


if __name__ == "__main__":
    _selftest()
