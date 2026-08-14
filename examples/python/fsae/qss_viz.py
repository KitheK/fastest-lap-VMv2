"""OpenLAP-style result plots and a fastest-lap-style HUD for a QSS lap.

The static figure follows OpenLAP.m (speed, curvature, ax/ay, pedals, steer,
GGV, colour map). The HTML HUD follows plot_run_dashboard.m: follow-cam,
throttle-coloured trail, steering wheel, G-G, per-tire cards, rolling telemetry.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Optional

from fsae.qss_channels import LapView

BG = "#0d1117"
FG = "#c9d1d9"
ORANGE = "#f5a623"
CYAN = "#00ffff"
MAGENTA = "#ff00ff"
GREEN = "#3fb950"
RED = "#f85149"
GRID = "#30363d"


def _setup_mpl():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.edgecolor": "#444",
            "axes.labelcolor": "#111",
            "xtick.color": "#111",
            "ytick.color": "#111",
            "text.color": "#111",
            "font.size": 9,
        }
    )
    return plt


def plot_openlap_results(view: LapView, path: str | Path) -> Path:
    """Seven-row OpenLAP results page (distance on the x axis, GGV + map at the bottom)."""
    plt = _setup_mpl()
    from matplotlib.collections import LineCollection
    import numpy as np

    s = np.array(view.s)
    v_kmh = np.array(view.v) * 3.6
    fig = plt.figure(figsize=(12.5, 16.5), dpi=120)
    fig.suptitle(
        f"OpenLAP-style results: {view.vehicle_name} @ {view.track_name}\n"
        f"QSS lap {view.lap_time:.3f} s   min/mean/max {min(v_kmh):.1f}/{v_kmh.mean():.1f}/{max(v_kmh):.1f} km/h",
        fontsize=13,
        fontweight="bold",
    )
    gs = fig.add_gridspec(7, 2, height_ratios=[1, 1, 1, 1, 1, 1.15, 1.15], hspace=0.42, wspace=0.22)

    ax = fig.add_subplot(gs[0, :])
    ax.plot(s, v_kmh, color=ORANGE, lw=1.8, label="Speed")
    ax.plot(s, np.array(view.v_max) * 3.6, color="#888", lw=0.8, ls="--", label="Corner limit")
    ax.set_ylabel("Speed [km/h]")
    ax.set_xlim(s[0], s[-1])
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(gs[1, :])
    ax.plot(s, view.kappa, color="#0969da", label="Curvature")
    ax.set_ylabel("Curvature [1/m]")
    ax.set_xlim(s[0], s[-1])
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    ax = fig.add_subplot(gs[2, :])
    gsum = [math.hypot(a, b) for a, b in zip(view.ax, view.ay)]
    ax.plot(s, view.ax, label="LonAcc", color=GREEN)
    ax.plot(s, view.ay, label="LatAcc", color="#0969da")
    ax.plot(s, gsum, label="GSum", color="#111", ls=":")
    ax.set_ylabel("Acceleration [g]")
    ax.set_xlim(s[0], s[-1])
    ax.legend(loc="upper right", ncol=3)
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(gs[3, :])
    ax.plot(s, [t * 100 for t in view.tps], color=GREEN, label="tps")
    ax.plot(s, [b * 100 for b in view.bps], color=RED, label="bps")
    ax.set_ylabel("input [%]")
    ax.set_ylim(-5, 110)
    ax.set_xlim(s[0], s[-1])
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(gs[4, :])
    ax.plot(s, view.steer, color=ORANGE, label="Steering wheel")
    ax.plot(s, view.delta, color="#0969da", label=r"Steering $\delta$")
    ax.plot(s, view.beta, color="#8250df", label=r"Vehicle slip $\beta$")
    ax.set_ylabel("angle [deg]")
    ax.set_xlabel("Distance [m]")
    ax.set_xlim(s[0], s[-1])
    ax.legend(loc="upper right", ncol=3)
    ax.grid(True, alpha=0.3)

    ax = fig.add_subplot(gs[5:, 0])
    if view.env_ay:
        ay_p = list(view.env_ay)
        ay_m = [-a for a in ay_p]
        ax.plot(ay_p, view.env_ax_max, color=ORANGE, lw=1.2)
        ax.plot(ay_p, view.env_ax_min, color=ORANGE, lw=1.2)
        ax.plot(ay_m, view.env_ax_max, color=ORANGE, lw=1.2)
        ax.plot(ay_m, view.env_ax_min, color=ORANGE, lw=1.2, label="GGV envelope")
    sc = ax.scatter(view.ay, view.ax, c=v_kmh, s=12, cmap="plasma", zorder=3)
    fig.colorbar(sc, ax=ax, label="Speed [km/h]", fraction=0.046)
    ax.set_xlabel("LatAcc [g]")
    ax.set_ylabel("LonAcc [g]")
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(f"GGV usage  (envelope at {view.env_speed * 3.6:.0f} km/h)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower left", fontsize=8)

    ax = fig.add_subplot(gs[5:, 1])
    pts = np.column_stack([view.x, view.y])
    segs = np.concatenate([pts[:-1, None, :], pts[1:, None, :]], axis=1)
    lc = LineCollection(segs, cmap="plasma", linewidths=3)
    lc.set_array(v_kmh[:-1])
    ax.add_collection(lc)
    ax.plot(view.x[0], view.y[0], "k>", ms=10, label="Start")
    ax.set_aspect("equal")
    ax.autoscale()
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Track map")
    fig.colorbar(lc, ax=ax, label="Speed [km/h]", fraction=0.046)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.subplots_adjust(left=0.08, right=0.96, top=0.93, bottom=0.04, hspace=0.45, wspace=0.28)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    return path


def plot_hud_frame(view: LapView, path: str | Path, index: Optional[int] = None) -> Path:
    """Single fastest-lap-style HUD frame (dark, follow-cam, tire cards)."""
    plt = _setup_mpl()
    import matplotlib.pyplot as plt  # noqa: F401
    from matplotlib.patches import Polygon, Circle, Rectangle
    import numpy as np

    n = len(view.s)
    i = n // 3 if index is None else max(0, min(index, n - 1))
    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor=BG)
    ax = fig.add_axes([0.0, 0.0, 1.0, 1.0], facecolor=BG)
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)

    cx, cy, yaw = view.x[i], view.y[i], view.yaw[i]
    cam = 22.0
    c, s = math.cos(-yaw), math.sin(-yaw)

    def rot(x, y):
        dx, dy = x - cx, y - cy
        return dx * c - dy * s, dx * s + dy * c

    trail = 80
    i0 = max(0, i - trail)
    xs, ys, cols = [], [], []
    for k in range(i0, i + 1):
        rx, ry = rot(view.x[k], view.y[k])
        xs.append(rx)
        ys.append(ry)
        tps, bps = view.tps[k], view.bps[k]
        if tps >= bps:
            cols.append((0.05, 0.9 * tps + 0.1, 0.15, 1.0))
        else:
            cols.append((0.95, 0.15 * (1.0 - bps), 0.15, 1.0))
    if len(xs) > 1:
        from matplotlib.collections import LineCollection

        pts = np.column_stack([xs, ys])
        segs = np.concatenate([pts[:-1, None, :], pts[1:, None, :]], axis=1)
        lc = LineCollection(segs, colors=cols[1:], linewidths=4, zorder=2)
        ax.add_collection(lc)

    path_xy = [rot(x, y) for x, y in zip(view.x, view.y)]
    ax.plot([p[0] for p in path_xy], [p[1] for p in path_xy], color="#30363d", lw=1.0, zorder=1)

    # Car wedge pointing along +x in camera frame (yaw rotated out).
    car = Polygon(
        [[1.6, 0.0], [-1.1, 0.55], [-1.1, -0.55]],
        closed=True,
        facecolor=ORANGE,
        edgecolor="white",
        lw=0.8,
        zorder=5,
    )
    ax.add_patch(car)
    ax.set_xlim(-cam * 1.777 * 0.5, cam * 1.777 * 0.5)
    ax.set_ylim(-cam * 0.5, cam * 0.5)
    ax.set_aspect("equal")

    fig.text(0.02, 0.96, f"{view.vehicle_name}  ·  {view.track_name}", color="white", fontsize=14, fontweight="bold")
    fig.text(0.02, 0.92, f"t = {view.time[i]:.2f} s / {view.lap_time:.3f} s    {view.v[i]*3.6:.1f} km/h", color=FG, fontsize=11)

    # Pedals
    axb = fig.add_axes([0.02, 0.55, 0.13, 0.28], facecolor="#161b22")
    axb.set_xlim(0, 3)
    axb.set_ylim(0, 1.05)
    axb.add_patch(Rectangle((0.3, 0), 0.7, view.bps[i], color=RED))
    axb.add_patch(Rectangle((1.4, 0), 0.7, view.tps[i], color=GREEN))
    axb.add_patch(Rectangle((0.3, 0), 0.7, 1.0, fill=False, edgecolor=FG, lw=1.2))
    axb.add_patch(Rectangle((1.4, 0), 0.7, 1.0, fill=False, edgecolor=FG, lw=1.2))
    axb.text(0.65, -0.12, "BPS", color=RED, ha="center", fontsize=8, transform=axb.transData)
    axb.text(1.75, -0.12, "TPS", color=GREEN, ha="center", fontsize=8)
    axb.axis("off")
    axb.set_title("Pedals", color=FG, fontsize=9)

    # Steering wheel
    axs = fig.add_axes([0.16, 0.55, 0.13, 0.28], facecolor="#161b22")
    axs.set_xlim(-1.2, 1.2)
    axs.set_ylim(-1.2, 1.2)
    axs.set_aspect("equal")
    axs.add_patch(Circle((0, 0), 1.0, fill=False, edgecolor=FG, lw=2))
    ang = math.radians(view.steer[i])
    axs.plot([0, math.sin(ang)], [0, math.cos(ang)], color=ORANGE, lw=3)
    axs.axis("off")
    axs.set_title(f"Steer {view.steer[i]:+.1f}°", color=FG, fontsize=9)

    # G-G
    axg = fig.add_axes([0.02, 0.22, 0.22, 0.30], facecolor="#161b22")
    if view.env_ay:
        ay_p = view.env_ay
        ay_m = [-a for a in ay_p]
        axg.plot(ay_p, view.env_ax_max, color=ORANGE, lw=1)
        axg.plot(ay_p, view.env_ax_min, color=ORANGE, lw=1)
        axg.plot(ay_m, view.env_ax_max, color=ORANGE, lw=1)
        axg.plot(ay_m, view.env_ax_min, color=ORANGE, lw=1)
    axg.plot(view.ay[max(0, i - 40) : i + 1], view.ax[max(0, i - 40) : i + 1], color=CYAN, lw=1.2)
    axg.plot(view.ay[i], view.ax[i], "o", color=ORANGE, ms=8)
    axg.set_aspect("equal")
    axg.set_facecolor("#161b22")
    axg.tick_params(colors=FG, labelsize=7)
    axg.set_title("G-G [g]", color=FG, fontsize=9)
    for spine in axg.spines.values():
        spine.set_color(GRID)

    # Tires
    names = ("FL", "FR", "RL", "RR")
    fzs = (view.fz_fl[i], view.fz_fr[i], view.fz_rl[i], view.fz_rr[i])
    ps = (view.power_fl[i], view.power_fr[i], view.power_rl[i], view.power_rr[i])
    es = (view.energy_fl[i], view.energy_fr[i], view.energy_rl[i], view.energy_rr[i])
    fz_max = max(max(view.fz_fl), max(view.fz_fr), max(view.fz_rl), max(view.fz_rr), 1.0)
    axt = fig.add_axes([0.78, 0.22, 0.20, 0.62], facecolor="#161b22")
    axt.set_xlim(0, 1)
    axt.set_ylim(0, 4.2)
    axt.axis("off")
    axt.set_title("Tires (QSS estimate)", color=FG, fontsize=9, pad=8)
    for k, name in enumerate(names):
        y0 = 3.1 - k * 1.0
        axt.text(0.02, y0 + 0.55, name, color=ORANGE, fontsize=10, fontweight="bold")
        axt.add_patch(Rectangle((0.28, y0 + 0.50), 0.65 * fzs[k] / fz_max, 0.18, color=CYAN))
        axt.add_patch(Rectangle((0.28, y0 + 0.50), 0.65, 0.18, fill=False, edgecolor=FG, lw=0.8))
        axt.text(0.02, y0 + 0.28, f"Fz {fzs[k]:.0f} N", color=FG, fontsize=8)
        axt.text(0.02, y0 + 0.08, f"P {ps[k]/1000:.2f} kW   E {es[k]/1000:.2f} kJ", color=FG, fontsize=8)

    # Mini map
    axm = fig.add_axes([0.80, 0.02, 0.18, 0.18], facecolor="#161b22")
    axm.plot(view.x, view.y, color="#58a6ff", lw=1.5)
    axm.plot(view.x[i], view.y[i], "o", color=ORANGE, ms=6)
    axm.set_aspect("equal")
    axm.axis("off")
    axm.set_title("Map", color=FG, fontsize=8)

    # Telemetry
    axtel = fig.add_axes([0.26, 0.04, 0.50, 0.16], facecolor="#161b22")
    t0 = view.time[i] - 8.0
    j0 = 0
    for j, t in enumerate(view.time):
        if t >= t0:
            j0 = j
            break
    tt = view.time[j0 : i + 1]
    if len(tt) > 1:
        def norm(seq):
            arr = seq[j0 : i + 1]
            lo, hi = min(arr), max(arr)
            if hi - lo < 1e-9:
                return [0.5] * len(arr)
            return [(x - lo) / (hi - lo) for x in arr]

        axtel.plot(tt, norm(view.v), color=MAGENTA, lw=1.6, label="speed")
        axtel.plot(tt, [0.5 * x for x in view.tps[j0 : i + 1]], color=GREEN, lw=1.4, label="tps")
        axtel.plot(tt, [0.5 * x for x in view.bps[j0 : i + 1]], color=RED, lw=1.4, label="bps")
        axtel.plot(tt, norm(view.steer), color=CYAN, lw=1.4, label="steer")
    axtel.set_facecolor("#161b22")
    axtel.tick_params(colors=FG, labelsize=7)
    axtel.legend(loc="upper left", fontsize=7, facecolor="#161b22", edgecolor=GRID, labelcolor=FG, ncol=4)
    axtel.set_title("Telemetry (last 8 s)", color=FG, fontsize=9)
    for spine in axtel.spines.values():
        spine.set_color(GRID)

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, facecolor=fig.get_facecolor())
    plt.close(fig)
    return path


def _round_list(values, ndigits: int = 5):
    return [round(float(v), ndigits) for v in values]


def write_hud_html(view: LapView, path: str | Path) -> Path:
    """Self-contained follow-cam HUD. Plays the lap in real time in a browser."""
    payload = {
        "vehicle": view.vehicle_name,
        "track": view.track_name,
        "lapTime": round(view.lap_time, 4),
        "notes": view.notes,
        "s": _round_list(view.s, 4),
        "x": _round_list(view.x, 4),
        "y": _round_list(view.y, 4),
        "yaw": _round_list(view.yaw, 5),
        "time": _round_list(view.time, 4),
        "v": _round_list(view.v, 4),
        "ax": _round_list(view.ax, 4),
        "ay": _round_list(view.ay, 4),
        "tps": _round_list(view.tps, 4),
        "bps": _round_list(view.bps, 4),
        "steer": _round_list(view.steer, 3),
        "delta": _round_list(view.delta, 3),
        "beta": _round_list(view.beta, 3),
        "fz": {
            "fl": _round_list(view.fz_fl, 1),
            "fr": _round_list(view.fz_fr, 1),
            "rl": _round_list(view.fz_rl, 1),
            "rr": _round_list(view.fz_rr, 1),
        },
        "power": {
            "fl": _round_list(view.power_fl, 1),
            "fr": _round_list(view.power_fr, 1),
            "rl": _round_list(view.power_rl, 1),
            "rr": _round_list(view.power_rr, 1),
        },
        "energy": {
            "fl": _round_list(view.energy_fl, 1),
            "fr": _round_list(view.energy_fr, 1),
            "rl": _round_list(view.energy_rl, 1),
            "rr": _round_list(view.energy_rr, 1),
        },
        "envAy": _round_list(view.env_ay, 4),
        "envAxMax": _round_list(view.env_ax_max, 4),
        "envAxMin": _round_list(view.env_ax_min, 4),
        "envSpeed": round(view.env_speed, 3),
    }
    html = _HUD_HTML.replace("__PAYLOAD__", json.dumps(payload, separators=(",", ":")))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(html, encoding="utf-8")
    return path


_HUD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>Fastest-lap HUD</title>
<style>
  :root { --bg:#0d1117; --panel:#161b22; --fg:#c9d1d9; --muted:#8b949e; --orange:#f5a623;
          --green:#3fb950; --red:#f85149; --cyan:#00ffff; --mag:#ff00ff; --line:#30363d; }
  * { box-sizing: border-box; }
  html, body { margin:0; height:100%; background:var(--bg); color:var(--fg);
               font-family: ui-sans-serif, system-ui, sans-serif; }
  #wrap { display:grid; grid-template-columns: 1fr 320px; grid-template-rows: auto 1fr 160px;
          height:100%; gap:8px; padding:8px; }
  header { grid-column:1 / -1; display:flex; align-items:baseline; justify-content:space-between;
           padding:6px 10px; background:var(--panel); border:1px solid var(--line); border-radius:8px; }
  header h1 { font-size:18px; margin:0; font-weight:650; }
  header .meta { color:var(--muted); font-size:13px; }
  header .lap { color:var(--orange); font-variant-numeric: tabular-nums; font-size:20px; }
  #stage { position:relative; background:#010409; border:1px solid var(--line); border-radius:8px; overflow:hidden; }
  #track { width:100%; height:100%; display:block; }
  #side { display:flex; flex-direction:column; gap:8px; }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:8px; padding:8px 10px; }
  .card h2 { margin:0 0 6px; font-size:11px; letter-spacing:.08em; text-transform:uppercase; color:var(--muted); }
  #gg, #mini, #wheel { width:100%; display:block; }
  .pedals { display:flex; gap:10px; height:90px; align-items:flex-end; }
  .bar { width:36px; height:100%; border:1px solid var(--fg); position:relative; background:#010409; }
  .bar > i { position:absolute; left:0; right:0; bottom:0; }
  .tires { display:grid; grid-template-columns:1fr 1fr; gap:6px; font-family: ui-monospace, monospace; font-size:11px; }
  .tire { border:1px solid var(--line); padding:6px; border-radius:6px; }
  .tire b { color:var(--orange); }
  #telem { grid-column:1 / -1; background:var(--panel); border:1px solid var(--line); border-radius:8px; }
  #telem canvas { width:100%; height:100%; display:block; }
  button { background:#21262d; color:var(--fg); border:1px solid var(--line); border-radius:6px;
           padding:4px 10px; cursor:pointer; }
  button:hover { border-color:var(--orange); }
  .note { font-size:10px; color:var(--muted); margin-top:6px; line-height:1.35; }
</style>
</head>
<body>
<div id="wrap">
  <header>
    <div>
      <h1 id="title">HUD</h1>
      <div class="meta" id="clock">t = 0.00 s</div>
    </div>
    <div>
      <button id="play">Pause</button>
      <span class="lap" id="lap">0.000 s</span>
    </div>
  </header>
  <div id="stage"><canvas id="track"></canvas></div>
  <div id="side">
    <div class="card">
      <h2>Driver</h2>
      <div style="display:flex; gap:12px; align-items:center;">
        <div class="pedals">
          <div><div class="bar"><i id="bps" style="background:var(--red);height:0"></i></div><div style="text-align:center;color:var(--red);font-size:10px;">BPS</div></div>
          <div><div class="bar"><i id="tps" style="background:var(--green);height:0"></i></div><div style="text-align:center;color:var(--green);font-size:10px;">TPS</div></div>
        </div>
        <canvas id="wheel" width="140" height="140"></canvas>
      </div>
    </div>
    <div class="card"><h2>G-G</h2><canvas id="gg" width="280" height="200"></canvas></div>
    <div class="card">
      <h2>Tires</h2>
      <div class="tires" id="tires"></div>
      <div class="note" id="notes"></div>
    </div>
    <div class="card"><h2>Map</h2><canvas id="mini" width="280" height="140"></canvas></div>
  </div>
  <div id="telem"><canvas id="strip"></canvas></div>
</div>
<script>
const D = __PAYLOAD__;
const N = D.time.length;
let playing = true, t0 = performance.now(), tSim = 0;

function lerp(a,b,u){ return a + (b-a)*u; }
function atTime(t){
  if (t <= D.time[0]) return 0;
  if (t >= D.time[N-1]) return N-1;
  let lo=0, hi=N-1;
  while (hi-lo>1){ const m=(lo+hi)>>1; if (D.time[m] <= t) lo=m; else hi=m; }
  return lo + (t - D.time[lo]) / Math.max(1e-9, D.time[hi]-D.time[lo]);
}
function samp(arr, f){
  const i = Math.max(0, Math.min(N-2, Math.floor(f)));
  return lerp(arr[i], arr[i+1], f-i);
}

const track = document.getElementById('track');
const gg = document.getElementById('gg');
const mini = document.getElementById('mini');
const wheel = document.getElementById('wheel');
const strip = document.getElementById('strip');
function fit(c){
  const r = c.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  c.width = Math.max(1, r.width * dpr);
  c.height = Math.max(1, r.height * dpr);
  const ctx = c.getContext('2d');
  ctx.setTransform(dpr,0,0,dpr,0,0);
  return ctx;
}

function drawWheel(deg){
  const ctx = wheel.getContext('2d');
  const w = wheel.width, h = wheel.height, cx=w/2, cy=h/2, r=Math.min(w,h)*0.38;
  ctx.setTransform(1,0,0,1,0,0);
  ctx.clearRect(0,0,w,h);
  ctx.strokeStyle = '#c9d1d9'; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.arc(cx,cy,r,0,Math.PI*2); ctx.stroke();
  const a = deg * Math.PI/180;
  ctx.strokeStyle = '#f5a623'; ctx.lineWidth = 4;
  ctx.beginPath(); ctx.moveTo(cx,cy);
  ctx.lineTo(cx + r*Math.sin(a), cy - r*Math.cos(a)); ctx.stroke();
}

function drawMini(f){
  const ctx = mini.getContext('2d');
  const w = mini.width, h = mini.height;
  ctx.setTransform(1,0,0,1,0,0);
  ctx.fillStyle = '#010409'; ctx.fillRect(0,0,w,h);
  let minx=1e9,maxx=-1e9,miny=1e9,maxy=-1e9;
  for (let i=0;i<N;i++){ minx=Math.min(minx,D.x[i]); maxx=Math.max(maxx,D.x[i]);
    miny=Math.min(miny,D.y[i]); maxy=Math.max(maxy,D.y[i]); }
  const pad=12, sx=(w-2*pad)/Math.max(1e-6,maxx-minx), sy=(h-2*pad)/Math.max(1e-6,maxy-miny);
  const sc=Math.min(sx,sy);
  const X=x=>pad+(x-minx)*sc, Y=y=>h-pad-(y-miny)*sc;
  ctx.strokeStyle='#58a6ff'; ctx.lineWidth=2; ctx.beginPath();
  for (let i=0;i<N;i++){ const x=X(D.x[i]), y=Y(D.y[i]); if(i) ctx.lineTo(x,y); else ctx.moveTo(x,y); }
  ctx.stroke();
  ctx.fillStyle='#f5a623'; ctx.beginPath();
  ctx.arc(X(samp(D.x,f)), Y(samp(D.y,f)), 5, 0, Math.PI*2); ctx.fill();
}

function drawGG(f){
  const ctx = gg.getContext('2d');
  const w = gg.width, h = gg.height;
  ctx.setTransform(1,0,0,1,0,0);
  ctx.fillStyle='#010409'; ctx.fillRect(0,0,w,h);
  const lim=2.2, cx=w/2, cy=h*0.48, sc=Math.min(w,h)*0.38/lim;
  const X=ay=>cx+ay*sc, Y=ax=>cy-ax*sc;
  ctx.strokeStyle='#30363d';
  for (let g=0.5; g<=2.001; g+=0.5){
    ctx.beginPath(); ctx.arc(cx,cy,g*sc,0,Math.PI*2); ctx.stroke();
  }
  ctx.strokeStyle='#f5a623'; ctx.lineWidth=1.5;
  if (D.envAy && D.envAy.length){
    const draw=(xs,ys)=>{ ctx.beginPath(); for(let i=0;i<xs.length;i++){ const x=X(xs[i]), y=Y(ys[i]); if(i) ctx.lineTo(x,y); else ctx.moveTo(x,y);} ctx.stroke(); };
    draw(D.envAy, D.envAxMax); draw(D.envAy, D.envAxMin);
    draw(D.envAy.map(a=>-a), D.envAxMax); draw(D.envAy.map(a=>-a), D.envAxMin);
  }
  const i0 = Math.max(0, Math.floor(f)-40);
  ctx.strokeStyle='#00ffff'; ctx.beginPath();
  for (let i=i0;i<=Math.floor(f);i++){ const x=X(D.ay[i]), y=Y(D.ax[i]); if(i===i0) ctx.moveTo(x,y); else ctx.lineTo(x,y); }
  ctx.stroke();
  ctx.fillStyle='#f5a623'; ctx.beginPath();
  ctx.arc(X(samp(D.ay,f)), Y(samp(D.ax,f)), 5, 0, Math.PI*2); ctx.fill();
}

function drawTrack(f){
  const ctx = fit(track);
  const r = track.getBoundingClientRect();
  const w=r.width, h=r.height;
  ctx.fillStyle='#010409'; ctx.fillRect(0,0,w,h);
  const x=samp(D.x,f), y=samp(D.y,f), yaw=samp(D.yaw,f);
  const cam=24, aspect=w/h;
  ctx.save();
  ctx.translate(w/2, h/2);
  ctx.rotate(-yaw + Math.PI/2);
  ctx.scale(h/cam, h/cam);
  ctx.translate(-x, -y);
  ctx.strokeStyle='#21262d'; ctx.lineWidth=cam*0.015; ctx.beginPath();
  for (let i=0;i<N;i++){ if(i) ctx.lineTo(D.x[i], D.y[i]); else ctx.moveTo(D.x[i], D.y[i]); }
  ctx.stroke();
  const i = Math.floor(f);
  const i0 = Math.max(0, i-140);
  for (let k=i0;k<i;k++){
    const tps=D.tps[k], bps=D.bps[k];
    ctx.strokeStyle = tps>=bps ? `rgb(${20},${80+175*tps},${30})` : `rgb(${80+175*bps},${30},${30})`;
    ctx.lineWidth = cam*0.04;
    ctx.beginPath(); ctx.moveTo(D.x[k], D.y[k]); ctx.lineTo(D.x[k+1], D.y[k+1]); ctx.stroke();
  }
  ctx.restore();
  // car in screen center, pointing up
  ctx.save();
  ctx.translate(w/2, h/2);
  ctx.fillStyle='#f5a623'; ctx.strokeStyle='#fff'; ctx.lineWidth=1.5;
  ctx.beginPath(); ctx.moveTo(0,-18); ctx.lineTo(10,14); ctx.lineTo(0,8); ctx.lineTo(-10,14); ctx.closePath();
  ctx.fill(); ctx.stroke();
  ctx.restore();
}

function drawStrip(f){
  const ctx = fit(strip);
  const r = strip.getBoundingClientRect();
  const w=r.width, h=r.height;
  ctx.fillStyle='#010409'; ctx.fillRect(0,0,w,h);
  const tNow = samp(D.time,f), span=8;
  const t0 = tNow-span;
  function nrm(arr){
    let lo=1e9, hi=-1e9;
    for (let i=0;i<N;i++){ if (D.time[i]>=t0 && D.time[i]<=tNow){ lo=Math.min(lo,arr[i]); hi=Math.max(hi,arr[i]); } }
    if (hi-lo<1e-9) return x=>0.5;
    return x => (x-lo)/(hi-lo);
  }
  function trace(arr, color, scale){
    const nn = nrm(arr);
    ctx.strokeStyle=color; ctx.lineWidth=2; ctx.beginPath();
    let started=false;
    for (let i=0;i<N;i++){
      if (D.time[i]<t0 || D.time[i]>tNow) continue;
      const x = (D.time[i]-t0)/span * w;
      const y = h - 8 - nn(arr[i])*scale*(h-16);
      if (!started){ ctx.moveTo(x,y); started=true; } else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }
  ctx.strokeStyle='#21262d';
  for (let k=0;k<=8;k++){ const x=k/8*w; ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,h); ctx.stroke(); }
  trace(D.v, '#ff00ff', 1);
  trace(D.steer, '#00ffff', 1);
  trace(D.tps.map(x=>x*0.5), '#3fb950', 1);
  trace(D.bps.map(x=>x*0.5), '#f85149', 1);
}

function tires(f){
  const names=['fl','fr','rl','rr'];
  const labels=['FL','FR','RL','RR'];
  const el = document.getElementById('tires');
  el.innerHTML = names.map((k,i)=>{
    const fz=samp(D.fz[k],f), p=samp(D.power[k],f)/1000, e=samp(D.energy[k],f)/1e3;
    return `<div class="tire"><b>${labels[i]}</b><br/>Fz ${fz.toFixed(0)} N<br/>P ${p.toFixed(2)} kW<br/>E ${e.toFixed(2)} kJ</div>`;
  }).join('');
}

function frame(now){
  if (playing){
    tSim = (now - t0)/1000;
    if (tSim > D.lapTime){ tSim = tSim % D.lapTime; t0 = now - tSim*1000; }
  }
  const f = atTime(tSim);
  document.getElementById('clock').textContent =
    `t = ${tSim.toFixed(2)} s    ${ (samp(D.v,f)*3.6).toFixed(1) } km/h    ax ${samp(D.ax,f).toFixed(2)} g    ay ${samp(D.ay,f).toFixed(2)} g`;
  document.getElementById('tps').style.height = (samp(D.tps,f)*100)+'%';
  document.getElementById('bps').style.height = (samp(D.bps,f)*100)+'%';
  drawTrack(f); drawGG(f); drawMini(f); drawWheel(samp(D.steer,f)); drawStrip(f); tires(f);
  requestAnimationFrame(frame);
}

document.getElementById('title').textContent = D.vehicle + '  ·  ' + D.track;
document.getElementById('lap').textContent = D.lapTime.toFixed(3) + ' s';
document.getElementById('notes').textContent = D.notes;
document.getElementById('play').onclick = () => {
  playing = !playing;
  document.getElementById('play').textContent = playing ? 'Pause' : 'Play';
  if (playing) t0 = performance.now() - tSim*1000;
};
window.addEventListener('keydown', e => { if (e.code==='Space'){ e.preventDefault(); document.getElementById('play').click(); }});
requestAnimationFrame(frame);
</script>
</body>
</html>
"""


def write_summary(view: LapView, path: str | Path) -> Path:
    v_kmh = [x * 3.6 for x in view.v]
    text = (
        f"{view.vehicle_name} on {view.track_name}\n"
        f"QSS lap time: {view.lap_time:.3f} s\n"
        f"Speed: min {min(v_kmh):.1f}  mean {sum(v_kmh)/len(v_kmh):.1f}  max {max(v_kmh):.1f} km/h\n"
        f"Peak |ay|: {max(abs(a) for a in view.ay):.3f} g\n"
        f"Long. accel: {max(view.ax):.3f} g  brake: {min(view.ax):.3f} g\n"
        f"Peak |steer|: {max(abs(a) for a in view.steer):.2f} deg\n"
        f"{view.notes}\n"
    )
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path
