#!/usr/bin/env python3
"""Plot the CSV logs written by the TECS closed-loop unit tests.

The tests in TECSClosedLoopTest.cpp write one CSV per test case when
TECS_TEST_LOG_DIR is set:

    TECS_TEST_LOG_DIR=/tmp/tecs ./build/px4_sitl_test/functional-TECSClosedLoop
    python3 src/lib/tecs/plot_closed_loop.py /tmp/tecs            # interactive
    python3 src/lib/tecs/plot_closed_loop.py /tmp/tecs -o /tmp/tecs/png

Each CSV becomes one figure with eight panels on a shared time axis:
altitude, airspeed, pitch, throttle, total energy rate, energy balance rate,
altitude rate and airspeed rate. The measured rates are finite differences of
the logged states; the altitude reference and its rate are shown when the CSV contains them.
Only numpy and matplotlib are required.
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# Colours by role (fixed assignment, never cycled).
C_MEAS = "#2a78d6"    # measured / estimated state
C_SP = "#eb6834"      # setpoint / reference
C_INTEG = "#6250d6"   # integrator contribution
C_GRID = "#d8d7d2"
C_TEXT = "#52514e"

REQUIRED = ["t", "V", "h", "V_sp", "alt_sp", "pitch_cmd", "throttle",
            "pitch_integ", "throttle_integ",
            "ste_rate_sp", "ste_rate_est", "seb_rate_sp", "seb_rate_est"]
# Written by newer test builds; plotted when present.
OPTIONAL = ["alt_ref", "alt_ref_rate"]


def load(path):
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [[float(x) for x in r] for r in reader if r]

    missing = [c for c in REQUIRED if c not in header]

    if missing:
        raise ValueError(f"{path}: missing columns {missing} in header {header}")

    data = np.asarray(rows)
    return {name: data[:, i] for i, name in enumerate(header)}


def style_axis(ax, ylabel):
    ax.set_ylabel(ylabel, color=C_TEXT)
    ax.grid(True, color=C_GRID, linewidth=0.6)
    ax.tick_params(colors=C_TEXT, labelsize=8)
    ax.ticklabel_format(axis="y", useOffset=False)

    for side in ("top", "right"):
        ax.spines[side].set_visible(False)

    for side in ("left", "bottom"):
        ax.spines[side].set_color(C_GRID)


def robust_ylim(ax, *series, skip=1):
    """Set y limits from the data excluding the first `skip` samples.

    The first TECS update after reset produces transient energy-rate
    estimates that would otherwise dominate the axis.
    """
    vals = np.concatenate([s[skip:] for s in series])
    lo, hi = float(np.min(vals)), float(np.max(vals))
    pad = 0.08 * (hi - lo) if hi > lo else 1.0
    ax.set_ylim(lo - pad, hi + pad)


def plot(d, title):
    t = d["t"]
    fig, axes = plt.subplots(4, 2, figsize=(13, 11), sharex=True)
    fig.suptitle(title, fontsize=13, color="#0b0b0b")
    (ax_h, ax_v), (ax_p, ax_thr), (ax_ste, ax_seb), (ax_hr, ax_vr) = axes

    ax_h.plot(t, d["alt_sp"], color=C_SP, lw=1.6, ls="--", label="alt_sp")

    if "alt_ref" in d:
        ax_h.plot(t, d["alt_ref"], color=C_INTEG, lw=1.2, label="alt_ref")

    ax_h.plot(t, d["h"], color=C_MEAS, lw=1.8, label="h")
    style_axis(ax_h, "altitude [m]")

    ax_v.plot(t, d["V_sp"], color=C_SP, lw=1.6, ls="--", label="V_sp")
    ax_v.plot(t, d["V"], color=C_MEAS, lw=1.8, label="V")
    style_axis(ax_v, "airspeed [m/s]")

    ax_p.plot(t, np.degrees(d["pitch_cmd"]), color=C_MEAS, lw=1.8, label="pitch_cmd")
    ax_p.plot(t, np.degrees(d["pitch_integ"]), color=C_INTEG, lw=1.4, label="pitch_integ")
    style_axis(ax_p, "pitch [deg]")

    ax_thr.plot(t, d["throttle"], color=C_MEAS, lw=1.8, label="throttle")
    ax_thr.plot(t, d["throttle_integ"], color=C_INTEG, lw=1.4, label="throttle_integ")
    style_axis(ax_thr, "throttle [-]")

    ax_ste.plot(t, d["ste_rate_sp"], color=C_SP, lw=1.6, ls="--", label="ste_rate_sp")
    ax_ste.plot(t, d["ste_rate_est"], color=C_MEAS, lw=1.8, label="ste_rate_est")
    style_axis(ax_ste, "total energy rate [m²/s³]")
    robust_ylim(ax_ste, d["ste_rate_sp"], d["ste_rate_est"])

    ax_seb.plot(t, d["seb_rate_sp"], color=C_SP, lw=1.6, ls="--", label="seb_rate_sp")
    ax_seb.plot(t, d["seb_rate_est"], color=C_MEAS, lw=1.8, label="seb_rate_est")
    style_axis(ax_seb, "energy balance rate [m²/s³]")
    robust_ylim(ax_seb, d["seb_rate_sp"], d["seb_rate_est"])

    h_rate = np.gradient(d["h"], t)
    hr_series = [h_rate]

    if "alt_ref_rate" in d:
        ax_hr.plot(t, d["alt_ref_rate"], color=C_SP, lw=1.6, ls="--", label="alt_ref_rate")
        hr_series.append(d["alt_ref_rate"])

    ax_hr.plot(t, h_rate, color=C_MEAS, lw=1.8, label="dh/dt")
    style_axis(ax_hr, "altitude rate [m/s]")
    robust_ylim(ax_hr, *hr_series)

    v_rate = np.gradient(d["V"], t)
    ax_vr.plot(t, v_rate, color=C_MEAS, lw=1.8, label="dV/dt")
    style_axis(ax_vr, "airspeed rate [m/s²]")
    robust_ylim(ax_vr, v_rate)

    for ax in axes.flat:
        ax.legend(loc="best", fontsize=8, frameon=False, labelcolor=C_TEXT)

    for ax in axes[-1]:
        ax.set_xlabel("time [s]", color=C_TEXT)

    ax_h.set_xlim(t[0], t[-1])
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    return fig


def collect(paths):
    files = []

    for p in paths:
        p = Path(p)

        if p.is_dir():
            files += sorted(p.glob("*.csv"))

        elif p.suffix == ".csv":
            files.append(p)

        else:
            print(f"skipping {p}: not a CSV file or directory", file=sys.stderr)

    return files


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="CSV files or directories containing them")
    ap.add_argument("-o", "--out", type=Path,
                    help="write one PNG per CSV into this directory instead of showing")
    ap.add_argument("--dpi", type=int, default=130)
    args = ap.parse_args()

    files = collect(args.paths)

    if not files:
        sys.exit("no CSV files found")

    if args.out:
        matplotlib.use("Agg")
        args.out.mkdir(parents=True, exist_ok=True)

    for f in files:
        fig = plot(load(f), f.stem)

        if args.out:
            target = args.out / f"{f.stem}.png"
            fig.savefig(target, dpi=args.dpi)
            plt.close(fig)
            print(target)

    if not args.out:
        plt.show()


if __name__ == "__main__":
    main()
