#!/usr/bin/env python3
"""Plot and interpret an mfd_replay --dump file (VISUALIZATION ONLY).

Every number shown (measured/expected current, residual, LPF residual, peaks,
verdicts) is computed by the C++ MotorFailureDetector in mfd_replay; this never
re-implements detection. Per log it draws three stacked panels:
  1. measured current (solid) vs model expected I-r (dashed)
  2. residual r = I - I_expected
  3. LPF(residual) with the +/-delta threshold bands and any trip
Needs matplotlib (use the calib .venv). --no-show just writes the PNGs.
"""
import os, sys, glob, argparse


def parse_dump(path):
    """Return (cfg, verdicts, t, motors) where motors[m] = {'I':[], 'r':[], 'rlpf':[]}."""
    cfg, verdicts, t, motors, cols = {}, [], [], {}, None
    with open(path) as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("#"):
                if "thr=" in line:
                    for tok in line[1:].split():
                        if "=" in tok:
                            k, v = tok.split("=", 1)
                            try:
                                cfg[k] = float(v)
                            except ValueError:
                                cfg[k] = v
                elif line.startswith("# motor "):
                    p = line[len("# motor "):].split()
                    failed = (p[1] == "FAILED")
                    verdicts.append({"motor": int(p[0]), "failed": failed,
                                     "trip": float(p[3]) if failed else None, "peak": float(p[-1])})
                continue
            if cols is None:                       # header row: t,m0_I,m0_r,m0_rlpf,m1_I,...
                cols = line.split(",")[1:]
                for name in cols:
                    m = int(name.split("_")[0][1:])
                    motors.setdefault(m, {"I": [], "r": [], "rlpf": []})
                continue
            vals = line.split(",")
            t.append(float(vals[0]))
            for name, v in zip(cols, vals[1:]):
                m, field = name.split("_")
                motors[int(m[1:])][field].append(float(v))
    return cfg, verdicts, t, motors


def interpret(name, cfg, verdicts):
    thr = cfg.get("thr", 0.0)
    print(f"\n  {name}  (model I={cfg.get('a')}·u²+{cfg.get('b')}·u̇+{cfg.get('c')}, "
          f"thr {thr:.1f} A, tau {cfg.get('tau')}s, persist {cfg.get('persist')}s, gate {cfg.get('gate')})")
    tripped = []
    worst = max(verdicts, key=lambda d: d["peak"]) if verdicts else None
    for d in sorted(verdicts, key=lambda d: d["motor"]):
        if d["failed"]:
            tripped.append(d["motor"])
            v = f"FAILED @ {d['trip']:.1f}s"
        else:
            v = f"ok  (margin {thr - d['peak']:+.2f} A)"
        flag = "  <-- worst" if worst and d["motor"] == worst["motor"] else ""
        print(f"    motor[{d['motor']}]  peak {d['peak']:5.2f} A   {v}{flag}")
    if tripped:
        print(f"  -> motor(s) {tripped} tripped. On a healthy log that is a FALSE POSITIVE; "
              f"on a fault log this is the detection.")
    elif worst:
        print(f"  -> clean. Closest approach {worst['peak']:.2f} A = {100*worst['peak']/thr:.0f}% of the {thr:.1f} A threshold.")
    return bool(tripped)


def plot(name, cfg, verdicts, t, motors, outdir):
    import matplotlib.pyplot as plt
    thr = cfg.get("thr", 0.0)
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    for m, s in sorted(motors.items()):
        col = colors[m % len(colors)]
        I = s["I"]; r = s["r"]
        Ie = [a - b for a, b in zip(I, r)]                       # expected = measured - residual
        ax1.plot(t, I, lw=0.7, color=col, label=f"m{m}")
        ax1.plot(t, Ie, lw=0.7, color=col, ls="--", alpha=0.7)
        ax2.plot(t, r, lw=0.7, color=col)
        ax3.plot(t, s["rlpf"], lw=0.8, color=col)

    ax1.set_ylabel("current [A]"); ax1.set_title(f"{name}   measured (solid) vs model expected (dashed)")
    ax1.legend(ncol=6, fontsize=8); ax1.grid(alpha=0.3)

    ax2.axhline(0, color="k", lw=0.8); ax2.set_ylabel("residual  I - I_exp [A]")
    ax2.set_title("difference (raw residual)"); ax2.grid(alpha=0.3)

    if thr > 0:
        ax3.axhline(thr, color="r", ls="--", lw=1)
        ax3.axhline(-thr, color="r", ls="--", lw=1, label=f"±delta ({thr:.1f} A)")
        ax3.axhspan(-thr, thr, color="green", alpha=0.05)
    for d in verdicts:
        if d["failed"] and d["trip"] is not None:
            ax3.axvline(d["trip"], color="k", ls=":", lw=1)
            ax3.annotate(f"trip m{d['motor']}", (d["trip"], thr), fontsize=8)
    ax3.set_ylabel("LPF(residual) [A]"); ax3.set_xlabel("time [s]")
    ax3.set_title("difference after the LPF, with ±delta bands")
    ax3.legend(fontsize=8); ax3.grid(alpha=0.3)

    fig.tight_layout()
    png = os.path.join(outdir, f"{name}.detail.png")
    fig.savefig(png, dpi=110)
    print(f"  plot: {png}")
    return fig


def main():
    ap = argparse.ArgumentParser(description="Plot/interpret mfd_replay --dump files.")
    ap.add_argument("dumps", nargs="+", help="dump files/globs from mfd_replay --dump")
    ap.add_argument("--out-dir", default=None, help="where to write PNGs (default: alongside each dump)")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    files = []
    for p in args.dumps:
        files += glob.glob(p) if any(c in p for c in "*?[") else [p]
    files = sorted(set(files))
    if not files:
        print("no dump files found.", file=sys.stderr); sys.exit(2)

    figs, any_trip = [], False
    for f in files:
        name = os.path.basename(f).rsplit(".dump", 1)[0]
        outdir = args.out_dir or os.path.dirname(os.path.abspath(f))
        os.makedirs(outdir, exist_ok=True)
        cfg, verdicts, t, motors = parse_dump(f)
        any_trip = interpret(name, cfg, verdicts) or any_trip
        figs.append(plot(name, cfg, verdicts, t, motors, outdir))

    print("\n" + ("RESULT: a motor tripped (see above)." if any_trip else "RESULT: no trips across the dumps."))
    if figs and not args.no_show:
        import matplotlib
        if matplotlib.get_backend().lower() != "agg":
            import matplotlib.pyplot as plt
            plt.show()


if __name__ == "__main__":
    main()
