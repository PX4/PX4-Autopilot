#!/usr/bin/env python3
"""Migrate a board's default.px4board to the target-class layout.

Splits boards/<v>/<b>/default.px4board into:

  * boards/<v>/<b>/base.px4board   -- vehicle-agnostic foundation (everything
                                      that is NOT a vehicle-class controller and
                                      not already provided by the class base).
  * boards/<v>/<b>/<class>.px4board -- the board's overlay for its class: the
                                      delta between the board's controllers and
                                      target_classes/<class>.px4board. For a board
                                      whose controller set matches the class base
                                      exactly this file only carries an "inherits"
                                      comment.

The resolved target then merges, in order (cmake/kconfig.cmake):

    base.px4board -> target_classes/<class>.px4board -> <class>.px4board

which, by construction, reproduces the original default.px4board configuration
(verified symbol-by-symbol before anything is written). default.px4board is left
in place so the legacy `<board>_default` target keeps building during the
transition.

The class(es) are inferred from the enabled controllers (and ROMFSROOT for
cannode). The air controller families collapse to one class (policy A); each extra
vehicle class becomes its own target sharing the same base:

    romfsroot == "cannode"      -> cannode
    MC only                     -> copter
    FW only                     -> plane
    MC + FW + VTOL              -> vtol   (the air superset)
    MC + FW + VTOL + UUV        -> vtol AND uuv      (two targets, shared base)
    MC + FW + VTOL + AIRSHIP    -> vtol AND airship
    ROVER / UUV / AIRSHIP only  -> rover / uuv / spacecraft

For a multi-class board, base = default minus ALL controllers, and each class gets
its own overlay; the per-class targets are strict subsets whose UNION reproduces
the original default (verified before writing).

A Linux SBC flight controller (PLATFORM_POSIX + BOARD_LINUX_TARGET, carrying
vehicle controllers) joins the single all-vehicle `linux` class. SITL, the
muorb/QURT boards (voxl2), IO firmware and other no-controller/companion
targets are reported as MANUAL and never written -- they need a
`sitl`/`io`/`ros2` class decision, not a vehicle-controller split, and the
script refuses to guess.

Usage:
    Tools/migrate_px4board.py [--apply] [--force] [BOARD ...]

BOARD may be `vendor/model`, a board directory, or a default.px4board path; with
no BOARD argument every boards/*/*/default.px4board is processed. Without
--apply the script only reports (dry run). --force overwrites an existing
base.px4board (otherwise such boards are skipped as already-migrated).
"""
import argparse
import glob
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# A vehicle-class controller module: these move out of the board's base and into
# the shared target_classes/<class>.px4board. CONTROL_ALLOCATOR is intentionally
# NOT a controller here -- it is the vehicle-agnostic actuator stage and stays in
# base (every class needs it).
_CTRL_RE = re.compile(r"^MODULES_(MC|FW|VTOL|ROVER|UUV|AIRSHIP)_")


def is_controller(sym):
    return bool(_CTRL_RE.match(sym)) or sym == "MODULES_SPACECRAFT"


def controller_family(sym):
    if sym == "MODULES_SPACECRAFT":
        return "spacecraft"
    m = _CTRL_RE.match(sym)
    return m.group(1).lower() if m else None


# The air controller families collapse into one class (policy A: a generalist air
# board is a single vtol target, not separate copter/plane/vtol targets).
_AIR = {"mc", "fw", "vtol"}

# Class -> the controller families it owns. A board that carries an extra vehicle
# class on top of the air stack (e.g. vtol+uuv) is split into one target per
# class, each claiming its own class base. cannode owns no controllers (its base
# is the cannode romfs + uavcannode driver). linux is the all-vehicle Linux SBC
# binary (every family in one target, like sitl), so it owns them all.
_CLASS_FAMILIES = {
    "copter": {"mc"},
    "plane": {"fw"},
    "vtol": {"mc", "fw", "vtol"},
    "rover": {"rover"},
    "uuv": {"uuv"},
    "airship": {"airship"},
    "spacecraft": {"spacecraft"},
    "linux": {"mc", "fw", "vtol", "rover", "uuv", "airship", "spacecraft"},
    "cannode": set(),
}


class Fragment:
    """A parsed .px4board: ordered raw lines plus the symbol->token deltas.

    `assignments[sym]` is the right-hand side string for `CONFIG_<sym>=<rhs>`
    lines, or the sentinel UNSET for `# CONFIG_<sym> is not set` lines.
    """

    UNSET = "__UNSET__"
    _SET_RE = re.compile(r"^CONFIG_([A-Za-z0-9_]+)=(.*)$")
    _UNSET_RE = re.compile(r"^# CONFIG_([A-Za-z0-9_]+) is not set$")

    def __init__(self, lines):
        self.lines = list(lines)
        self.assignments = {}
        for line in self.lines:
            sym, val = self._parse(line)
            if sym is not None:
                self.assignments[sym] = val

    @classmethod
    def from_file(cls, path):
        with open(path) as f:
            return cls(f.read().splitlines())

    @classmethod
    def _parse(cls, line):
        m = cls._SET_RE.match(line.strip())
        if m:
            return m.group(1), m.group(2)
        m = cls._UNSET_RE.match(line.strip())
        if m:
            return m.group(1), cls.UNSET
        return None, None

    @classmethod
    def sym_of(cls, line):
        return cls._parse(line)[0]


OFF = "__off__"


def effective(fragments):
    """Net symbol -> normalized value after layering fragments (later wins).

    `n` and an explicit unset both normalize to OFF so the two encodings of
    "disabled" compare equal.
    """
    out = {}
    for frag in fragments:
        for sym, val in frag.assignments.items():
            out[sym] = OFF if val in (Fragment.UNSET, "n") else val
    return out


def infer_classes(assignments):
    """Return (classes, families, reason). `classes` is the sorted list of target
    classes the board splits into (one per vehicle class, air collapsed to vtol);
    an empty list means manual handling is required and `reason` explains why."""
    if "PLATFORM_POSIX" in assignments:
        if assignments.get("BOARD_LINUX_TARGET") == "y":
            # A Linux SBC flight controller: one all-vehicle binary that selects
            # its airframe at boot (via init.d-posix), so it joins the single
            # `linux` class. muorb/QURT boards (voxl2) run the controllers on a
            # DSP and carry none on the apps side -> they do not fit a
            # controller-providing class; leave them for manual handling.
            if any(v == "y" and is_controller(s) for s, v in assignments.items()):
                return ["linux"], [], None
            return [], [], "linux board with no apps-side controllers (muorb/QURT)"
        return [], [], "posix/sitl board (needs the sitl class)"
    if assignments.get("BOARD_ROMFSROOT") == '"cannode"':
        return ["cannode"], [], None

    families = set()
    for sym, val in assignments.items():
        if val == "y" and is_controller(sym):
            families.add(controller_family(sym))
    if not families:
        return [], [], "no controllers and not a cannode romfs"

    classes = set()
    air = families & _AIR
    if air == {"mc"}:
        classes.add("copter")
    elif air == {"fw"}:
        classes.add("plane")
    elif air == {"mc", "fw", "vtol"}:
        classes.add("vtol")
    elif air:
        return [], sorted(families), "partial air controller set %s" % sorted(air)
    # Each non-air vehicle class becomes its own target alongside the air target.
    classes |= families - _AIR
    return sorted(classes), sorted(families), None


def _overlay_lines(default, common, scope):
    """The overlay lines that make base + common reproduce `default` over the
    symbols in `scope` (the class base's symbols plus the board's controllers for
    that class)."""
    lines = []
    for sym in sorted(scope):
        desired = default.assignments.get(sym)          # rhs, UNSET, or None
        provided = common.assignments.get(sym)          # rhs or None
        prov_off = provided in (None, "n", Fragment.UNSET)
        if desired is None or desired in ("n", Fragment.UNSET):
            # Board does not enable this symbol; revert if the class base set it.
            if not prov_off:
                lines.append("# CONFIG_%s is not set" % sym)
        elif provided != desired:
            # Board enables it; add it unless the class base already provides it.
            lines.append("CONFIG_%s=%s" % (sym, desired))
    return lines


def migrate_board(default_path, apply, force):
    """Plan (and optionally write) the split for one board. Returns a result
    dict for the report."""
    board_dir = os.path.dirname(default_path)
    rel = os.path.relpath(board_dir, os.path.join(REPO_ROOT, "boards"))
    result = {"board": rel}

    default = Fragment.from_file(default_path)
    classes, families, reason = infer_classes(default.assignments)
    result["classes"] = classes
    result["families"] = families
    if not classes:
        result["status"] = "manual"
        result["reason"] = reason
        return result

    commons = {}
    for cls in classes:
        cpath = os.path.join(REPO_ROOT, "target_classes", cls + ".px4board")
        if not os.path.exists(cpath):
            result["status"] = "manual"
            result["reason"] = "no target_classes/%s.px4board" % cls
            return result
        commons[cls] = Fragment.from_file(cpath)

    base_path = os.path.join(board_dir, "base.px4board")
    if os.path.exists(base_path) and not force:
        result["status"] = "skip"
        result["reason"] = "base.px4board already exists (use --force)"
        return result

    # base = default minus every controller and every symbol any of the board's
    # class bases provide. The remainder (board hardware, support modules,
    # non-controller disables) is preserved verbatim and shared by all of the
    # board's per-class targets.
    controllers = {s for s, v in default.assignments.items()
                   if v == "y" and is_controller(s)}
    common_syms = set().union(*(set(c.assignments) for c in commons.values()))
    strip = common_syms | controllers
    base_lines = [ln for ln in default.lines if Fragment.sym_of(ln) not in strip]

    # One overlay per class: the delta over the class base's symbols plus the
    # board's controllers that belong to that class.
    overlays = {}
    for cls in classes:
        cls_controllers = {s for s in controllers
                           if controller_family(s) in _CLASS_FAMILIES[cls]}
        scope = set(commons[cls].assignments) | cls_controllers
        overlays[cls] = _overlay_lines(default, commons[cls], scope)

    # Pre-write gate: base plus every (class base + its overlay) layered together
    # must reproduce default. For a multi-class board this is the UNION of its
    # per-class targets (each individual target is a strict subset by design).
    layers = [Fragment(base_lines)]
    for cls in classes:
        layers.append(commons[cls])
        layers.append(Fragment(overlays[cls]))

    # Symbols a class base force-enables (controllers, plus extras like the
    # cannode DRIVERS_UAVCANNODE). A board that does not want one omits it from
    # its default; its overlay then reverts it to an explicit off, so the merged
    # config shows off where the original relied on the Kconfig default. That is
    # equivalent because every such symbol is `default n` -- the same assumption
    # the vehicle controllers already rely on (they are class-base-provided too).
    class_provided_on = {s for c in commons.values()
                         for s, v in c.assignments.items()
                         if v not in (Fragment.UNSET, "n")}

    want = effective([default])
    got = effective(layers)
    reverted = []
    unsafe = {}
    for s in set(want) | set(got):
        if want.get(s) == got.get(s):
            continue
        if s in class_provided_on and want.get(s) is None and got.get(s) == OFF:
            reverted.append(s)
        else:
            unsafe[s] = (want.get(s), got.get(s))
    if unsafe:
        result["status"] = "error"
        result["reason"] = "merge mismatch: %s" % unsafe
        return result

    result["status"] = "ok"
    result["reverted"] = sorted(reverted)
    result["moved"] = sorted(strip & set(default.assignments))
    result["overlays"] = overlays
    result["targets"] = ["%s_%s" % (rel.replace("/", "_"), cls) for cls in classes]

    if apply:
        with open(base_path, "w") as f:
            f.write("\n".join(base_lines) + ("\n" if base_lines else ""))
        for cls in classes:
            with open(os.path.join(board_dir, cls + ".px4board"), "w") as f:
                if overlays[cls]:
                    f.write("\n".join(overlays[cls]) + "\n")
                else:
                    # The build needs the label fragment to exist; document that an
                    # empty overlay simply inherits the class base (kconfig.cmake
                    # merges base -> target_classes/<class> -> this file).
                    f.write("# Board overrides for the %s target go here; "
                            "inherits target_classes/%s.px4board.\n" % (cls, cls))
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("boards", nargs="*",
                    help="vendor/model, board dir, or default.px4board path")
    ap.add_argument("--apply", action="store_true", help="write files (default: dry run)")
    ap.add_argument("--force", action="store_true", help="overwrite existing base.px4board")
    args = ap.parse_args()

    if args.boards:
        paths = []
        for b in args.boards:
            if b.endswith("default.px4board"):
                paths.append(os.path.abspath(b))
            else:
                cand = os.path.join(REPO_ROOT, "boards", b, "default.px4board")
                if not os.path.exists(cand):
                    cand = os.path.join(os.path.abspath(b), "default.px4board")
                paths.append(cand)
    else:
        paths = sorted(glob.glob(os.path.join(REPO_ROOT, "boards", "*", "*", "default.px4board")))

    results = [migrate_board(p, args.apply, args.force) for p in paths if os.path.exists(p)]

    by_status = {}
    for r in results:
        by_status.setdefault(r["status"], []).append(r)

    verb = "Wrote" if args.apply else "Would write"
    for r in by_status.get("ok", []):
        tgts = ", ".join(
            "%s [%s]" % (t, "inherits" if not r["overlays"][c]
                         else "%d-line overlay" % len(r["overlays"][c]))
            for t, c in zip(r["targets"], r["classes"]))
        print("OK    %-22s moved=%-2d -> %s" % (r["board"], len(r["moved"]), tgts))
    for r in by_status.get("skip", []):
        print("SKIP  %-22s    %s" % (r["board"], r["reason"]))
    for r in by_status.get("manual", []):
        print("MANUAL %-21s    %s (families=%s)" % (r["board"], r["reason"], r["families"]))
    for r in by_status.get("error", []):
        print("ERROR %-22s -> %-20s %s" % (r["board"], ",".join(r.get("classes", [])), r["reason"]))

    print("\nSummary: %d ok, %d skip, %d manual, %d error  (%s)"
          % (len(by_status.get("ok", [])), len(by_status.get("skip", [])),
             len(by_status.get("manual", [])), len(by_status.get("error", [])),
             verb.lower() if args.apply else "dry run"))
    return 1 if by_status.get("error") else 0


if __name__ == "__main__":
    sys.exit(main())
