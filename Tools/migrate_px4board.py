#!/usr/bin/env python3
"""Migrate a board's default.px4board to the target-class layout.

Splits boards/<v>/<b>/default.px4board into:

  * boards/<v>/<b>/base.px4board   -- vehicle-agnostic foundation: everything in
                                      default that is NOT pulled in by a target
                                      class (controllers, the cannode romfs +
                                      uavcannode driver, the linux platform).
  * boards/<v>/<b>/<class>.px4board -- the board's overlay for its class. It
                                      states the airframe classes this target
                                      ships:
                                        CONFIG_AIRFRAMES_<Y>=y ...
                                      plus, for a board that does not want one of
                                      the class controllers, a `# CONFIG_MODULES_X
                                      is not set` revert line. The class itself
                                      (CONFIG_TARGET_CLASS_<X>=y plus its
                                      controllers) is the merged class base.

The resolved target then merges, in order (cmake/kconfig.cmake):

    base.px4board -> target_classes/<class>.px4board -> <class>.px4board

The class base fragment target_classes/<class>.px4board (merged by cmake on the
class name) provides the class controllers, the cannode romfs/uavcannode driver,
or the POSIX/Linux platform, and sets CONFIG_TARGET_CLASS_<X>=y as a tag. Module
Kconfigs carry no class coupling (the controllers are plain `default n`, turned on
by the fragment's explicit =y and overridable by a board's revert line).
By construction the merge reproduces the original default.px4board configuration,
verified symbol-by-symbol before anything is written. default.px4board is left in
place so the legacy `<board>_default` target keeps building during the transition.

The class(es) are inferred from the enabled controllers (and ROMFSROOT for
cannode, PLATFORM_POSIX + LINUX_TARGET for linux). The air controller families
collapse to one class (policy A); each extra vehicle class becomes its own target
sharing the same base:

    PLATFORM_ROS2                        -> ros2   (ROS2 platform build)
    PLATFORM_POSIX (no LINUX_TARGET)     -> sitl   (simulator, full controller set)
    romfsroot == "cannode"               -> cannode
    PLATFORM_POSIX + BOARD_LINUX_TARGET  -> linux  (all-vehicle SBC binary)
    PX4IOFIRMWARE, no controllers        -> io     (coprocessor firmware blob)
    MC only                              -> copter
    FW only                              -> plane
    MC + FW + VTOL                       -> vtol   (the air superset)
    MC + FW + VTOL + UUV                 -> vtol AND uuv      (two targets, shared base)
    MC + FW + VTOL + AIRSHIP             -> vtol AND airship
    ROVER / UUV / AIRSHIP only           -> rover / uuv / spacecraft

For a multi-class board, base = default minus everything the classes pull in, and
each class gets its own overlay; the per-class targets are strict subsets whose
UNION reproduces the original default (verified before writing).

muorb/QURT Linux boards with no apps-side controllers (voxl2) carry their
controllers on a DSP, so they fit no controller-providing class and are reported
as MANUAL (never written) -- handled separately at the Phase-4 cutover.

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

# A vehicle-class controller module: these move out of the board's base and are
# provided by the class base fragment target_classes/<class>.px4board (merged by
# cmake on the class name). CONTROL_ALLOCATOR is intentionally NOT a controller
# here -- it is the vehicle-agnostic actuator stage and stays in base.
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

# Class -> the controller families it owns (for partitioning a multi-class board's
# controllers across its per-class overlays). cannode/linux carry their non-vehicle
# essentials in CLASS_PROVIDES below.
_CLASS_FAMILIES = {
    "copter": {"mc"},
    "plane": {"fw"},
    "vtol": {"mc", "fw", "vtol"},
    "rover": {"rover"},
    "uuv": {"uuv"},
    "airship": {"airship"},
    "spacecraft": {"spacecraft"},
    "linux": {"mc", "fw", "vtol", "rover", "uuv", "airship", "spacecraft"},
    "sitl": {"mc", "fw", "vtol", "rover", "uuv", "airship", "spacecraft"},
    "cannode": set(),
    "io": set(),
    "ros2": set(),
}

_MC = ["MODULES_MC_ATT_CONTROL", "MODULES_MC_AUTOTUNE_ATTITUDE_CONTROL",
       "MODULES_MC_HOVER_THRUST_ESTIMATOR", "MODULES_MC_POS_CONTROL",
       "MODULES_MC_RATE_CONTROL"]
_FW = ["MODULES_FW_ATT_CONTROL", "MODULES_FW_AUTOTUNE_ATTITUDE_CONTROL",
       "MODULES_FW_LATERAL_LONGITUDINAL_CONTROL", "MODULES_FW_MODE_MANAGER",
       "MODULES_FW_RATE_CONTROL"]
_UUV = ["MODULES_UUV_ATT_CONTROL", "MODULES_UUV_POS_CONTROL"]
_ROVER = ["MODULES_ROVER_ACKERMANN", "MODULES_ROVER_DIFFERENTIAL",
          "MODULES_ROVER_MECANUM"]

# What the class base fragment target_classes/<class>.px4board provides (the
# controllers, plus the cannode romfs/driver and the linux platform). The migrate
# tool models this to compute the base/overlay split and to verify the layered
# merge reproduces `default`; the ground-truth check is the real per-target
# boardconfig diff.
CLASS_PROVIDES = {
    "copter": {s: "y" for s in _MC},
    "plane": {s: "y" for s in _FW},
    "vtol": {s: "y" for s in _MC + _FW + ["MODULES_VTOL_ATT_CONTROL"]},
    "rover": {s: "y" for s in _ROVER},
    "uuv": {s: "y" for s in _UUV},
    "spacecraft": {"MODULES_SPACECRAFT": "y"},
    "airship": {"MODULES_AIRSHIP_ATT_CONTROL": "y"},
    "linux": dict({s: "y" for s in _MC + _FW + ["MODULES_VTOL_ATT_CONTROL"] + _UUV},
                  PLATFORM_POSIX="y", BOARD_LINUX_TARGET="y"),
    "sitl": dict({s: "y" for s in _MC + _FW + ["MODULES_VTOL_ATT_CONTROL"]
                  + _UUV + _ROVER + ["MODULES_AIRSHIP_ATT_CONTROL"]},
                 PLATFORM_POSIX="y"),
    "cannode": {"BOARD_ROMFSROOT": '"cannode"', "DRIVERS_UAVCANNODE": "y"},
    "io": {"BOARD_ROMFSROOT": '""', "MODULES_PX4IOFIRMWARE": "y"},
    "ros2": {"PLATFORM_ROS2": "y"},
}

# A vehicle controller -> the AIRFRAMES_<class> its presence historically turned
# on (the old `default y if MODULES_*` cascade in ROMFS/Kconfig), now made
# explicit per board. BALLOON stays a global `default y`; SIMULATION follows
# PWM_OUT_SIM and is set on the relevant specials in Phase 4.
_AIRFRAME_TRIGGERS = [
    ("COPTER", ["MODULES_MC_RATE_CONTROL"]),
    ("PLANE", ["MODULES_FW_RATE_CONTROL"]),
    ("VTOL", ["MODULES_VTOL_ATT_CONTROL"]),
    ("ROVER", _ROVER),
    ("UUV", ["MODULES_UUV_ATT_CONTROL"]),
    ("SPACECRAFT", ["MODULES_SPACECRAFT"]),
    ("AIRSHIP", ["MODULES_AIRSHIP_ATT_CONTROL"]),
]


def airframes_for(cls, assignments):
    """The AIRFRAMES_<X> this class's target ships: the airframe classes whose
    trigger controller is both provided by `cls` and enabled by the board (so a
    vtol target ships copter+plane+vtol frames, a uuv target only uuv)."""
    provided = CLASS_PROVIDES.get(cls, {})
    out = []
    for name, triggers in _AIRFRAME_TRIGGERS:
        if any(t in provided and assignments.get(t) == "y" for t in triggers):
            out.append(name)
    return out


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
    def from_provides(cls, provides):
        """Synthetic fragment modelling what a TARGET_CLASS pulls in."""
        return cls(["CONFIG_%s=%s" % (s, v) for s, v in provides.items()])

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
    if "PLATFORM_ROS2" in assignments:
        return ["ros2"], [], None
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
        # Plain POSIX (no Linux cross-target) is the SITL simulator: the class
        # base provides the POSIX platform plus the full controller set.
        if any(v == "y" and is_controller(s) for s, v in assignments.items()):
            return ["sitl"], [], None
        return [], [], "posix board with no controllers"
    if assignments.get("BOARD_ROMFSROOT") == '"cannode"':
        return ["cannode"], [], None

    families = set()
    for sym, val in assignments.items():
        if val == "y" and is_controller(sym):
            families.add(controller_family(sym))
    if not families:
        if assignments.get("MODULES_PX4IOFIRMWARE") == "y":
            # PX4IO coprocessor firmware blob: no ROMFS, no vehicle controllers.
            return ["io"], [], None
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


def _class_overlay(default, cls):
    """The board's overlay for `cls`: the explicit AIRFRAMES symbols, then the
    board's deltas vs what the class base provides over the class-provided symbols
    plus the board's controllers in this class's families: a revert for a class
    controller the board lacks (or the cannode uavcannode driver), and an explicit
    `=y` for a board controller the class does not provide (e.g. a board carrying
    the legacy/undefined MODULES_FW_POS_CONTROL). CONFIG_TARGET_CLASS_<X>=y is set
    by the class base fragment (target_classes/<class>.px4board), not here."""
    provided = CLASS_PROVIDES[cls]
    lines = []
    for af in airframes_for(cls, default.assignments):
        lines.append("CONFIG_AIRFRAMES_%s=y" % af)
    cls_controllers = {s for s, v in default.assignments.items()
                       if v == "y" and is_controller(s)
                       and controller_family(s) in _CLASS_FAMILIES[cls]}
    for sym in sorted(set(provided) | cls_controllers):
        prov = provided.get(sym)
        desired = default.assignments.get(sym)
        if desired == prov:
            continue
        if desired is None or desired in ("n", Fragment.UNSET):
            if prov is not None and prov not in ("n", Fragment.UNSET):
                lines.append("# CONFIG_%s is not set" % sym)
        else:
            lines.append("CONFIG_%s=%s" % (sym, desired))
    if not lines:
        lines = ["# Board overrides for the %s target go here; inherits "
                 "target_classes/%s.px4board." % (cls, cls)]
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

    base_path = os.path.join(board_dir, "base.px4board")
    if os.path.exists(base_path) and not force:
        result["status"] = "skip"
        result["reason"] = "base.px4board already exists (use --force)"
        return result

    # base = default minus every controller and every symbol the board's target
    # classes pull in. The remainder (board hardware, support modules,
    # non-controller disables) is preserved verbatim and shared by all of the
    # board's per-class targets.
    controllers = {s for s, v in default.assignments.items()
                   if v == "y" and is_controller(s)}
    provided_syms = set().union(*(set(CLASS_PROVIDES[c]) for c in classes))
    strip = provided_syms | controllers
    base_lines = [ln for ln in default.lines if Fragment.sym_of(ln) not in strip]

    overlays = {cls: _class_overlay(default, cls) for cls in classes}

    # Pre-write gate: base plus every (class provides + its overlay) layered
    # together must reproduce default over the controller/platform/romfs symbols.
    # The TARGET_CLASS_<X> and AIRFRAMES_<Y> lines are intentionally new (not in
    # default) and excluded. For a multi-class board this is the UNION of its
    # per-class targets (each individual target is a strict subset by design).
    layers = [Fragment(base_lines)]
    for cls in classes:
        layers.append(Fragment.from_provides(CLASS_PROVIDES[cls]))
        layers.append(Fragment(overlays[cls]))

    class_provided_on = {s for cls in classes
                         for s, v in CLASS_PROVIDES[cls].items()
                         if v not in (Fragment.UNSET, "n")}

    want = effective([default])
    got = effective(layers)
    reverted = []
    unsafe = {}
    for s in set(want) | set(got):
        if s.startswith("TARGET_CLASS_") or s.startswith("AIRFRAMES_"):
            continue
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
                f.write("\n".join(overlays[cls]) + "\n")
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
        tgts = ", ".join("%s [%d-line]" % (t, len(r["overlays"][c]))
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
