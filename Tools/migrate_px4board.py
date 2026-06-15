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

Class is inferred from the enabled controllers (and ROMFSROOT for cannode):

    romfsroot == "cannode"      -> cannode
    MC only                     -> copter
    FW only                     -> plane
    MC + FW + VTOL              -> vtol   (the air superset, policy A)
    ROVER only                  -> rover
    UUV only                    -> uuv
    AIRSHIP only                -> airship
    SPACECRAFT only             -> spacecraft

Boards that carry extra vehicle classes on top of the air stack (e.g. vtol+uuv,
vtol+airship) or that have no controllers and no cannode romfs (IO firmware,
companion targets, SITL) are reported as MANUAL and never written -- their split
needs a human decision (multi-target policy / special class) and the script
refuses to guess.

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


# Inferred class -> the controller-family set that defines it. A board whose
# enabled families exactly match one of these maps cleanly; anything else is
# reported for manual handling.
_FAMILY_TO_CLASS = {
    frozenset({"mc"}): "copter",
    frozenset({"fw"}): "plane",
    frozenset({"mc", "fw", "vtol"}): "vtol",
    frozenset({"rover"}): "rover",
    frozenset({"uuv"}): "uuv",
    frozenset({"airship"}): "airship",
    frozenset({"spacecraft"}): "spacecraft",
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


def infer_class(assignments):
    """Return (class, families, reason). class is None when manual handling is
    required; reason explains why."""
    if assignments.get("BOARD_ROMFSROOT") == '"cannode"':
        return "cannode", set(), None

    families = set()
    for sym, val in assignments.items():
        if val == "y" and is_controller(sym):
            families.add(controller_family(sym))

    if not families:
        return None, families, "no controllers and not a cannode romfs"

    cls = _FAMILY_TO_CLASS.get(frozenset(families))
    if cls is None:
        return None, families, "multi-class controller set %s" % sorted(families)
    return cls, families, None


def migrate_board(default_path, apply, force):
    """Plan (and optionally write) the split for one board. Returns a result
    dict for the report."""
    board_dir = os.path.dirname(default_path)
    rel = os.path.relpath(board_dir, os.path.join(REPO_ROOT, "boards"))
    result = {"board": rel}

    default = Fragment.from_file(default_path)
    cls, families, reason = infer_class(default.assignments)
    result["class"] = cls
    result["families"] = sorted(families)
    if cls is None:
        result["status"] = "manual"
        result["reason"] = reason
        return result

    common_path = os.path.join(REPO_ROOT, "target_classes", cls + ".px4board")
    if not os.path.exists(common_path):
        result["status"] = "manual"
        result["reason"] = "no target_classes/%s.px4board" % cls
        return result
    common = Fragment.from_file(common_path)

    base_path = os.path.join(board_dir, "base.px4board")
    overlay_path = os.path.join(board_dir, cls + ".px4board")
    if os.path.exists(base_path) and not force:
        result["status"] = "skip"
        result["reason"] = "base.px4board already exists (use --force)"
        return result

    # Symbols the class base provides plus the board's own controllers: these are
    # stripped from base. Everything else (board hardware, support modules,
    # non-controller disables) is preserved verbatim, in order.
    controllers = {s for s, v in default.assignments.items()
                   if v == "y" and is_controller(s)}
    strip = set(common.assignments) | controllers
    base_lines = [ln for ln in default.lines if Fragment.sym_of(ln) not in strip]

    # Overlay: reconcile every symbol the class base provides or the board
    # carried as a controller against what base+common already yields.
    overlay_lines = []
    for sym in sorted(strip):
        desired = default.assignments.get(sym)          # rhs, UNSET, or None
        provided = common.assignments.get(sym)          # rhs or None
        prov_off = provided in (None, "n", Fragment.UNSET)
        if desired is None or desired in ("n", Fragment.UNSET):
            # Board does not enable this symbol; revert if the class base turned
            # it on.
            if not prov_off:
                overlay_lines.append("# CONFIG_%s is not set" % sym)
        else:
            # Board enables it with `desired`; add only if the class base does
            # not already provide that exact value.
            if provided != desired:
                overlay_lines.append("CONFIG_%s=%s" % (sym, desired))

    overlay = Fragment(overlay_lines)

    # Pre-write gate: base -> common -> overlay must reproduce default. The one
    # tolerated difference is a controller the board omitted that the class base
    # provides and the overlay pins off -- every controller module is `default
    # n`, so an omitted controller is off in the legacy build too. Any other
    # divergence is a real error and blocks the write.
    want = effective([default])
    got = effective([Fragment(base_lines), common, overlay])
    pinned_off = []
    unsafe = {}
    for s in set(want) | set(got):
        if want.get(s) == got.get(s):
            continue
        if is_controller(s) and want.get(s) is None and got.get(s) == OFF:
            pinned_off.append(s)
        else:
            unsafe[s] = (want.get(s), got.get(s))
    if unsafe:
        result["status"] = "error"
        result["reason"] = "merge mismatch: %s" % unsafe
        return result

    result["status"] = "ok"
    result["pinned_off"] = sorted(pinned_off)
    result["moved"] = sorted(strip & set(default.assignments))
    result["overlay_lines"] = overlay_lines
    result["base_path"] = os.path.relpath(base_path, REPO_ROOT)
    result["overlay_path"] = os.path.relpath(overlay_path, REPO_ROOT)
    result["target"] = "%s_%s" % (rel.replace("/", "_"), cls)

    if apply:
        with open(base_path, "w") as f:
            f.write("\n".join(base_lines) + ("\n" if base_lines else ""))
        with open(overlay_path, "w") as f:
            if overlay_lines:
                f.write("\n".join(overlay_lines) + "\n")
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
        ov = "inherits target_classes/%s" % r["class"] if not r["overlay_lines"] \
            else "%d line(s): %s" % (len(r["overlay_lines"]), ", ".join(r["overlay_lines"]))
        print("OK    %-22s -> %-9s target=%-26s moved=%d  overlay=%s"
              % (r["board"], r["class"], r["target"], len(r["moved"]), ov))
    for r in by_status.get("skip", []):
        print("SKIP  %-22s    %s" % (r["board"], r["reason"]))
    for r in by_status.get("manual", []):
        print("MANUAL %-21s    %s (families=%s)" % (r["board"], r["reason"], r["families"]))
    for r in by_status.get("error", []):
        print("ERROR %-22s -> %-9s %s" % (r["board"], r["class"], r["reason"]))

    print("\nSummary: %d ok, %d skip, %d manual, %d error  (%s)"
          % (len(by_status.get("ok", [])), len(by_status.get("skip", [])),
             len(by_status.get("manual", [])), len(by_status.get("error", [])),
             verb.lower() if args.apply else "dry run"))
    return 1 if by_status.get("error") else 0


if __name__ == "__main__":
    sys.exit(main())
