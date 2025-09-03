#!/usr/bin/env python3
import argparse, json, os, re, sys
from typing import Dict

def parse_defconfig(path: str) -> Dict[str, str]:
    d: Dict[str, str] = {}
    if not path or not os.path.exists(path):
        return d
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.strip()
            if not line or not line.startswith("CONFIG_") or "=" not in line or line.startswith("#"):
                continue
            k, v = line.split("=", 1)
            v = v.strip()
            if len(v) >= 2 and v[0] == '"' and v[-1] == '"':
                v = v[1:-1]
            d[k.strip()] = v
    return d

def norm_hex(s: str) -> str:
    if not s:
        return ""
    s = s.strip()
    if s.lower().startswith("0x"):
        return s.lower()
    try:
        return f"0x{int(s, 0):04x}"
    except Exception:
        return s

def detect_chip(defcfg: Dict[str,str]) -> str:
    for k, v in defcfg.items():
        if k.startswith("CONFIG_ARCH_CHIP_") and v == "y":
            return k[len("CONFIG_ARCH_CHIP_"):].lower().replace("_", "")
    s = defcfg.get("CONFIG_ARCH_CHIP", "")
    return s.lower().replace("_", "") if s else ""

def pick(preferred: str, fallback_key: str, defcfg: Dict[str, str]) -> str:
    return preferred if preferred else defcfg.get(fallback_key, "")

def main():
    ap = argparse.ArgumentParser(description="Generate board manifest (prefer CMake-passed overrides, fallback to defconfig).")
    ap.add_argument("--defconfig", required=False, help="Path to defconfig (fallback only)")
    # explicit overrides coming from CMake
    ap.add_argument("--manufacturer", default="")
    ap.add_argument("--productstr", default="")
    ap.add_argument("--target", default="")
    ap.add_argument("--name", default="")
    ap.add_argument("--arch", default="")
    ap.add_argument("--chip", default="")
    ap.add_argument("--vid", default="")
    ap.add_argument("--pid", default="")
    ap.add_argument("--out", help="Write to file instead of stdout")
    args = ap.parse_args()

    defcfg = parse_defconfig(args.defconfig) if args.defconfig else {}

    manufacturer    = pick(args.manufacturer, "CONFIG_BOARD_MANUFACTURER", defcfg)
    productstr      = pick(args.productstr,   "CONFIG_BOARD_PRODUCTSTR",   defcfg)
    target          = args.target or ""
    name            = args.name or ""
    arch            = (pick(args.arch,       "CONFIG_ARCH",                defcfg)).lower()
    chip            = args.chip or detect_chip(defcfg)
    vid             = norm_hex(pick(args.vid, "CONFIG_CDCACM_VENDORID",    defcfg))
    pid             = norm_hex(pick(args.pid, "CONFIG_CDCACM_PRODUCTID",   defcfg))

    manifest = {
        "name": name,
        "target": target,
        "manufacturer": manufacturer,
        "hardware": {
            "architecture": arch,
            "vendor_id": vid,
            "product_id": pid,
            "chip": chip,
            "productstr": productstr
        }
    }

    if args.out:
        out_dir = os.path.dirname(args.out)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=2)
            f.write("\n")
    else:
        json.dump(manifest, sys.stdout, indent=2)
        sys.stdout.write("\n")
    return 0

if __name__ == "__main__":
    sys.exit(main())
