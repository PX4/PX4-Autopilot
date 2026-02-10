#!/usr/bin/env python3
import argparse, json, os, re, sys
from typing import Dict

_VEHICLE_LABELS = frozenset({
    "multicopter", "fixedwing", "vtol", "rover", "uuv", "spacecraft",
})
_BOOTLOADER_LABELS = frozenset({
    "bootloader", "canbootloader",
})

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

def detect_firmware_category(target: str, defcfg: Dict[str, str]) -> str:
    """Infer firmware_category from the build label and defconfig.

    Detection order:
      1. bootloader / canbootloader labels → "bootloader"
      2. Known vehicle labels → "vehicle"
      3. ROMFSROOT == "cannode" (CAN peripheral boards) → "peripheral"
      4. Everything else → "dev"

    If you are adding a NEW vehicle type (e.g. "balloon"), you must EITHER:
      1. Add the label to _VEHICLE_LABELS in this file, OR
      2. Set CONFIG_BOARD_FIRMWARE_CATEGORY="vehicle" in the .px4board file
    Otherwise the build will be classified as "dev" and hidden from
    end-users in ground stations like QGroundControl.
    """
    label = target.rsplit("_", 1)[-1].lower() if target else ""
    if label in _BOOTLOADER_LABELS:
        return "bootloader"
    if label in _VEHICLE_LABELS:
        return "vehicle"
    # CAN peripheral boards (sensors, GPS, flow, etc.) use cannode ROMFS
    romfsroot = defcfg.get("CONFIG_BOARD_ROMFSROOT", "")
    if romfsroot == "cannode":
        return "peripheral"
    if label not in ("default", ""):
        print(f"WARNING: label '{label}' (target '{target}') is not a known "
              f"vehicle type — defaulting firmware_category to 'dev'. "
              f"If this is a vehicle type, add it to _VEHICLE_LABELS in "
              f"{__file__} or set CONFIG_BOARD_FIRMWARE_CATEGORY in the "
              f".px4board file.", file=sys.stderr)
    return "dev"

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
    ap.add_argument("--label-pretty", default="")
    ap.add_argument("--firmware-category", default="")
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
    label_pretty    = pick(args.label_pretty,    "CONFIG_BOARD_LABEL_PRETTY",    defcfg)
    firmware_cat    = pick(args.firmware_category,"CONFIG_BOARD_FIRMWARE_CATEGORY",defcfg)
    if not firmware_cat:
        firmware_cat = detect_firmware_category(target, defcfg)

    manifest = {
        "name": name,
        "target": target,
        "label_pretty": label_pretty,
        "firmware_category": firmware_cat,
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
