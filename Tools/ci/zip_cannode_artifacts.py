#!/usr/bin/env python3
"""Zip artifacts/cannode so GitHub Releases can keep per-target folders.

GitHub release assets are a flat namespace; S3 already has the unzipped tree.
"""
from pathlib import Path
import sys
import zipfile

artifacts = Path(sys.argv[1] if len(sys.argv) > 1 else "artifacts")
cannode = artifacts / "cannode"
# .uavcan.bin (app) and *_canbootloader.bin (SWD); pathlib suffix of both is .bin
files = sorted(cannode.glob("*/*.bin")) if cannode.is_dir() else []

if not files:
    print("no cannode images to zip")
    sys.exit(0)

zip_path = artifacts / "cannode.zip"
with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
    for path in files:
        zf.write(path, path.relative_to(artifacts).as_posix())

print(f"wrote {zip_path} ({len(files)} images)")
