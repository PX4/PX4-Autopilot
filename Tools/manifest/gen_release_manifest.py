#!/usr/bin/env python3
############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

"""
PX4 Release Manifest Generator

Scans a directory for *.px4 firmware files, extracts their embedded manifests,
and generates a consolidated release manifest JSON file.

The release manifest provides a single index of all available firmware builds
for a release, enabling tools like QGroundControl to discover available
firmware without downloading individual files.
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional


def extract_px4_metadata(px4_path: Path) -> Optional[Dict[str, Any]]:
    """
    Extract metadata from a .px4 firmware file.

    The .px4 file is a JSON-encoded object containing firmware metadata
    and a compressed binary image.
    """
    try:
        with open(px4_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        # Verify this is a valid firmware file by checking required fields
        # Different boards use different magic strings (PX4FWv1, PX4FWv2, ARKFWv1, etc.)
        if "magic" not in data or "board_id" not in data or "image" not in data:
            print(f"Warning: {px4_path.name} missing required fields, skipping", file=sys.stderr)
            return None

        # Extract relevant metadata (exclude the large binary data)
        metadata = {
            "filename": px4_path.name,
            "board_id": data.get("board_id", 0),
            "board_revision": data.get("board_revision", 0),
            "version": data.get("version", ""),
            "git_identity": data.get("git_identity", ""),
            "git_hash": data.get("git_hash", ""),
            "build_time": data.get("build_time", 0),
            "image_size": data.get("image_size", 0),
            "sha256sum": data.get("sha256sum", ""),
            "mav_autopilot": data.get("mav_autopilot", 12),
        }

        # Include the board manifest if present
        if "manifest" in data and data["manifest"]:
            metadata["manifest"] = data["manifest"]

        return metadata

    except json.JSONDecodeError as e:
        print(f"Warning: Failed to parse {px4_path.name}: {e}", file=sys.stderr)
        return None
    except Exception as e:
        print(f"Warning: Error reading {px4_path.name}: {e}", file=sys.stderr)
        return None


def generate_release_manifest(
    px4_dir: Path,
    version: str = "",
    git_tag: str = "",
    base_url: str = "",
) -> Dict[str, Any]:
    """
    Generate a release manifest from all .px4 files in a directory.
    """
    builds: List[Dict[str, Any]] = []

    # Find all .px4 files
    px4_files = sorted(px4_dir.glob("*.px4"))

    if not px4_files:
        print(f"Warning: No .px4 files found in {px4_dir}", file=sys.stderr)

    for px4_path in px4_files:
        metadata = extract_px4_metadata(px4_path)
        if metadata:
            # Add download URL if base_url is provided
            if base_url:
                url = base_url.rstrip("/") + "/" + metadata["filename"]
                metadata["url"] = url
            builds.append(metadata)

    # Sort builds by target name for consistent ordering
    builds.sort(key=lambda b: b.get("manifest", {}).get("target", b["filename"]))

    # Create the release manifest
    manifest = {
        "format_version": 1,
        "generated_at": int(time.time()),
        "version": version,
        "git_tag": git_tag,
        "build_count": len(builds),
        "builds": builds,
    }

    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate a release manifest from PX4 firmware files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate manifest from current directory
  %(prog)s --dir ./artifacts --out release_manifest.json

  # Generate manifest with version info
  %(prog)s --dir ./artifacts --version v1.15.0 --git-tag v1.15.0 --out manifest.json

  # Generate manifest with download URLs
  %(prog)s --dir ./artifacts --base-url https://github.com/PX4/PX4-Autopilot/releases/download/v1.15.0 --out manifest.json
        """,
    )
    parser.add_argument(
        "--dir",
        type=Path,
        default=Path("."),
        help="Directory containing .px4 files (default: current directory)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        help="Output file path (default: stdout)",
    )
    parser.add_argument(
        "--version",
        default="",
        help="Release version string (e.g., v1.15.0)",
    )
    parser.add_argument(
        "--git-tag",
        default="",
        help="Git tag for this release",
    )
    parser.add_argument(
        "--base-url",
        default="",
        help="Base URL for firmware downloads (URLs will be base_url/filename.px4)",
    )

    args = parser.parse_args()

    if not args.dir.is_dir():
        print(f"Error: {args.dir} is not a directory", file=sys.stderr)
        return 1

    manifest = generate_release_manifest(
        px4_dir=args.dir,
        version=args.version,
        git_tag=args.git_tag,
        base_url=args.base_url,
    )

    output = json.dumps(manifest, indent=2)

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(output)
            f.write("\n")
        print(f"Generated release manifest: {args.out} ({manifest['build_count']} builds)", file=sys.stderr)
    else:
        print(output)

    return 0


if __name__ == "__main__":
    sys.exit(main())
