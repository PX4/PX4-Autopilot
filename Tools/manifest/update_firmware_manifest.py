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
PX4 Unified Firmware Manifest

Scans a directory for *.px4 firmware files, extracts their embedded metadata,
and upserts the release into a single unified manifest JSON file containing
all releases with all firmware variants inline.

The manifest is stored at s3://px4-travis/Firmware/manifest.json and provides
a complete index of all available firmware for tools like QGroundControl.
"""

import argparse
import json
import re
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.request import urlopen
from urllib.error import URLError


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


def determine_channel(version: str) -> str:
    """
    Determine the release channel from version string.
    - stable: vX.Y.Z (no suffix)
    - beta: vX.Y.Z-beta*, vX.Y.Z-rc*
    - dev: vX.Y.Z-alpha*, vX.Y.Z-dev*, or any other suffix
    """
    version_lower = version.lower()
    if re.match(r"^v?\d+\.\d+\.\d+$", version):
        return "stable"
    elif "-beta" in version_lower or "-rc" in version_lower:
        return "beta"
    else:
        return "dev"


def parse_version_tuple(version: str) -> tuple:
    """
    Parse version string into a tuple for sorting.
    Returns (major, minor, patch, prerelease_type, prerelease_num)
    """
    # Remove 'v' prefix if present
    v = version.lstrip("v")

    # Match version pattern
    match = re.match(r"(\d+)\.(\d+)\.(\d+)(?:-([a-zA-Z]+)(\d+)?)?", v)
    if not match:
        return (0, 0, 0, "zzz", 0)  # Unknown versions sort last

    major, minor, patch = int(match.group(1)), int(match.group(2)), int(match.group(3))
    prerelease_type = match.group(4) or ""
    prerelease_num = int(match.group(5)) if match.group(5) else 0

    # Stable releases (no prerelease) should sort after prereleases
    # Use empty string to sort after alpha/beta/rc
    if not prerelease_type:
        prerelease_type = "zzz"  # Sorts after alpha, beta, rc

    return (major, minor, patch, prerelease_type.lower(), prerelease_num)


def fetch_existing_manifest(url: str) -> Optional[Dict[str, Any]]:
    """
    Fetch the existing firmware manifest from a URL.
    Returns None if the manifest doesn't exist or can't be fetched.
    """
    try:
        with urlopen(url, timeout=30) as response:
            return json.loads(response.read().decode("utf-8"))
    except URLError as e:
        print(f"Note: Could not fetch existing manifest from {url}: {e}", file=sys.stderr)
        return None
    except json.JSONDecodeError as e:
        print(f"Warning: Invalid JSON in existing manifest: {e}", file=sys.stderr)
        return None


def load_existing_manifest(path: Path) -> Optional[Dict[str, Any]]:
    """
    Load the existing firmware manifest from a local file.
    Returns None if the file doesn't exist.
    """
    if not path.exists():
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except json.JSONDecodeError as e:
        print(f"Warning: Invalid JSON in existing manifest file: {e}", file=sys.stderr)
        return None


def create_empty_manifest() -> Dict[str, Any]:
    """Create a new empty unified firmware manifest."""
    return {
        "format_version": 2,
        "updated_at": int(time.time()),
        "description": "PX4 Firmware Manifest",
        "releases": {},
    }


def scan_builds(px4_dir: Path, base_url: str = "") -> List[Dict[str, Any]]:
    """
    Scan a directory for .px4 files and extract metadata from each.
    Returns a sorted list of build metadata dicts.
    """
    builds: List[Dict[str, Any]] = []

    px4_files = sorted(px4_dir.glob("*.px4"))
    if not px4_files:
        print(f"Warning: No .px4 files found in {px4_dir}", file=sys.stderr)

    for px4_path in px4_files:
        metadata = extract_px4_metadata(px4_path)
        if metadata:
            if base_url:
                metadata["url"] = base_url.rstrip("/") + "/" + metadata["filename"]
            builds.append(metadata)

    # Sort builds by target name for consistent ordering
    builds.sort(key=lambda b: b.get("manifest", {}).get("target", b["filename"]))
    return builds


def update_latest_pointers(manifest: Dict[str, Any]) -> None:
    """
    Recompute latest_stable, latest_beta, and latest_dev from the releases dict.
    Modifies manifest in place.
    """
    channels: Dict[str, List[str]] = {"stable": [], "beta": [], "dev": []}

    for version, release in manifest["releases"].items():
        ch = release.get("channel", "dev")
        if ch in channels:
            channels[ch].append(version)

    for ch, versions in channels.items():
        if versions:
            versions.sort(key=parse_version_tuple, reverse=True)
            manifest[f"latest_{ch}"] = versions[0]
        elif f"latest_{ch}" in manifest:
            del manifest[f"latest_{ch}"]


def update_manifest(
    manifest: Dict[str, Any],
    version: str,
    git_tag: str,
    builds: List[Dict[str, Any]],
) -> Dict[str, Any]:
    """
    Upsert a release entry into the unified manifest with all builds inline.
    """
    channel = determine_channel(version)
    release_date = time.strftime("%Y-%m-%d")

    manifest["releases"][version] = {
        "git_tag": git_tag,
        "release_date": release_date,
        "channel": channel,
        "build_count": len(builds),
        "generated_at": int(time.time()),
        "builds": builds,
    }

    manifest["updated_at"] = int(time.time())
    update_latest_pointers(manifest)

    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Update the unified PX4 firmware manifest with a new release.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate manifest from local .px4 files (fresh start)
  %(prog)s --dir ./artifacts --version v1.15.0 --git-tag v1.15.0 --out manifest.json

  # Update existing manifest from S3 with a new release
  %(prog)s --dir ./artifacts --version v1.15.0 --git-tag v1.15.0 \\
           --base-url https://github.com/PX4/PX4-Autopilot/releases/download/v1.15.0 \\
           --fetch-url https://px4-travis.s3.us-west-1.amazonaws.com/Firmware/manifest.json \\
           --out manifest.json

  # Update an existing local manifest file
  %(prog)s --dir ./artifacts --version master --existing manifest.json --out manifest.json
        """,
    )
    parser.add_argument(
        "--dir",
        type=Path,
        required=True,
        help="Directory containing .px4 files to scan",
    )
    parser.add_argument(
        "--version",
        required=True,
        help="Release version / key (e.g., v1.15.0, master)",
    )
    parser.add_argument(
        "--git-tag",
        default="",
        help="Git tag for this release (empty for branch builds)",
    )
    parser.add_argument(
        "--base-url",
        default="",
        help="Base URL prefix for firmware download URLs",
    )
    parser.add_argument(
        "--fetch-url",
        default="",
        help="URL to fetch existing manifest from (e.g., S3 public URL)",
    )
    parser.add_argument(
        "--existing",
        type=Path,
        help="Path to existing local manifest file to update",
    )
    parser.add_argument(
        "--out",
        type=Path,
        required=True,
        help="Output file path for updated manifest",
    )

    args = parser.parse_args()

    if not args.dir.is_dir():
        print(f"Error: {args.dir} is not a directory", file=sys.stderr)
        return 1

    # Load or create manifest
    manifest = None

    if args.existing:
        manifest = load_existing_manifest(args.existing)
        if manifest:
            print(f"Loaded existing manifest from {args.existing}", file=sys.stderr)

    if manifest is None and args.fetch_url:
        manifest = fetch_existing_manifest(args.fetch_url)
        if manifest:
            print(f"Fetched existing manifest from {args.fetch_url}", file=sys.stderr)

    if manifest is None:
        print("Creating new firmware manifest", file=sys.stderr)
        manifest = create_empty_manifest()

    # Scan .px4 files
    builds = scan_builds(args.dir, base_url=args.base_url)

    # Upsert release into manifest
    manifest = update_manifest(
        manifest=manifest,
        version=args.version,
        git_tag=args.git_tag,
        builds=builds,
    )

    # Write output
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
        f.write("\n")

    release = manifest["releases"][args.version]
    print(
        f"Updated firmware manifest: {args.out} "
        f"({len(manifest['releases'])} releases, "
        f"{release['build_count']} builds for {args.version})",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
