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
PX4 Firmware Index Updater

Updates the top-level firmware index manifest with a new release entry.
The index provides a lightweight list of all available releases, allowing
tools like QGroundControl to discover firmware versions without downloading
individual release manifests.

The index is stored at s3://px4-travis/Firmware/index.json and contains
metadata about each release including URLs to the per-release manifests.
"""

import argparse
import json
import sys
import time
import re
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.request import urlopen
from urllib.error import URLError


def fetch_existing_index(index_url: str) -> Optional[Dict[str, Any]]:
    """
    Fetch the existing firmware index from a URL.
    Returns None if the index doesn't exist or can't be fetched.
    """
    try:
        with urlopen(index_url, timeout=30) as response:
            return json.loads(response.read().decode("utf-8"))
    except URLError as e:
        print(f"Note: Could not fetch existing index from {index_url}: {e}", file=sys.stderr)
        return None
    except json.JSONDecodeError as e:
        print(f"Warning: Invalid JSON in existing index: {e}", file=sys.stderr)
        return None


def load_existing_index(index_path: Path) -> Optional[Dict[str, Any]]:
    """
    Load the existing firmware index from a local file.
    Returns None if the file doesn't exist.
    """
    if not index_path.exists():
        return None
    try:
        with open(index_path, "r", encoding="utf-8") as f:
            return json.load(f)
    except json.JSONDecodeError as e:
        print(f"Warning: Invalid JSON in existing index file: {e}", file=sys.stderr)
        return None


def create_empty_index() -> Dict[str, Any]:
    """Create a new empty firmware index."""
    return {
        "format_version": 1,
        "updated_at": int(time.time()),
        "description": "PX4 Firmware Release Index",
        "releases": [],
    }


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


def update_index(
    index: Dict[str, Any],
    version: str,
    git_tag: str,
    build_count: int,
    manifest_url: str,
    s3_url: str = "",
) -> Dict[str, Any]:
    """
    Update the firmware index with a new release entry.
    If the version already exists, it will be updated.
    """
    channel = determine_channel(version)
    release_date = time.strftime("%Y-%m-%d")

    new_entry = {
        "version": version,
        "git_tag": git_tag,
        "release_date": release_date,
        "channel": channel,
        "build_count": build_count,
        "manifest_url": manifest_url,
    }

    # Add S3 URL if provided (as alternative download location)
    if s3_url:
        new_entry["s3_manifest_url"] = s3_url

    # Remove existing entry for this version if present
    index["releases"] = [r for r in index["releases"] if r.get("version") != version]

    # Add new entry
    index["releases"].append(new_entry)

    # Sort releases by version (newest first)
    index["releases"].sort(key=lambda r: parse_version_tuple(r.get("version", "")), reverse=True)

    # Update metadata
    index["updated_at"] = int(time.time())

    # Set latest stable version
    stable_releases = [r for r in index["releases"] if r.get("channel") == "stable"]
    if stable_releases:
        index["latest_stable"] = stable_releases[0]["version"]

    # Set latest for each channel
    for ch in ["stable", "beta", "dev"]:
        channel_releases = [r for r in index["releases"] if r.get("channel") == ch]
        if channel_releases:
            index[f"latest_{ch}"] = channel_releases[0]["version"]

    return index


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Update the PX4 firmware index with a new release.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Update index with a new release (fetch existing from URL)
  %(prog)s --version v1.15.0 --git-tag v1.15.0 --build-count 150 \\
           --manifest-url https://github.com/PX4/PX4-Autopilot/releases/download/v1.15.0/release_manifest.json \\
           --fetch-url https://px4-travis.s3.us-west-1.amazonaws.com/Firmware/index.json \\
           --out index.json

  # Update existing local index file
  %(prog)s --version v1.15.0 --git-tag v1.15.0 --build-count 150 \\
           --manifest-url https://github.com/.../release_manifest.json \\
           --existing-index ./index.json \\
           --out ./index.json
        """,
    )
    parser.add_argument(
        "--version",
        required=True,
        help="Release version (e.g., v1.15.0)",
    )
    parser.add_argument(
        "--git-tag",
        required=True,
        help="Git tag for this release",
    )
    parser.add_argument(
        "--build-count",
        type=int,
        required=True,
        help="Number of firmware builds in this release",
    )
    parser.add_argument(
        "--manifest-url",
        required=True,
        help="URL to the release manifest JSON (GitHub releases URL)",
    )
    parser.add_argument(
        "--s3-manifest-url",
        default="",
        help="Alternative S3 URL to the release manifest",
    )
    parser.add_argument(
        "--fetch-url",
        default="",
        help="URL to fetch existing index from (e.g., S3 public URL)",
    )
    parser.add_argument(
        "--existing-index",
        type=Path,
        help="Path to existing local index file to update",
    )
    parser.add_argument(
        "--out",
        type=Path,
        required=True,
        help="Output file path for updated index",
    )

    args = parser.parse_args()

    # Load or create index
    index = None

    if args.existing_index:
        index = load_existing_index(args.existing_index)
        if index:
            print(f"Loaded existing index from {args.existing_index}", file=sys.stderr)

    if index is None and args.fetch_url:
        index = fetch_existing_index(args.fetch_url)
        if index:
            print(f"Fetched existing index from {args.fetch_url}", file=sys.stderr)

    if index is None:
        print("Creating new firmware index", file=sys.stderr)
        index = create_empty_index()

    # Update index with new release
    index = update_index(
        index=index,
        version=args.version,
        git_tag=args.git_tag,
        build_count=args.build_count,
        manifest_url=args.manifest_url,
        s3_url=args.s3_manifest_url,
    )

    # Write output
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(index, f, indent=2)
        f.write("\n")

    print(f"Updated firmware index: {args.out} ({len(index['releases'])} releases)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
