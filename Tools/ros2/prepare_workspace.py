#!/usr/bin/env python3

import argparse
from pathlib import Path
import re
import shutil
import subprocess
import sys

import yaml


def prepare_workspace(
    workspace: Path, library_sha: str, repo_dir: Path
) -> None:
    if library_sha and not re.fullmatch(r"[0-9a-fA-F]{40}", library_sha):
        raise ValueError("LIBRARY_SHA must be a full 40-hex commit SHA")
    if workspace.is_symlink() or (
        workspace.exists()
        and (not workspace.is_dir() or any(workspace.iterdir()))
    ):
        raise ValueError(
            f"Workspace must be a new or empty directory: {workspace}"
        )

    with (repo_dir / "Tools/ros2/ros2.repos").open() as stream:
        manifest = yaml.safe_load(stream)
    repositories = manifest["repositories"]
    if library_sha:
        repositories["px4-ros2-interface-lib"]["version"] = library_sha.lower()
    for name, repository in repositories.items():
        version = repository["version"]
        if repository["type"] != "git" or not (
            isinstance(version, str) and re.fullmatch(r"[0-9a-f]{40}", version)
        ):
            raise ValueError(f"{name}: expected an immutable git commit SHA")
    if shutil.which("vcs") is None:
        raise FileNotFoundError("vcs is required; install vcstool")

    sources = workspace / "src"
    sources.mkdir(parents=True)
    resolved_manifest = workspace / "ros2.repos"
    resolved_manifest.write_text(yaml.safe_dump(manifest))
    with resolved_manifest.open() as stream:
        subprocess.run(
            ["vcs", "import", str(sources)], stdin=stream, check=True
        )

    for name, repository in repositories.items():
        actual = subprocess.check_output(
            ["git", "-C", str(sources / name), "rev-parse", "HEAD"], text=True
        ).strip()
        if actual != repository["version"]:
            raise ValueError(
                f"{name}: expected {repository['version']}, got {actual}"
            )
        print(f"{name}: {actual}", flush=True)

    subprocess.run(
        [
            "bash",
            str(repo_dir / "Tools/packaging/containers/prepare_context.sh"),
            "--messages-only",
            str(sources / "px4_msgs"),
        ],
        check=True,
    )
    print("Resolved ROS source manifest:", flush=True)
    print(resolved_manifest.read_text(), end="", flush=True)
    revision = subprocess.run(
        ["git", "-C", str(repo_dir), "rev-parse", "--verify", "HEAD"],
        capture_output=True, text=True,
    )
    if revision.returncode == 0:
        print(f"PX4 source: {revision.stdout.strip()}", flush=True)
    else:
        print(
            f"PX4 source revision unavailable: {revision.stderr.strip()}",
            file=sys.stderr,
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Prepare pinned ROS sources with this checkout's messages."
    )
    parser.add_argument("workspace", metavar="WS_DIR")
    parser.add_argument(
        "library_sha", metavar="LIBRARY_SHA", nargs="?", default=""
    )
    args = parser.parse_args()
    if not args.workspace:
        parser.error("WS_DIR must not be empty")
    try:
        prepare_workspace(
            Path(args.workspace), args.library_sha,
            Path(__file__).resolve().parents[2],
        )
    except (
        ValueError, OSError, subprocess.CalledProcessError, yaml.YAMLError
    ) as error:
        parser.exit(1, f"{parser.prog}: {error}\n")


if __name__ == "__main__":
    main()
