#!/usr/bin/env python3
"""
Build CubeRed secondary iofirmware from an ArduPilot checkout.
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List


def run_command(command: List[str], cwd: Path, env: Dict[str, str]) -> None:
    print(f"+ {' '.join(command)}")
    subprocess.run(command, cwd=cwd, env=env, check=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ardupilot-dir", required=True, type=Path, help="Path to ArduPilot source tree")
    parser.add_argument("--output", required=True, type=Path, help="Output binary path")
    parser.add_argument("--board", default="CubeRedSecondary-IO", help="ArduPilot board name")
    parser.add_argument("--target", default="iofirmware", help="waf build target")
    parser.add_argument("--artifact", default="iofirmware.bin", help="Artifact filename under build/<board>/bin/")
    parser.add_argument(
        "--workdir",
        type=Path,
        help="Optional isolated workspace directory (defaults to a temporary directory)",
    )
    parser.add_argument(
        "--metadata-out",
        type=Path,
        help="Optional path to write build metadata JSON",
    )
    parser.add_argument(
        "--keep-workdir",
        action="store_true",
        help="Keep the isolated workspace after completion (for debugging)",
    )
    return parser.parse_args()


def get_git_value(repo_dir: Path, command: List[str]) -> str:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo_dir), *command],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        return result.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ""


def copy_source_tree(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    rsync = shutil.which("rsync")

    if rsync:
        command = [
            rsync,
            "-a",
            "--safe-links",
            "--delete",
            "--exclude=build",
            "--exclude=.waf-*",
            "--exclude=.lock-waf*",
            "--exclude=.cache",
            "--exclude=.git",
            "--exclude=*.pyc",
            "--exclude=__pycache__",
            f"{src}/",
            f"{dst}/",
        ]
        subprocess.run(command, check=True)
        return

    shutil.copytree(
        src,
        dst,
        dirs_exist_ok=True,
        ignore=shutil.ignore_patterns(
            "build",
            ".waf-*",
            ".lock-waf*",
            ".cache",
            ".git",
            "*.pyc",
            "__pycache__",
        ),
    )


def sha256sum(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as file_handle:
        for chunk in iter(lambda: file_handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_metadata(
    metadata_path: Path,
    source_repo: Path,
    board: str,
    artifact_path: Path,
) -> None:
    metadata = {
        "source_repo": str(source_repo),
        "board": board,
        "git_describe": get_git_value(source_repo, ["describe", "--always", "--tags"]),
        "git_hash": get_git_value(source_repo, ["rev-parse", "--verify", "HEAD"]),
        "artifact_path": str(artifact_path),
        "artifact_size": artifact_path.stat().st_size,
        "artifact_sha256": sha256sum(artifact_path),
        "build_time_utc": datetime.now(timezone.utc).isoformat(),
    }
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    with open(metadata_path, "w", encoding="utf-8") as metadata_file:
        json.dump(metadata, metadata_file, indent=2, sort_keys=True)


def main() -> int:
    args = parse_args()

    ardupilot_dir = args.ardupilot_dir.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    metadata_path = args.metadata_out.expanduser().resolve() if args.metadata_out else None
    source_waf_path = ardupilot_dir / "waf"

    if not source_waf_path.exists():
        print(f"error: waf not found in {ardupilot_dir}", file=sys.stderr)
        return 1

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    temp_workspace: Path
    managed_tempdir = None

    if args.workdir:
        temp_workspace = args.workdir.expanduser().resolve()
        temp_workspace.mkdir(parents=True, exist_ok=True)
    else:
        managed_tempdir = tempfile.TemporaryDirectory(prefix="cubered_secondary_build_")
        temp_workspace = Path(managed_tempdir.name)

    build_root = temp_workspace / "ardupilot_src"
    artifact_path = build_root / "build" / args.board / "bin" / args.artifact

    try:
        print(f"preparing isolated workspace at {build_root}")
        copy_source_tree(ardupilot_dir, build_root)
        shutil.rmtree(build_root / "build", ignore_errors=True)
        for lock_file in build_root.glob(".lock-waf*"):
            lock_file.unlink(missing_ok=True)

        build_env = env.copy()
        source_git_dir = ardupilot_dir / ".git"
        if source_git_dir.exists():
            build_env["GIT_DIR"] = str(source_git_dir)
            build_env["GIT_WORK_TREE"] = str(build_root)

        run_command(["./waf", "configure", "--board", args.board], cwd=build_root, env=build_env)
        run_command(["./waf", args.target], cwd=build_root, env=build_env)

        if not artifact_path.exists():
            print(f"error: expected artifact not found: {artifact_path}", file=sys.stderr)
            return 1

        output_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(artifact_path, output_path)
        print(f"copied {artifact_path} -> {output_path}")

        if metadata_path is not None:
            write_metadata(metadata_path, ardupilot_dir, args.board, artifact_path)
            print(f"wrote metadata {metadata_path}")

        return 0

    except subprocess.CalledProcessError as err:
        print(f"error: command failed with exit code {err.returncode}", file=sys.stderr)
        return err.returncode

    finally:
        if managed_tempdir and not args.keep_workdir:
            managed_tempdir.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
