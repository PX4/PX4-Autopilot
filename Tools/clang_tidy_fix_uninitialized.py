#!/usr/bin/env python3
"""
Zero initialize and default construct variables using clang-tidy.
Only modifies files within the specified directory.

Usage: ./Tools/clang_tidy_fix_uninitialized.py <path>
Example: ./Tools/clang_tidy_fix_uninitialized.py src/modules/fw_att_control
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path


def check_dependencies():
    """Check that required tools are installed. Returns True if all deps are met."""
    missing = []

    # Check for clang-tidy
    if not shutil.which("clang-tidy"):
        missing.append(("clang-tidy", "sudo apt install clang-tidy"))

    # Check for clang-apply-replacements (needed for -fix)
    if not shutil.which("clang-apply-replacements"):
        # Check for versioned binary
        versioned = None
        for v in range(20, 10, -1):
            if shutil.which(f"clang-apply-replacements-{v}"):
                versioned = f"clang-apply-replacements-{v}"
                break

        if versioned:
            missing.append((
                "clang-apply-replacements",
                f"sudo ln -s /usr/bin/{versioned} /usr/bin/clang-apply-replacements"
            ))
        else:
            missing.append((
                "clang-apply-replacements",
                "sudo apt install clang-tools"
            ))

    # Check for clang/clang++ (needed to build with compile_commands.json)
    if not shutil.which("clang") or not shutil.which("clang++"):
        missing.append(("clang", "sudo apt install clang"))

    if missing:
        print("Error: Missing required dependencies:\n")
        for tool, install_cmd in missing:
            print(f"  {tool}")
            print(f"    Install with: {install_cmd}\n")
        return False

    return True


def main():
    parser = argparse.ArgumentParser(
        description="Fix uninitialized member variables using clang-tidy"
    )
    parser.add_argument(
        "path",
        help="Directory to fix (e.g., src/modules/fw_att_control)"
    )
    parser.add_argument(
        "--build-dir",
        help="Build directory containing compile_commands.json",
        default=None
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show what would be fixed without making changes"
    )
    args = parser.parse_args()

    # Check dependencies first
    if not check_dependencies():
        return 1

    # Find repo root (where this script lives in Tools/)
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent

    # Determine build directory
    if args.build_dir:
        build_dir = Path(args.build_dir)
    else:
        build_dir = repo_root / "build" / "px4_sitl_default-clang"

    compile_commands = build_dir / "compile_commands.json"
    if not compile_commands.exists():
        print(f"Error: compile_commands.json not found in {build_dir}")
        print("Run 'make px4_sitl_default-clang' first")
        return 1

    # Resolve target path
    target_path = Path(args.path)
    if not target_path.is_absolute():
        target_path = repo_root / target_path
    target_path = target_path.resolve()

    if not target_path.is_dir():
        print(f"Error: Directory not found: {target_path}")
        return 1

    # Load compile_commands.json and find matching files
    with open(compile_commands) as f:
        db = json.load(f)

    target_str = str(target_path)
    files_to_check = [
        entry["file"] for entry in db
        if entry["file"].startswith(target_str)
    ]

    if not files_to_check:
        print(f"No files found in compile_commands.json matching: {target_path}")
        return 1

    print(f"Found {len(files_to_check)} files to check")

    # Run clang-tidy on each file
    checks = "-*,cppcoreguidelines-pro-type-member-init,hicpp-member-init"
    header_filter = f"{target_str}/.*"

    fixed_count = 0
    error_count = 0

    for filepath in files_to_check:
        if not os.path.isfile(filepath):
            continue

        rel_path = os.path.relpath(filepath, repo_root)
        print(f"  Checking: {rel_path}")

        cmd = [
            "clang-tidy",
            f"-checks={checks}",
            f"-header-filter={header_filter}",
            f"-p={build_dir}",
            filepath,
        ]

        if not args.dry_run:
            cmd.extend(["-fix", "-fix-errors"])

        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True
        )

        output = result.stdout + result.stderr

        # Check if any fixes were suggested/applied
        if "member-init" in output:
            if args.dry_run:
                print(output)
            fixed_count += 1

        if result.returncode != 0 and "error:" in result.stderr and "member-init" not in result.stderr:
            error_count += 1

    print()
    if args.dry_run:
        print(f"Would fix issues in {fixed_count} files")
    else:
        print(f"Processed {len(files_to_check)} files, {fixed_count} had fixes applied")

    return 0 if error_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
