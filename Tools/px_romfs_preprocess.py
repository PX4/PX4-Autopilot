#!/usr/bin/env python3
"""
Preprocesses ROMFS files with C preprocessor to enable KConfig support.

This script processes all rc* files in the ROMFS directory through the C preprocessor,
allowing use of #ifdef, #ifndef, #if, #else, #endif directives with KConfig definitions.
"""

import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path


def preprocess_file(file_path, kconfig_header, cpp_command):
    """
    Preprocess a single file through the C preprocessor.

    Uses % as the preprocessor directive symbol (instead of #) to avoid conflicts
    with shell comments. Converts %ifdef, %ifndef, %if, %else, %endif to
    #ifdef, #ifndef, #if, #else, #endif before preprocessing.

    Args:
        file_path: Path to the file to preprocess
        kconfig_header: Path to the px4_boardconfig.h header
        cpp_command: C preprocessor command (usually the C compiler)
    """
    # Read original file
    with open(file_path, 'r') as f:
        original_content = f.read()

    # Process the file line by line:
    # 1. Remove shell comment lines (to avoid conflicts with CPP)
    # 2. Convert % preprocessor directives to # directives
    lines = original_content.split('\n')
    converted_lines = []

    for line in lines:
        stripped = line.lstrip()

        # Check if line starts with % followed by a preprocessor keyword
        if stripped.startswith('%ifdef ') or stripped.startswith('%ifdef\t'):
            # Preserve leading whitespace, convert %ifdef to #ifdef
            converted_lines.append(line.replace('%ifdef', '#ifdef', 1))
        elif stripped.startswith('%ifndef ') or stripped.startswith('%ifndef\t'):
            converted_lines.append(line.replace('%ifndef', '#ifndef', 1))
        elif stripped.startswith('%if '):
            converted_lines.append(line.replace('%if', '#if', 1))
        elif stripped.startswith('%elif '):
            converted_lines.append(line.replace('%elif', '#elif', 1))
        elif stripped.startswith('%else'):
            converted_lines.append(line.replace('%else', '#else', 1))
        elif stripped.startswith('%endif'):
            converted_lines.append(line.replace('%endif', '#endif', 1))
        elif stripped.startswith('#') and not stripped.startswith('#!'):
            # Remove shell comment lines (but keep shebang)
            # This prevents "# if ..." comments from being interpreted as "#if" by CPP
            continue
        else:
            converted_lines.append(line)
    converted_content = '\n'.join(converted_lines)

    # Create temporary file with include directive and converted content
    with tempfile.NamedTemporaryFile(mode='w', suffix='.in', delete=False) as tmp:
        tmp.write(f'#include "{kconfig_header}"\n')
        tmp.write(converted_content)
        tmp_path = tmp.name

    try:
        # Run C preprocessor
        # -P: don't generate #line directives
        # -E: preprocess only
        # -undef: don't predefine any non-standard macros
        # -nostdinc: don't search standard include directories
        # -x assembler-with-cpp: treat input as assembly (allows # comments to pass through)
        result = subprocess.run(
            [cpp_command, '-P', '-E', '-undef', '-nostdinc', '-x', 'assembler-with-cpp', tmp_path],
            capture_output=True,
            text=True,
            check=True
        )

        preprocessed = result.stdout

        # Clean up the output:
        # 1. Remove empty lines at the beginning
        # 2. Remove lines that are just whitespace
        lines = preprocessed.split('\n')
        cleaned_lines = []
        started = False

        for line in lines:
            # Skip empty lines at the beginning
            if not started and not line.strip():
                continue
            started = True
            cleaned_lines.append(line)

        # Remove trailing empty lines
        while cleaned_lines and not cleaned_lines[-1].strip():
            cleaned_lines.pop()

        # Write preprocessed content back
        with open(file_path, 'w') as f:
            f.write('\n'.join(cleaned_lines))
            if cleaned_lines:  # Add final newline if file is not empty
                f.write('\n')

        return True

    except subprocess.CalledProcessError as e:
        print(f"Error preprocessing {file_path}: {e}")
        print(f"stderr: {e.stderr}")
        return False
    finally:
        # Clean up temporary file
        try:
            os.unlink(tmp_path)
        except:
            pass


def main():
    parser = argparse.ArgumentParser(description='Preprocess ROMFS files with KConfig definitions')
    parser.add_argument('--romfs-dir', required=True, help='ROMFS root directory')
    parser.add_argument('--kconfig-header', required=True, help='Path to px4_boardconfig.h')
    parser.add_argument('--cpp', required=True, help='C preprocessor command')
    parser.add_argument('--pattern', default='rc*', help='File pattern to preprocess (default: rc*)')

    args = parser.parse_args()

    # Verify inputs
    romfs_dir = Path(args.romfs_dir)
    kconfig_header = Path(args.kconfig_header)

    if not romfs_dir.exists():
        print(f"Error: ROMFS directory not found: {romfs_dir}")
        return 1

    if not kconfig_header.exists():
        print(f"Error: KConfig header not found: {kconfig_header}")
        return 1

    # Find all files to preprocess in init.d directory
    init_d_dir = romfs_dir / 'init.d'
    if not init_d_dir.exists():
        print(f"Warning: init.d directory not found in {romfs_dir}")
        return 0

    # Find all rc* files (shell scripts)
    files_to_process = []
    for pattern in ['rc*', 'rcS']:
        files_to_process.extend(init_d_dir.glob(pattern))

    # Also check subdirectories like airframes
    for subdir in init_d_dir.iterdir():
        if subdir.is_dir():
            for pattern in ['rc*']:
                files_to_process.extend(subdir.glob(pattern))

    # Remove duplicates and filter only files
    files_to_process = list(set([f for f in files_to_process if f.is_file()]))

    if not files_to_process:
        print(f"Warning: No files matching pattern '{args.pattern}' found in {init_d_dir}")
        return 0

    print(f"Preprocessing {len(files_to_process)} ROMFS files with KConfig definitions...")

    # Process each file
    success_count = 0
    for file_path in sorted(files_to_process):
        rel_path = file_path.relative_to(romfs_dir)
        print(f"  Processing: {rel_path}")
        if preprocess_file(file_path, kconfig_header.absolute(), args.cpp):
            success_count += 1
        else:
            print(f"  Warning: Failed to preprocess {rel_path}")

    print(f"Successfully preprocessed {success_count}/{len(files_to_process)} files")

    return 0 if success_count == len(files_to_process) else 1


if __name__ == '__main__':
    sys.exit(main())
