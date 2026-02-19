#!/usr/bin/env python3
"""Upload files to PX4 SD card via MAVLink FTP.

Pushes a local directory tree to the device's SD card using MAVSDK's
FTP plugin. Intended to be used after firmware upload to push metadata
files (parameters, events, actuators) for constrained flash boards.

Uses are_files_identical to skip files that already match on the device.

Usage:
    python3 px4_sdcard_upload.py --port serial:///dev/ttyACM0:57600 --sdcard-dir build/board/sdcard
    python3 px4_sdcard_upload.py --sdcard-dir build/board/sdcard  # auto-detect port

Requires: mavsdk
"""

import argparse
import asyncio
import glob
import os
import platform
import sys
import time

from mavsdk import System
from mavsdk.ftp import FtpError


def detect_port():
    """Try to auto-detect a PX4 serial port."""
    candidates = []
    system = platform.system()

    if system == "Linux":
        candidates.extend(sorted(glob.glob("/dev/serial/by-id/usb-*PX4*")))
        candidates.extend(sorted(glob.glob("/dev/serial/by-id/usb-*px4*")))
        candidates.extend(sorted(glob.glob("/dev/ttyACM*")))
    elif system == "Darwin":
        candidates.extend(sorted(glob.glob("/dev/cu.usbmodem*")))
    elif system == "Windows":
        for i in range(10):
            candidates.append(f"COM{i}")

    for port in candidates:
        if os.path.exists(port):
            return port

    return None


def make_system_address(port, baudrate):
    """Convert a device path to a MAVSDK system address string."""
    if "://" in port:
        return port
    return f"serial://{port}:{baudrate}"


async def list_remote_files(drone, remote_dir):
    """Return set of filenames in remote_dir, or None if dir doesn't exist."""
    try:
        result = await drone.ftp.list_directory(remote_dir)
        return set(result.files)
    except FtpError:
        return None


async def ensure_remote_dirs(drone, remote_dir, device_root, created_dirs):
    """Create remote directory hierarchy as needed."""
    if remote_dir in created_dirs:
        return
    suffix = remote_dir[len(device_root.rstrip("/")):]
    parts = suffix.strip("/").split("/")
    path = device_root.rstrip("/")
    for part in parts:
        if not part:
            continue
        path = path + "/" + part
        if path not in created_dirs:
            print(f"  Creating directory: {path}")
            try:
                await drone.ftp.create_directory(path)
            except FtpError:
                pass  # Already exists
            created_dirs.add(path)


async def upload_directory(drone, local_dir, device_root):
    """Walk local_dir and upload changed files to device_root on the device."""
    local_dir = os.path.abspath(local_dir)
    upload_count = 0
    skip_count = 0
    fail_count = 0
    created_dirs = set()
    remote_file_cache = {}  # remote_dir -> set of filenames or None

    for dirpath, _dirnames, filenames in os.walk(local_dir):
        for filename in filenames:
            local_path = os.path.join(dirpath, filename)
            rel_path = os.path.relpath(local_path, local_dir)
            remote_path = device_root.rstrip("/") + "/" + rel_path.replace(os.sep, "/")
            remote_dir = os.path.dirname(remote_path)
            file_size = os.path.getsize(local_path)

            # List remote directory once to know which files exist
            if remote_dir not in remote_file_cache:
                remote_file_cache[remote_dir] = await list_remote_files(
                    drone, remote_dir)
                if remote_file_cache[remote_dir] is not None:
                    created_dirs.add(remote_dir)

            remote_files = remote_file_cache[remote_dir]

            if remote_files is not None and filename in remote_files:
                # File exists remotely, check CRC
                print(f"  {rel_path}: checking...", end="", flush=True)
                try:
                    if await drone.ftp.are_files_identical(
                            local_path, remote_path):
                        print(" identical, skipped")
                        skip_count += 1
                        continue
                    else:
                        print(f" changed, uploading ({file_size} bytes)...",
                              end="", flush=True)
                except FtpError as e:
                    print(f" check failed ({e}), uploading ({file_size}"
                          f" bytes)...", end="", flush=True)
            else:
                # File doesn't exist remotely
                await ensure_remote_dirs(
                    drone, remote_dir, device_root, created_dirs)
                print(f"  {rel_path}: new, uploading ({file_size} bytes)...",
                      end="", flush=True)

            try:
                async for _progress in drone.ftp.upload(local_path, remote_dir):
                    pass
                print(" done")
                upload_count += 1
            except FtpError as e:
                print(f" FAILED ({e})")
                fail_count += 1

    return upload_count, skip_count, fail_count


async def run(args):
    port = args.port
    if port is None:
        print("Waiting for serial port...")
        deadline = time.monotonic() + args.port_timeout
        while time.monotonic() < deadline:
            port = detect_port()
            if port is not None:
                break
            await asyncio.sleep(0.5)
        if port is None:
            print("Error: no PX4 serial port detected. "
                  "Specify one with --port.")
            sys.exit(1)
        print(f"Detected port: {port}")

    system_address = make_system_address(port, args.baudrate)
    print(f"Connecting to {system_address} ...")

    if args.mavsdk_server:
        host, _, port_str = args.mavsdk_server.partition(":")
        grpc_port = int(port_str) if port_str else 50051
        drone = System(mavsdk_server_address=host, port=grpc_port)
        await drone.connect()
    else:
        drone = System()
        await drone.connect(system_address=system_address)

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected.")
            break

    print(f"Checking {args.total_files} file(s) from {args.sdcard_dir} "
          f"against {args.device_root} ...")

    uploaded, skipped, failed = await upload_directory(
        drone, args.sdcard_dir, args.device_root)

    print(f"\nDone: {uploaded} uploaded, {skipped} skipped, {failed} failed "
          f"(out of {args.total_files} files)")

    if failed > 0:
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description="Upload files to PX4 SD card via MAVLink FTP")
    parser.add_argument("--port", default=None,
                        help="MAVLink connection string (e.g. serial:///dev/ttyACM0:57600, "
                             "udp://:14540). Auto-detected if not specified.")
    parser.add_argument("--baudrate", type=int, default=57600,
                        help="Serial baud rate (default: 57600)")
    parser.add_argument("--sdcard-dir", required=True,
                        help="Local directory tree to upload (mirrors SD card layout)")
    parser.add_argument("--device-root", default="/fs/microsd",
                        help="SD card root path on device (default: /fs/microsd)")
    parser.add_argument("--port-timeout", type=float, default=5,
                        help="Seconds to wait for serial port to appear (default: 5)")
    parser.add_argument("--mavsdk-server", default=None,
                        help="Connect to external mavsdk_server at host:port "
                             "(e.g. localhost:50051) instead of starting one")
    args = parser.parse_args()

    if not os.path.isdir(args.sdcard_dir):
        print(f"Error: sdcard directory not found: {args.sdcard_dir}")
        sys.exit(1)

    total_files = sum(len(files) for _, _, files in os.walk(args.sdcard_dir))
    if total_files == 0:
        print("No files to upload.")
        sys.exit(0)

    args.total_files = total_files
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
