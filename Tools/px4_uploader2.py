#!/usr/bin/env python3
############################################################################
#
#   Copyright (c) 2012-2026 PX4 Development Team. All rights reserved.
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
PX4 Firmware Uploader v2 - Rewritten with improved error handling and debugging.

This script uploads firmware to PX4-based flight controllers via their bootloader.

The PX4 firmware file is a JSON-encoded Python object containing metadata fields
and a zlib-compressed base64-encoded firmware image.

Key improvements over px_uploader.py:
- Proper exception hierarchy with full context
- Verbose/debug logging support
- Self-contained port detection
- Clean separation of concerns
- Configurable timeouts
- Better progress reporting
"""

import argparse
import base64
import glob
import json
import logging
import os
import socket
import struct
import sys
import time
import zlib
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path
from typing import Optional

# Check Python version early
if sys.version_info < (3, 7):
    print("Python 3.7 or later is required.", file=sys.stderr)
    sys.exit(1)

try:
    import serial
    import serial.tools.list_ports
except ImportError as e:
    print(f"Failed to import pyserial: {e}", file=sys.stderr)
    print("\nInstall it with: python -m pip install pyserial", file=sys.stderr)
    sys.exit(1)


# =============================================================================
# Logging Configuration
# =============================================================================

logger = logging.getLogger("px4_uploader")


def setup_logging(verbose: bool = False, debug: bool = False) -> None:
    """Configure logging based on verbosity level.

    Args:
        verbose: Enable INFO level logging for operational details
        debug: Enable DEBUG level logging for protocol-level details
    """
    if debug:
        level = logging.DEBUG
        fmt = "%(asctime)s.%(msecs)03d [%(levelname)s] %(name)s: %(message)s"
    elif verbose:
        level = logging.INFO
        fmt = "[%(levelname)s] %(message)s"
    else:
        level = logging.WARNING
        fmt = "%(message)s"

    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter(fmt, datefmt="%H:%M:%S"))
    logger.addHandler(handler)
    logger.setLevel(level)

    # Also check environment variable
    if os.environ.get("PX4_UPLOADER_DEBUG", "").lower() in ("1", "true", "yes"):
        logger.setLevel(logging.DEBUG)


# =============================================================================
# Exception Hierarchy
# =============================================================================


class UploadError(Exception):
    """Base exception for all upload-related errors."""

    def __init__(
        self,
        message: str,
        port: Optional[str] = None,
        operation: Optional[str] = None,
        details: Optional[str] = None,
    ):
        self.port = port
        self.operation = operation
        self.details = details

        parts = [message]
        if port:
            parts.append(f"port={port}")
        if operation:
            parts.append(f"during {operation}")
        if details:
            parts.append(f"({details})")

        super().__init__(" ".join(parts))


class ProtocolError(UploadError):
    """Error in bootloader protocol communication."""

    pass


class ConnectionError(UploadError):
    """Error establishing or maintaining serial connection."""

    pass


class FirmwareError(UploadError):
    """Error loading or validating firmware file."""

    pass


class BoardMismatchError(UploadError):
    """Firmware not suitable for the connected board."""

    pass


class TimeoutError(UploadError):
    """Operation timed out."""

    pass


class SiliconErrataError(UploadError):
    """Board has silicon errata that prevents safe operation."""

    pass


# =============================================================================
# Protocol Constants
# =============================================================================


class BootloaderCommand(IntEnum):
    """Bootloader protocol commands."""

    NOP = 0x00  # Guaranteed to be discarded by the bootloader
    GET_SYNC = 0x21
    GET_DEVICE = 0x22
    CHIP_ERASE = 0x23
    CHIP_VERIFY = 0x24  # rev2 only
    PROG_MULTI = 0x27
    READ_MULTI = 0x28  # rev2 only
    GET_CRC = 0x29  # rev3+
    GET_OTP = 0x2A  # rev4+, get a word from OTP area
    GET_SN = 0x2B  # rev4+, get a word from SN area
    GET_CHIP = 0x2C  # rev5+, get chip version
    SET_BOOT_DELAY = 0x2D  # rev5+, set boot delay
    GET_CHIP_DES = 0x2E  # rev5+, get chip description in ASCII
    GET_VERSION = 0x2F  # rev5+, get bootloader version in ASCII
    REBOOT = 0x30
    CHIP_FULL_ERASE = 0x40  # Full erase of flash, rev6+


class BootloaderResponse(IntEnum):
    """Bootloader response codes."""

    INSYNC = 0x12
    EOC = 0x20
    OK = 0x10
    FAILED = 0x11
    INVALID = 0x13  # rev3+
    BAD_SILICON_REV = 0x14  # rev5+


class DeviceInfo(IntEnum):
    """Device information parameter codes."""

    BL_REV = 0x01  # Bootloader protocol revision
    BOARD_ID = 0x02  # Board type
    BOARD_REV = 0x03  # Board revision
    FLASH_SIZE = 0x04  # Max firmware size in bytes


@dataclass
class ProtocolConfig:
    """Protocol configuration constants."""

    BL_REV_MIN: int = 2  # Minimum supported bootloader protocol
    BL_REV_MAX: int = 6  # Maximum supported bootloader protocol
    PROG_MULTI_MAX: int = (
        252  # Max bytes per PROG_MULTI (protocol max 255, must be multiple of 4)
    )
    READ_MULTI_MAX: int = 252  # Max bytes per READ_MULTI
    MAX_DES_LENGTH: int = 20  # Max chip description length


# =============================================================================
# Known PX4 USB Vendor/Product IDs
# =============================================================================

# Known VID/PID combinations for PX4 bootloaders and devices
PX4_USB_IDS: list[tuple[int, int, str]] = [
    # (Vendor ID, Product ID, Description)
    (0x26AC, 0x0010, "3D Robotics PX4 FMU"),
    (0x26AC, 0x0011, "3D Robotics PX4 BL"),
    (0x26AC, 0x0012, "3D Robotics PX4IO"),
    (0x26AC, 0x0032, "3D Robotics PX4 FMU v5"),
    (0x3185, 0x0035, "Holybro Durandal"),
    (0x3185, 0x0036, "Holybro Kakute"),
    (0x3162, 0x004B, "Holybro Pixhawk 4"),
    (0x1FC9, 0x001C, "NXP FMUK66"),
    (0x2DAE, 0x1058, "Cube Orange"),
    (0x2DAE, 0x1016, "Cube Black"),
    (0x2DAE, 0x1011, "Cube Yellow"),
    (0x0483, 0x5740, "STMicroelectronics Virtual COM Port"),  # Generic ST bootloader
    (0x1209, 0x5740, "Generic STM32"),
    (0x1209, 0x5741, "ArduPilot"),
    (0x2341, 0x8036, "Arduino Leonardo"),  # Some PX4 boards use this
]


# =============================================================================
# Firmware Class
# =============================================================================


@dataclass
class Firmware:
    """Loads and validates a PX4 firmware file.

    The firmware file is JSON containing metadata and a zlib-compressed,
    base64-encoded firmware image.

    Attributes:
        path: Path to the firmware file
        board_id: Target board ID from firmware metadata
        board_revision: Board revision from metadata
        image: Decompressed firmware binary (padded to 4-byte alignment)
        image_size: Original image size before padding
        image_maxsize: Maximum image size the firmware was built for
        description: Full firmware metadata dictionary
    """

    path: Path
    board_id: int = field(init=False)
    board_revision: int = field(init=False)
    image: bytes = field(init=False)
    image_size: int = field(init=False)
    image_maxsize: int = field(init=False)
    description: dict = field(init=False)

    def __post_init__(self):
        """Load and validate the firmware file."""
        self.path = Path(self.path)
        self._load()

    def _load(self) -> None:
        """Load firmware from JSON file."""
        logger.info(f"Loading firmware from {self.path}")

        if not self.path.exists():
            raise FirmwareError(f"Firmware file not found: {self.path}")

        try:
            with open(self.path, "r") as f:
                self.description = json.load(f)
        except json.JSONDecodeError as e:
            raise FirmwareError(f"Invalid firmware JSON: {e}", details=str(self.path))
        except IOError as e:
            raise FirmwareError(
                f"Cannot read firmware file: {e}", details=str(self.path)
            )

        # Extract required fields
        required_fields = ["image", "board_id", "image_size", "image_maxsize"]
        for field_name in required_fields:
            if field_name not in self.description:
                raise FirmwareError(
                    f"Firmware missing required field: {field_name}",
                    details=str(self.path),
                )

        self.board_id = self.description["board_id"]
        self.board_revision = self.description.get("board_revision", 0)
        self.image_size = self.description["image_size"]
        self.image_maxsize = self.description["image_maxsize"]

        # Decompress image
        try:
            compressed = base64.b64decode(self.description["image"])
            image_data = bytearray(zlib.decompress(compressed))
        except (base64.binascii.Error, zlib.error) as e:
            raise FirmwareError(
                f"Cannot decompress firmware image: {e}", details=str(self.path)
            )

        # Pad to 4-byte alignment
        while len(image_data) % 4 != 0:
            image_data.append(0xFF)

        self.image = bytes(image_data)

        logger.info(
            f"Loaded firmware: board_id={self.board_id}, "
            f"size={self.image_size} bytes ({self.usage_percent:.1f}%)"
        )

    @property
    def usage_percent(self) -> float:
        """Percentage of maximum flash used."""
        return (self.image_size / self.image_maxsize) * 100.0

    def crc(self, padlen: int) -> int:
        """Calculate CRC32 of firmware image with padding.

        Args:
            padlen: Total length to pad image to (typically flash size)

        Returns:
            CRC32 value matching bootloader's calculation
        """
        state = 0xFFFFFFFF
        state = zlib.crc32(self.image, state)

        padding_length = padlen - len(self.image)
        if padding_length > 0:
            padding = b"\xff" * padding_length
            state = zlib.crc32(padding, state)

        return (state ^ 0xFFFFFFFF) & 0xFFFFFFFF


# =============================================================================
# Serial Transport
# =============================================================================


class SerialTransport:
    """Handles serial port communication with proper resource management.

    Provides context manager support for automatic cleanup and configurable
    timeouts.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 0.5,
        write_timeout: float = 2.0,
    ):
        """Initialize serial transport.

        Args:
            port: Serial port path (e.g., /dev/ttyUSB0, COM3)
            baudrate: Baud rate for communication
            timeout: Read timeout in seconds
            write_timeout: Write timeout in seconds (0 = no timeout)
        """
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self._port: Optional[serial.Serial] = None
        self._chartime = 10.0 / baudrate  # 8N1 = 10 bits per byte

    def __enter__(self) -> "SerialTransport":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    def open(self) -> None:
        """Open the serial port."""
        if self._port is not None and self._port.is_open:
            return

        logger.debug(f"Opening serial port {self.port_name} at {self.baudrate} baud")

        try:
            self._port = serial.Serial(
                self.port_name,
                self.baudrate,
                timeout=self.timeout,
                write_timeout=self.write_timeout,
            )
        except serial.SerialException as e:
            raise ConnectionError(
                f"Cannot open serial port: {e}", port=self.port_name, operation="open"
            )

    def close(self) -> None:
        """Close the serial port."""
        if self._port is not None:
            logger.debug(f"Closing serial port {self.port_name}")
            try:
                self._port.close()
            except Exception as e:
                logger.warning(f"Error closing port {self.port_name}: {e}")
            self._port = None

    @property
    def is_open(self) -> bool:
        """Check if port is open."""
        return self._port is not None and self._port.is_open

    def send(self, data: bytes) -> None:
        """Send data over serial port.

        Args:
            data: Bytes to send

        Raises:
            ConnectionError: If send fails
        """
        if not self.is_open:
            raise ConnectionError(
                "Port not open", port=self.port_name, operation="send"
            )

        logger.debug(f"TX: {data.hex()}")

        try:
            self._port.write(data)
        except serial.SerialException as e:
            raise ConnectionError(
                f"Write failed: {e}", port=self.port_name, operation="send"
            )

    def recv(self, count: int = 1, timeout: Optional[float] = None) -> bytes:
        """Receive data from serial port.

        Args:
            count: Number of bytes to receive
            timeout: Override default timeout

        Returns:
            Received bytes

        Raises:
            TimeoutError: If timeout expires before all bytes received
            ConnectionError: If read fails
        """
        if not self.is_open:
            raise ConnectionError(
                "Port not open", port=self.port_name, operation="recv"
            )

        old_timeout = self._port.timeout
        if timeout is not None:
            self._port.timeout = timeout

        try:
            data = self._port.read(count)
        except serial.SerialException as e:
            raise ConnectionError(
                f"Read failed: {e}", port=self.port_name, operation="recv"
            )
        finally:
            if timeout is not None:
                self._port.timeout = old_timeout

        if len(data) < count:
            raise TimeoutError(
                f"Timeout waiting for {count} bytes, got {len(data)}",
                port=self.port_name,
                operation="recv",
            )

        logger.debug(f"RX: {data.hex()}")
        return data

    def flush(self) -> None:
        """Flush output buffer."""
        if self._port is not None:
            self._port.flush()

    def reset_buffers(self) -> None:
        """Reset input and output buffers."""
        if self._port is not None:
            self._port.reset_input_buffer()
            self._port.reset_output_buffer()

    def set_baudrate(self, baudrate: int) -> None:
        """Change baud rate.

        Args:
            baudrate: New baud rate
        """
        logger.debug(f"Changing baudrate to {baudrate}")
        self.baudrate = baudrate
        self._chartime = 10.0 / baudrate

        if self._port is not None:
            try:
                self._port.baudrate = baudrate
            except (serial.SerialException, NotImplementedError) as e:
                logger.debug(f"Cannot change baudrate: {e}")
                raise

    @property
    def chartime(self) -> float:
        """Time to transmit one character."""
        return self._chartime


# =============================================================================
# Bootloader Protocol
# =============================================================================


class BootloaderProtocol:
    """Implements the PX4 bootloader protocol.

    Handles all communication with the bootloader including sync,
    identification, programming, and verification.
    """

    # Reboot command sequences
    NSH_INIT = b"\r\r\r"
    NSH_REBOOT_BL = b"reboot -b\n"
    NSH_REBOOT = b"reboot\n"

    # MAVLink reboot commands (MAVLink v1 COMMAND_LONG with MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)
    MAVLINK_REBOOT_ID1 = bytes.fromhex(
        "fe2172ff004c00004040000000000000000000000000"
        "000000000000000000000000f600010000536b"
    )
    MAVLINK_REBOOT_ID0 = bytes.fromhex(
        "fe2145ff004c00004040000000000000000000000000"
        "000000000000000000000000f600000000cc37"
    )

    def __init__(
        self,
        transport: SerialTransport,
        sync_timeout: float = 0.5,
        erase_timeout: float = 30.0,
        windowed: bool = False,
    ):
        """Initialize bootloader protocol handler.

        Args:
            transport: Serial transport instance
            sync_timeout: Timeout for sync operations
            erase_timeout: Timeout for chip erase
            windowed: Use windowed mode for faster uploads on real serial ports
        """
        self.transport = transport
        self.sync_timeout = sync_timeout
        self.erase_timeout = erase_timeout

        # Board info (populated by identify())
        self.bl_rev: int = 0
        self.board_type: int = 0
        self.board_rev: int = 0
        self.fw_maxsize: int = 0
        self.version: str = "unknown"
        self.otp: bytes = b""
        self.sn: bytes = b""
        self.chip_id: int = 0
        self.chip_family: str = ""
        self.chip_revision: str = ""

        # Windowed mode for faster uploads on some interfaces
        self._windowed_mode = windowed
        self._window_size = 0
        self._window_max = 256
        self._window_per = 2  # SYNC + result per block

    def _send_command(self, cmd: int, *args: bytes) -> None:
        """Send a command to the bootloader.

        Args:
            cmd: Command byte
            *args: Additional data bytes
        """
        data = bytes([cmd]) + b"".join(args) + bytes([BootloaderResponse.EOC])
        self.transport.send(data)

    def _recv_int(self) -> int:
        """Receive a 32-bit little-endian integer."""
        raw = self.transport.recv(4)
        return struct.unpack("<I", raw)[0]

    def _get_sync(self, flush: bool = True) -> None:
        """Wait for and validate sync response.

        Args:
            flush: Whether to flush output buffer first

        Raises:
            ProtocolError: If response is not valid INSYNC + OK
        """
        if flush:
            self.transport.flush()

        insync = self.transport.recv(1)
        if insync[0] != BootloaderResponse.INSYNC:
            raise ProtocolError(
                f"Expected INSYNC (0x{BootloaderResponse.INSYNC:02X}), "
                f"got 0x{insync[0]:02X}",
                port=self.transport.port_name,
                operation="sync",
            )

        result = self.transport.recv(1)
        if result[0] == BootloaderResponse.INVALID:
            raise ProtocolError(
                "Bootloader reports INVALID OPERATION", port=self.transport.port_name
            )
        if result[0] == BootloaderResponse.FAILED:
            raise ProtocolError(
                "Bootloader reports OPERATION FAILED", port=self.transport.port_name
            )
        if result[0] == BootloaderResponse.BAD_SILICON_REV:
            raise SiliconErrataError(
                "Chip has silicon errata, programming not supported.\n"
                "See https://docs.px4.io/main/en/flight_controller/silicon_errata.html",
                port=self.transport.port_name,
            )
        if result[0] != BootloaderResponse.OK:
            raise ProtocolError(
                f"Expected OK (0x{BootloaderResponse.OK:02X}), got 0x{result[0]:02X}",
                port=self.transport.port_name,
            )

    def _try_sync(self) -> bool:
        """Attempt to get sync without raising exceptions.

        Returns:
            True if sync successful, False otherwise
        """
        try:
            self.transport.flush()
            insync = self.transport.recv(1, timeout=0.1)
            if insync[0] != BootloaderResponse.INSYNC:
                return False
            result = self.transport.recv(1, timeout=0.1)
            if result[0] == BootloaderResponse.BAD_SILICON_REV:
                raise SiliconErrataError(
                    "Chip has silicon errata, programming not supported",
                    port=self.transport.port_name,
                )
            return result[0] == BootloaderResponse.OK
        except TimeoutError:
            return False
        except Exception as e:
            logger.debug(f"Sync attempt failed: {e}")
            return False

    def _validate_sync_window(self, count: int) -> None:
        """Validate multiple sync responses for windowed mode.

        Args:
            count: Number of sync responses to validate (each is 2 bytes)
        """
        if count <= 0:
            return

        data = self.transport.recv(count)
        if len(data) != count:
            raise ProtocolError(
                f"Expected {count} bytes, got {len(data)}",
                port=self.transport.port_name,
                operation="ack_window",
            )

        for i in range(0, len(data), 2):
            if data[i] != BootloaderResponse.INSYNC:
                raise ProtocolError(
                    f"Expected INSYNC at byte {i}, got 0x{data[i]:02X}",
                    port=self.transport.port_name,
                )
            if data[i + 1] == BootloaderResponse.INVALID:
                raise ProtocolError(
                    "Bootloader reports INVALID OPERATION",
                    port=self.transport.port_name,
                )
            if data[i + 1] == BootloaderResponse.FAILED:
                raise ProtocolError(
                    "Bootloader reports OPERATION FAILED", port=self.transport.port_name
                )
            if data[i + 1] != BootloaderResponse.OK:
                raise ProtocolError(
                    f"Expected OK, got 0x{data[i + 1]:02X}",
                    port=self.transport.port_name,
                )

    def _detect_interface_type(self) -> None:
        """Detect if connected via USB CDC or real serial port.

        Currently just resets buffers. Windowed mode can be enabled manually
        with --windowed for real serial ports (FTDI, etc.).
        """
        self.transport.reset_buffers()

    def sync(self) -> None:
        """Synchronize with bootloader.

        Sends sync command and waits for valid response.

        Raises:
            ProtocolError: If sync fails
        """
        logger.debug("Syncing with bootloader")
        self.transport.reset_buffers()
        self._send_command(BootloaderCommand.GET_SYNC)
        self._get_sync()
        logger.debug("Sync successful")

    def _get_device_info(self, param: int) -> int:
        """Get device information parameter.

        Args:
            param: DeviceInfo parameter code

        Returns:
            Parameter value
        """
        self._send_command(BootloaderCommand.GET_DEVICE, bytes([param]))
        value = self._recv_int()
        self._get_sync()
        return value

    def _get_otp(self, address: int) -> bytes:
        """Read 4 bytes from OTP area.

        Args:
            address: OTP address (byte offset)

        Returns:
            4 bytes of OTP data
        """
        self._send_command(BootloaderCommand.GET_OTP, struct.pack("<I", address))
        value = self.transport.recv(4)
        self._get_sync()
        return value

    def _get_sn(self, address: int) -> bytes:
        """Read 4 bytes from serial number area.

        Args:
            address: SN address (byte offset)

        Returns:
            4 bytes of SN data
        """
        self._send_command(BootloaderCommand.GET_SN, struct.pack("<I", address))
        value = self.transport.recv(4)
        self._get_sync()
        return value

    def _get_chip(self) -> int:
        """Get chip ID.

        Returns:
            Chip ID value
        """
        self._send_command(BootloaderCommand.GET_CHIP)
        value = self._recv_int()
        self._get_sync()
        return value

    def _get_chip_description(self) -> tuple[str, str]:
        """Get chip family and revision.

        Returns:
            Tuple of (family, revision) strings
        """
        self._send_command(BootloaderCommand.GET_CHIP_DES)
        length = self._recv_int()
        value = self.transport.recv(length)
        self._get_sync()

        pieces = value.split(b",")
        if len(pieces) >= 2:
            return pieces[0].decode("latin-1"), pieces[1].decode("latin-1")
        return "unknown", "unknown"

    def _get_version(self) -> str:
        """Get bootloader version string.

        Returns:
            Version string or "unknown" if not supported
        """
        self._send_command(BootloaderCommand.GET_VERSION)
        try:
            length = self._recv_int()
            value = self.transport.recv(length)
            self._get_sync()
            return value.decode("utf-8", errors="replace")
        except (TimeoutError, ProtocolError):
            # Older bootloaders don't support this
            return "unknown"

    def identify(self) -> None:
        """Identify the connected board.

        Queries bootloader for board information and stores in instance
        attributes.

        Raises:
            ProtocolError: If identification fails or protocol version unsupported
        """
        logger.info("Identifying board...")

        self._detect_interface_type()
        self.sync()

        # Get bootloader protocol revision
        self.bl_rev = self._get_device_info(DeviceInfo.BL_REV)
        logger.info(f"Bootloader protocol: v{self.bl_rev}")

        if self.bl_rev < ProtocolConfig.BL_REV_MIN:
            raise ProtocolError(
                f"Bootloader protocol {self.bl_rev} too old "
                f"(minimum {ProtocolConfig.BL_REV_MIN})",
                port=self.transport.port_name,
            )
        if self.bl_rev > ProtocolConfig.BL_REV_MAX:
            logger.warning(
                f"Bootloader protocol {self.bl_rev} newer than supported "
                f"({ProtocolConfig.BL_REV_MAX}), proceeding with caution"
            )

        # Get board info
        self.board_type = self._get_device_info(DeviceInfo.BOARD_ID)
        self.board_rev = self._get_device_info(DeviceInfo.BOARD_REV)
        self.fw_maxsize = self._get_device_info(DeviceInfo.FLASH_SIZE)

        logger.info(f"Board type: {self.board_type}, revision: {self.board_rev}")
        logger.info(f"Flash size: {self.fw_maxsize} bytes")

        # Get version string (v5+)
        if self.bl_rev >= 5:
            self.version = self._get_version()
            logger.info(f"Bootloader version: {self.version}")

        # Get OTP and serial number (v4+)
        if self.bl_rev >= 4:
            self._read_otp_and_sn()

        # Get chip info (v5+)
        if self.bl_rev >= 5:
            self._read_chip_info()

    def _read_otp_and_sn(self) -> None:
        """Read OTP and serial number data."""
        # Read OTP (32*6 = 192 bytes)
        otp_data = bytearray()
        for addr in range(0, 32 * 6, 4):
            otp_data.extend(self._get_otp(addr))
        self.otp = bytes(otp_data)

        # Read serial number (12 bytes)
        sn_data = bytearray()
        for addr in range(0, 12, 4):
            sn_bytes = self._get_sn(addr)
            sn_data.extend(sn_bytes[::-1])  # Reverse byte order
        self.sn = bytes(sn_data)

        logger.debug(f"Serial number: {self.sn.hex()}")

        # Try to get chip ID
        try:
            self.chip_id = self._get_chip()
            logger.debug(f"Chip ID: 0x{self.chip_id:08X}")
        except (TimeoutError, ProtocolError) as e:
            logger.debug(f"Could not read chip ID: {e}")

    def _read_chip_info(self) -> None:
        """Read chip family and revision (v5+)."""
        try:
            self.chip_family, self.chip_revision = self._get_chip_description()
            logger.info(f"Chip: {self.chip_family} rev {self.chip_revision}")
        except (TimeoutError, ProtocolError) as e:
            logger.debug(f"Could not read chip description: {e}")

    def erase(
        self, force_full: bool = False, progress_callback: Optional[callable] = None
    ) -> None:
        """Erase the flash memory.

        Args:
            force_full: Force full chip erase (v6+)
            progress_callback: Optional callback(progress, total) for progress

        Raises:
            TimeoutError: If erase times out
            ProtocolError: If erase fails
        """
        logger.debug("Erasing flash")

        if force_full and self.bl_rev >= 6:
            logger.debug("Using full chip erase")
            self._send_command(BootloaderCommand.CHIP_FULL_ERASE)
        else:
            self._send_command(BootloaderCommand.CHIP_ERASE)

        # Erase can take a long time, poll for completion
        deadline = time.monotonic() + self.erase_timeout
        usual_duration = 15.0

        while time.monotonic() < deadline:
            elapsed = time.monotonic() - (deadline - self.erase_timeout)
            remaining = deadline - time.monotonic()

            if progress_callback:
                if remaining >= usual_duration:
                    progress_callback(elapsed, usual_duration)
                else:
                    progress_callback(usual_duration, usual_duration)

            if self._try_sync():
                logger.debug("Erase complete")
                if progress_callback:
                    progress_callback(1.0, 1.0)
                return

        raise TimeoutError(
            f"Erase timed out after {self.erase_timeout}s",
            port=self.transport.port_name,
            operation="erase",
        )

    def program(
        self, firmware: Firmware, progress_callback: Optional[callable] = None
    ) -> None:
        """Program firmware to flash.

        Args:
            firmware: Firmware instance to program
            progress_callback: Optional callback(bytes_written, total_bytes)

        Raises:
            ProtocolError: If programming fails
        """
        image = firmware.image
        total = len(image)
        written = 0

        logger.debug(f"Programming {total} bytes")

        # Split image into chunks
        chunk_size = ProtocolConfig.PROG_MULTI_MAX
        chunks = [image[i : i + chunk_size] for i in range(0, total, chunk_size)]

        for i, chunk in enumerate(chunks):
            self._program_multi(chunk)

            if self._windowed_mode:
                self._window_size += self._window_per

                # Periodically validate window
                if (i + 1) % 256 == 0:
                    self._validate_sync_window(self._window_size)
                    self._window_size = 0
            else:
                self._get_sync(flush=False)

            written += len(chunk)
            if progress_callback:
                progress_callback(written, total)

        # Validate any remaining window
        if self._windowed_mode and self._window_size > 0:
            self._validate_sync_window(self._window_size)
            self._window_size = 0

        logger.debug("Programming complete")

    def _program_multi(self, data: bytes) -> None:
        """Program a chunk of data.

        Args:
            data: Bytes to program (max PROG_MULTI_MAX)
        """
        length = len(data)
        cmd = bytes([BootloaderCommand.PROG_MULTI, length]) + data
        cmd += bytes([BootloaderResponse.EOC])
        self.transport.send(cmd)

        if self._windowed_mode:
            # Delay based on transmission time plus flash programming time
            time.sleep(length * self.transport.chartime + 0.001)

    def verify_crc(
        self, firmware: Firmware, progress_callback: Optional[callable] = None
    ) -> None:
        """Verify programmed firmware using CRC (v3+).

        Args:
            firmware: Firmware instance to verify against
            progress_callback: Optional callback for progress

        Raises:
            ProtocolError: If verification fails
        """
        if self.bl_rev < 3:
            raise ProtocolError(
                "CRC verification requires bootloader v3+",
                port=self.transport.port_name,
            )

        logger.debug("Verifying CRC")

        expected_crc = firmware.crc(self.fw_maxsize)
        logger.debug(f"Expected CRC: 0x{expected_crc:08X}")

        self._send_command(BootloaderCommand.GET_CRC)

        # CRC calculation takes time, especially on larger flash
        time.sleep(0.5)

        if progress_callback:
            progress_callback(0.5, 1.0)

        reported_crc = self._recv_int()
        self._get_sync()

        if progress_callback:
            progress_callback(1.0, 1.0)

        logger.debug(f"Reported CRC: 0x{reported_crc:08X}")

        if reported_crc != expected_crc:
            raise ProtocolError(
                f"CRC mismatch: expected 0x{expected_crc:08X}, "
                f"got 0x{reported_crc:08X}",
                port=self.transport.port_name,
                operation="verify",
            )

        logger.debug("CRC verification passed")

    def verify_read(
        self, firmware: Firmware, progress_callback: Optional[callable] = None
    ) -> None:
        """Verify programmed firmware by reading back (v2).

        Args:
            firmware: Firmware instance to verify against
            progress_callback: Optional callback(bytes_verified, total_bytes)

        Raises:
            ProtocolError: If verification fails
        """
        logger.debug("Verifying by read-back")

        self._send_command(BootloaderCommand.CHIP_VERIFY)
        self._get_sync()

        image = firmware.image
        total = len(image)
        verified = 0

        chunk_size = ProtocolConfig.READ_MULTI_MAX
        chunks = [image[i : i + chunk_size] for i in range(0, total, chunk_size)]

        for chunk in chunks:
            length = len(chunk)
            cmd = bytes([BootloaderCommand.READ_MULTI, length])
            cmd += bytes([BootloaderResponse.EOC])
            self.transport.send(cmd)
            self.transport.flush()

            readback = self.transport.recv(length)
            self._get_sync()

            if readback != chunk:
                logger.error(f"Verify failed at offset {verified}")
                logger.debug(f"Expected: {chunk.hex()}")
                logger.debug(f"Got:      {readback.hex()}")
                raise ProtocolError(
                    "Verification failed",
                    port=self.transport.port_name,
                    operation="verify",
                )

            verified += length
            if progress_callback:
                progress_callback(verified, total)

        logger.debug("Read-back verification passed")

    def verify(
        self, firmware: Firmware, progress_callback: Optional[callable] = None
    ) -> None:
        """Verify programmed firmware using appropriate method.

        Uses CRC for v3+ bootloaders, read-back for v2.

        Args:
            firmware: Firmware to verify against
            progress_callback: Optional progress callback
        """
        if self.bl_rev >= 3:
            self.verify_crc(firmware, progress_callback)
        else:
            self.verify_read(firmware, progress_callback)

    def set_boot_delay(self, delay_ms: int) -> None:
        """Set boot delay in flash (v5+).

        Args:
            delay_ms: Boot delay in milliseconds
        """
        if self.bl_rev < 5:
            logger.warning("Boot delay requires bootloader v5+")
            return

        self._send_command(BootloaderCommand.SET_BOOT_DELAY, struct.pack("b", delay_ms))
        self._get_sync()
        logger.info(f"Boot delay set to {delay_ms}ms")

    def reboot(self) -> None:
        """Reboot into the application.

        Raises:
            ProtocolError: If reboot fails (v3+ validates first flash word)
        """
        logger.info("Rebooting to application")
        self._send_command(BootloaderCommand.REBOOT)
        self.transport.flush()

        # v3+ can report failure if first flash word is invalid
        if self.bl_rev >= 3:
            try:
                self._get_sync()
            except TimeoutError:
                # Timeout is expected - board is rebooting
                pass

    def send_reboot_commands(
        self, baudrates: list[int], use_protocol_splitter: bool = False
    ) -> bool:
        """Send reboot commands to try to enter bootloader.

        Tries MAVLink and NSH reboot commands at various baud rates.

        Args:
            baudrates: List of baud rates to try
            use_protocol_splitter: Use protocol splitter framing

        Returns:
            True if commands were sent, False if no more baud rates to try
        """
        for baudrate in baudrates:
            try:
                self.transport.set_baudrate(baudrate)
            except (serial.SerialException, NotImplementedError):
                continue

            logger.info(f"Sending reboot command at {baudrate} baud")

            def send(data: bytes) -> None:
                if use_protocol_splitter:
                    self._send_protocol_splitter_frame(data)
                else:
                    self.transport.send(data)

            try:
                self.transport.flush()
                send(self.MAVLINK_REBOOT_ID0)
                send(self.MAVLINK_REBOOT_ID1)
                send(self.NSH_INIT)
                send(self.NSH_REBOOT_BL)
                send(self.NSH_INIT)
                send(self.NSH_REBOOT)
                self.transport.flush()
            except Exception as e:
                logger.debug(f"Error sending reboot: {e}")
                continue

            return True

        return False

    def _send_protocol_splitter_frame(self, data: bytes) -> None:
        """Send data with protocol splitter framing.

        Header format:
        - Byte 0: Magic ('S' = 0x53)
        - Byte 1: Type (0) | Length high bits (7 bits)
        - Byte 2: Length low bits
        - Byte 3: Checksum (XOR of bytes 0-2)
        """
        magic = 0x53
        len_h = (len(data) >> 8) & 0x7F
        len_l = len(data) & 0xFF
        checksum = magic ^ len_h ^ len_l

        header = bytes([magic, len_h, len_l, checksum])
        self.transport.send(header + data)


# =============================================================================
# Port Detection
# =============================================================================


class PortDetector:
    """Detects PX4-compatible serial ports."""

    # Platform-specific port patterns
    LINUX_PATTERNS = [
        "/dev/serial/by-id/*PX4*",
        "/dev/serial/by-id/*px4*",
        "/dev/serial/by-id/*3D_Robotics*",
        "/dev/serial/by-id/*Autopilot*",
        "/dev/serial/by-id/*Bitcraze*",
        "/dev/serial/by-id/*Gumstix*",
        "/dev/serial/by-id/*Hex*",
        "/dev/serial/by-id/*Holybro*",
        "/dev/serial/by-id/*Cube*",
        "/dev/serial/by-id/*ArduPilot*",
        "/dev/serial/by-id/*BL_FMU*",
        "/dev/serial/by-id/*_BL*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    ]

    MACOS_PATTERNS = [
        "/dev/tty.usbmodemPX*",
        "/dev/tty.usbmodem*",
        "/dev/cu.usbmodemPX*",
        "/dev/cu.usbmodem*",
    ]

    WINDOWS_PATTERNS = [
        "COM*",
    ]

    def __init__(self):
        self.platform = sys.platform

    def detect_ports(self) -> list[str]:
        """Detect available PX4-compatible serial ports.

        Returns:
            List of port paths, prioritized by likelihood of being PX4
        """
        ports = set()

        # First, try USB VID/PID detection
        vid_pid_ports = self._detect_by_vid_pid()
        ports.update(vid_pid_ports)

        # Then try platform-specific patterns
        pattern_ports = self._detect_by_patterns()
        ports.update(pattern_ports)

        # Sort by priority (VID/PID matches first)
        result = []
        for port in vid_pid_ports:
            if port in ports:
                result.append(port)
                ports.discard(port)
        result.extend(sorted(ports))

        logger.info(f"Detected {len(result)} potential ports: {result}")
        return result

    def _detect_by_vid_pid(self) -> list[str]:
        """Detect ports by USB Vendor/Product ID.

        Returns:
            List of ports matching known PX4 VID/PIDs
        """
        ports = []
        known_ids = {(vid, pid) for vid, pid, _ in PX4_USB_IDS}

        try:
            for port_info in serial.tools.list_ports.comports():
                if (port_info.vid, port_info.pid) in known_ids:
                    logger.debug(
                        f"Found PX4 device: {port_info.device} "
                        f"(VID=0x{port_info.vid:04X}, PID=0x{port_info.pid:04X})"
                    )
                    ports.append(port_info.device)
        except Exception as e:
            logger.debug(f"VID/PID detection failed: {e}")

        return ports

    def _detect_by_patterns(self) -> list[str]:
        """Detect ports by platform-specific glob patterns.

        Returns:
            List of ports matching patterns
        """
        if self.platform.startswith("linux"):
            patterns = self.LINUX_PATTERNS
        elif self.platform == "darwin":
            patterns = self.MACOS_PATTERNS
        elif self.platform.startswith("win") or self.platform == "cygwin":
            patterns = self.WINDOWS_PATTERNS
        else:
            patterns = []

        ports = []
        for pattern in patterns:
            matches = glob.glob(pattern)
            ports.extend(matches)

        return list(set(ports))

    def expand_patterns(self, patterns: list[str]) -> list[str]:
        """Expand glob patterns to actual port paths.

        Args:
            patterns: List of port paths or glob patterns

        Returns:
            List of expanded port paths
        """
        ports = []
        for pattern in patterns:
            if "*" in pattern or "?" in pattern:
                matches = glob.glob(pattern)
                if matches:
                    ports.extend(matches)
                else:
                    logger.debug(f"Pattern matched no ports: {pattern}")
            else:
                ports.append(pattern)

        return list(set(ports))


# =============================================================================
# Progress Display
# =============================================================================


class UploadProgressBar:
    """Unified progress bar for the entire upload process.

    Shows a single progress bar with phases:
    - Erase: 0-49%
    - Program: 50-99%
    - Verify: 99-100%
    """

    ERASE_START = 0
    ERASE_END = 49
    PROGRAM_START = 50
    PROGRAM_END = 99
    VERIFY_START = 99
    VERIFY_END = 100

    def __init__(self, noninteractive: bool = False):
        # Use noninteractive mode if flag is set OR if not a TTY
        self._noninteractive = noninteractive or not sys.stdout.isatty()
        self._start_time = time.monotonic()
        self._last_percent = -1
        self._last_printed_percent = -1
        self._phase = "Erase"

    def _render(self, percent: int) -> None:
        """Render the progress bar."""
        if not self._noninteractive and percent == self._last_percent and percent < 100:
            return

        if self._noninteractive:
            # Print at 5% increments (0, 5, 10, ...) plus 99% for verify
            next_milestone = ((self._last_printed_percent // 5) + 1) * 5
            if self._last_printed_percent < 0:
                next_milestone = 0
            # Special case: print 99% only when in Verify phase
            is_verify_99 = (
                self._phase == "Verify"
                and self._last_printed_percent < 99
                and percent >= 99
            )
            if is_verify_99:
                next_milestone = 99
            should_print = percent >= next_milestone or (
                percent >= 100 and self._last_printed_percent < 100
            )
            # Don't print 99% unless we're in Verify phase
            if (
                should_print
                and percent >= 99
                and percent < 100
                and self._phase != "Verify"
            ):
                should_print = False
            if should_print:
                # Print at the milestone, not the actual percent
                if percent >= 100:
                    print_percent = 100
                elif percent >= 99 and self._phase == "Verify":
                    print_percent = 99
                else:
                    print_percent = (percent // 5) * 5
                if print_percent > self._last_printed_percent:
                    if print_percent >= 100:
                        phase = "Done"
                    elif self._phase == "Erase":
                        phase = "Erasing"
                    elif self._phase == "Program":
                        phase = "Programming"
                    elif self._phase == "Verify":
                        phase = "Verifying"
                    else:
                        phase = self._phase
                    print(f"{phase}, progress: {print_percent}%")
                    self._last_printed_percent = print_percent
            self._last_percent = percent
            return

        # Step through each percent for smooth animation
        if self._last_percent >= 0 and percent > self._last_percent + 1:
            for p in range(self._last_percent + 1, percent):
                self._render_single(p)
                time.sleep(0.02)

        self._render_single(percent)
        self._last_percent = percent

    def _render_single(self, percent: int) -> None:
        """Render a single frame of the progress bar."""
        bar_width = 30
        filled_exact = bar_width * percent / 100.0
        filled_full = int(filled_exact)
        filled_partial = filled_exact - filled_full

        # Unicode block characters for smooth progress
        blocks = " ▏▎▍▌▋▊▉█"
        partial_idx = int(filled_partial * 8)

        bar = "█" * filled_full
        if filled_full < bar_width:
            bar += blocks[partial_idx]
            bar += " " * (bar_width - filled_full - 1)

        line = f"{self._phase:8s} ▕{bar}▏ {percent:3d}%"
        print(f"\r{line}", end="", flush=True)

    def update_erase(self, current: float, total: float) -> None:
        """Update progress during erase phase (0-45%)."""
        self._phase = "Erase"
        if total <= 0:
            return
        phase_progress = min(current / total, 1.0)
        percent = int(
            self.ERASE_START + phase_progress * (self.ERASE_END - self.ERASE_START)
        )
        self._render(percent)

    def update_program(self, current: float, total: float) -> None:
        """Update progress during program phase (50-99%)."""
        self._phase = "Program"
        if total <= 0:
            return
        phase_progress = min(current / total, 1.0)
        percent = int(
            self.PROGRAM_START
            + phase_progress * (self.PROGRAM_END - self.PROGRAM_START)
        )
        self._render(percent)

    def update_verify(self, current: float, total: float) -> None:
        """Update progress during verify phase (90-100%)."""
        self._phase = "Verify"
        if total <= 0:
            return
        phase_progress = min(current / total, 1.0)
        percent = int(
            self.VERIFY_START + phase_progress * (self.VERIFY_END - self.VERIFY_START)
        )
        self._render(percent)

    def finish(self) -> None:
        """Complete the progress bar and show summary."""
        # Show "Verify" at 100% briefly so user sees verification passed
        self._phase = "Verify"
        self._last_percent = -1  # Force render
        self._render(100)

        elapsed = time.monotonic() - self._start_time

        if self._noninteractive:
            print(f"\nUploaded in {int(elapsed)}s")
            return

        # Interactive mode: show 100% briefly, then clear and print summary
        time.sleep(0.5)
        print("\r\033[K", end="")
        print(f"Uploaded in {int(elapsed)}s")


# =============================================================================
# Uploader
# =============================================================================


@dataclass
class UploaderConfig:
    """Configuration for uploader."""

    port: Optional[str] = None
    baud_bootloader: int = 115200
    baud_flightstack: list[int] = field(default_factory=lambda: [57600])
    force: bool = False
    force_erase: bool = False
    boot_delay: Optional[int] = None
    use_protocol_splitter: bool = False
    retry_count: int = 3
    windowed: bool = False
    noninteractive: bool = False


class Uploader:
    """Orchestrates firmware upload to PX4 bootloader."""

    def __init__(self, config: UploaderConfig):
        self.config = config
        self.port_detector = PortDetector()

    def upload(self, firmware_paths: list[str]) -> bool:
        """Upload firmware to connected board.

        Args:
            firmware_paths: List of firmware file paths to try

        Returns:
            True if upload successful

        Raises:
            UploadError: If upload fails
        """
        # Load all firmware files
        firmwares = []
        for path in firmware_paths:
            try:
                fw = Firmware(path)
                firmwares.append(fw)
            except FirmwareError as e:
                logger.error(f"Failed to load {path}: {e}")
                if len(firmware_paths) == 1:
                    raise

        if not firmwares:
            raise FirmwareError("No valid firmware files")

        # Determine ports to try
        if self.config.port:
            patterns = self.config.port.split(",")
            ports = self.port_detector.expand_patterns(patterns)
        else:
            ports = self.port_detector.detect_ports()

        if not ports:
            raise ConnectionError("No serial ports found")

        logger.info(f"Trying ports: {ports}")

        # Send MAVLink release command to GCS
        self._send_gcs_release()

        # Try each port
        last_error = None
        for port in ports:
            try:
                return self._upload_to_port(port, firmwares)
            except BoardMismatchError as e:
                logger.warning(f"Board mismatch on {port}: {e}")
                last_error = e
                continue
            except (ConnectionError, TimeoutError) as e:
                logger.debug(f"Connection failed on {port}: {e}")
                last_error = e
                continue
            except UploadError as e:
                logger.error(f"Upload failed on {port}: {e}")
                last_error = e
                raise

        if last_error:
            raise last_error
        raise ConnectionError("No bootloader found on any port")

    def _upload_to_port(self, port: str, firmwares: list[Firmware]) -> bool:
        """Attempt upload on a specific port.

        Args:
            port: Serial port path
            firmwares: List of firmware options

        Returns:
            True if successful

        Raises:
            Various UploadError subclasses on failure
        """
        logger.info(f"Trying port {port}")

        transport = SerialTransport(
            port,
            baudrate=self.config.baud_bootloader,
        )

        try:
            transport.open()
        except ConnectionError:
            return False

        protocol = BootloaderProtocol(
            transport,
            windowed=self.config.windowed,
        )

        try:
            # Try to identify bootloader
            if not self._try_identify(transport, protocol):
                return False

            # Find matching firmware
            firmware = self._select_firmware(firmwares, protocol)

            # Perform upload
            self._do_upload(protocol, firmware)
            return True

        finally:
            transport.close()

    def _try_identify(
        self, transport: SerialTransport, protocol: BootloaderProtocol
    ) -> bool:
        """Try to identify the bootloader, sending reboot if needed.

        Args:
            transport: Serial transport
            protocol: Bootloader protocol handler

        Returns:
            True if bootloader identified
        """
        # First try to identify without reboot
        try:
            protocol.identify()
            print()
            print(
                f"Found board {protocol.board_type},{protocol.board_rev} "
                f"protocol v{protocol.bl_rev} on {transport.port_name}"
            )
            return True
        except (ProtocolError, TimeoutError):
            pass

        # Try rebooting at each baud rate
        for baud in self.config.baud_flightstack:
            print(
                f"Attempting reboot on {transport.port_name} at {baud} baud...",
                file=sys.stderr,
            )

            try:
                transport.set_baudrate(baud)
            except Exception:
                continue

            # Send reboot commands multiple times to increase reliability
            # The board might be busy and miss the first command
            for attempt in range(3):
                try:
                    transport.reset_buffers()

                    # Send MAVLink reboot-to-bootloader commands
                    # Send broadcast (0/0) first, then targeted (1/0)
                    transport.send(protocol.MAVLINK_REBOOT_ID0)
                    transport.send(protocol.MAVLINK_REBOOT_ID1)
                    transport.flush()

                    # Give MAVLink stack time to process
                    time.sleep(0.1)

                    # Send NSH reboot-to-bootloader command
                    transport.send(protocol.NSH_INIT)
                    time.sleep(0.05)
                    transport.send(protocol.NSH_REBOOT_BL)
                    transport.flush()

                    time.sleep(0.2)
                except Exception:
                    pass

            # Wait for reboot - give the board time to process and restart
            time.sleep(0.5)
            transport.close()
            time.sleep(0.5)

            # Reopen at bootloader baud rate and try to identify
            try:
                transport.set_baudrate(self.config.baud_bootloader)
                transport.open()
            except Exception:
                continue

            # Try to identify multiple times - board may take time to enter bootloader
            for identify_attempt in range(5):
                try:
                    protocol.identify()
                    print()
                    print(
                        f"Found board {protocol.board_type},{protocol.board_rev} "
                        f"protocol v{protocol.bl_rev} on {transport.port_name}"
                    )
                    return True
                except (ProtocolError, TimeoutError):
                    # Board may still be rebooting, wait a bit and retry
                    time.sleep(0.3)

        return False

    def _select_firmware(
        self, firmwares: list[Firmware], protocol: BootloaderProtocol
    ) -> Firmware:
        """Select appropriate firmware for the board.

        Args:
            firmwares: Available firmware options
            protocol: Protocol with board info

        Returns:
            Selected firmware

        Raises:
            BoardMismatchError: If no suitable firmware
        """
        for fw in firmwares:
            if fw.board_id == protocol.board_type:
                if len(firmwares) > 1:
                    print(f"Using firmware {fw.path}")
                return fw

        if self.config.force and len(firmwares) == 1:
            print(
                f"WARNING: Firmware board_id={firmwares[0].board_id} "
                f"does not match device board_id={protocol.board_type}"
            )
            print("FORCED UPLOAD, FLASHING ANYWAY!")
            return firmwares[0]

        raise BoardMismatchError(
            f"No suitable firmware for board {protocol.board_type}",
            details=f"available: {[fw.board_id for fw in firmwares]}",
        )

    def _do_upload(self, protocol: BootloaderProtocol, firmware: Firmware) -> None:
        """Perform the actual upload sequence.

        Args:
            protocol: Bootloader protocol handler
            firmware: Firmware to upload
        """
        # Print firmware info
        print(
            f"\nFirmware: board_id={firmware.board_id}, "
            f"revision={firmware.board_revision}"
        )
        print(f"Size: {firmware.image_size} bytes ({firmware.usage_percent:.1f}%)")
        print(f"Bootloader version: {protocol.version}")

        # Check for silicon errata (bootloader v4 on Pixhawk)
        if protocol.bl_rev == 4 and firmware.board_id == 9:
            if firmware.image_size > 1032192 and not self.config.force:
                raise SiliconErrataError(
                    "Board uses bootloader v4 and cannot safely flash >1MB.\n"
                    "Use px4_fmu-v2_default or update the bootloader.\n"
                    "Use --force to override if you know the board is safe."
                )

        # Check flash size
        if protocol.fw_maxsize < firmware.image_size:
            raise FirmwareError(
                f"Firmware too large ({firmware.image_size} bytes) "
                f"for flash ({protocol.fw_maxsize} bytes)"
            )

        # Check for undersized config
        if (
            protocol.bl_rev >= 5
            and protocol.fw_maxsize > firmware.image_maxsize
            and not self.config.force
        ):
            print(
                f"WARNING: Board flash ({protocol.fw_maxsize} bytes) "
                f"larger than firmware config ({firmware.image_maxsize} bytes)"
            )

        # Print OTP/SN info
        self._print_board_info(protocol)

        # Create unified progress bar
        print()
        progress = UploadProgressBar(noninteractive=self.config.noninteractive)

        # Erase
        protocol.erase(
            force_full=self.config.force_erase,
            progress_callback=progress.update_erase,
        )

        # Program
        protocol.program(firmware, progress_callback=progress.update_program)

        # Verify
        protocol.verify(firmware, progress_callback=progress.update_verify)

        # Set boot delay if requested
        if self.config.boot_delay is not None:
            protocol.set_boot_delay(self.config.boot_delay)

        # Reboot and show summary
        protocol.reboot()
        progress.finish()

    def _print_board_info(self, protocol: BootloaderProtocol) -> None:
        """Print board OTP and chip info."""
        if protocol.sn:
            print(f"Serial: {protocol.sn.hex()}")

        if protocol.chip_id:
            print(f"Chip: 0x{protocol.chip_id:08X}")

        if protocol.chip_family:
            print(f"Family: {protocol.chip_family}")

        if protocol.chip_revision:
            print(f"Revision: {protocol.chip_revision}")

        print(f"Flash: {protocol.fw_maxsize} bytes")
        print(f"Windowed mode: {'yes' if protocol._windowed_mode else 'no'}")

    def _send_gcs_release(self) -> None:
        """Send UDP message to release serial port from GCS."""
        try:
            heartbeat = bytes.fromhex("fe097001010000000100020c5103033c8a")
            command = bytes.fromhex(
                "fe210101014c0000000000000000000000000000000000"
                "00000000000000803f00000000f6000000008459"
            )

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(heartbeat, ("127.0.0.1", 14550))
            sock.sendto(command, ("127.0.0.1", 14550))
            sock.close()
        except Exception:
            pass  # Non-critical


# =============================================================================
# Main Entry Point
# =============================================================================


def main() -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="PX4 Firmware Uploader v2",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s firmware.px4
  %(prog)s --port /dev/ttyACM0 firmware.px4
  %(prog)s --port /dev/serial/by-id/*PX4* firmware.px4
  %(prog)s -v --force firmware.px4
        """,
    )

    parser.add_argument("firmware", nargs="+", help="Firmware file(s) to upload")
    parser.add_argument(
        "--port",
        "-p",
        help="Serial port(s) to use (comma-separated, supports wildcards). "
        "If not specified, auto-detects PX4 devices.",
    )
    parser.add_argument(
        "--baud-bootloader",
        type=int,
        default=115200,
        help="Bootloader baud rate (default: 115200)",
    )
    parser.add_argument(
        "--baud-flightstack",
        default="57600",
        help="Flight stack baud rate(s) for reboot (comma-separated, default: 57600)",
    )
    parser.add_argument(
        "--force",
        "-f",
        action="store_true",
        help="Force upload even if board ID doesn't match",
    )
    parser.add_argument(
        "--force-erase",
        action="store_true",
        help="Force full chip erase (v6+ bootloader)",
    )
    parser.add_argument(
        "--boot-delay", type=int, help="Boot delay in milliseconds to store in flash"
    )
    parser.add_argument(
        "--use-protocol-splitter-format",
        action="store_true",
        help="Use protocol splitter framing for reboot commands",
    )
    parser.add_argument(
        "--windowed",
        action="store_true",
        help="Use windowed mode for faster uploads on real serial ports (FTDI)",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Enable verbose output"
    )
    parser.add_argument(
        "--debug",
        "-d",
        action="store_true",
        help="Enable debug output (includes protocol traces)",
    )
    parser.add_argument(
        "--noninteractive",
        action="store_true",
        help="Non-interactive mode: print progress every 5%% for tools to parse",
    )

    args = parser.parse_args()

    # Setup logging
    setup_logging(verbose=args.verbose, debug=args.debug)

    # Warn about ModemManager on Linux
    if sys.platform.startswith("linux") and os.path.exists("/usr/sbin/ModemManager"):
        print("=" * 80)
        print("WARNING: ModemManager detected. It may interfere with PX4 devices.")
        print("Consider: sudo systemctl disable ModemManager")
        print("=" * 80)

    # Parse baud rates
    baud_flightstack = [int(x) for x in args.baud_flightstack.split(",")]

    # Create config
    config = UploaderConfig(
        port=args.port,
        baud_bootloader=args.baud_bootloader,
        baud_flightstack=baud_flightstack,
        force=args.force,
        force_erase=args.force_erase,
        boot_delay=args.boot_delay,
        use_protocol_splitter=args.use_protocol_splitter_format,
        windowed=args.windowed,
        noninteractive=args.noninteractive,
    )

    if args.use_protocol_splitter_format:
        print("Using protocol splitter format for reboot commands")

    print("Waiting for bootloader...")

    uploader = Uploader(config)

    try:
        # Keep trying until we find a board or user interrupts
        while True:
            try:
                if uploader.upload(args.firmware):
                    return 0
            except BoardMismatchError:
                # No suitable firmware for this board
                return 2
            except (ConnectionError, TimeoutError):
                # No device found yet, keep trying
                time.sleep(0.05)
            except UploadError as e:
                print(f"\nError: {e}", file=sys.stderr)
                return 1

    except KeyboardInterrupt:
        print("\nUpload aborted by user.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
