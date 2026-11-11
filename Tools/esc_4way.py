#!/usr/bin/env python3
############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

"""ESC configuration over a flight controller's MSP + 4way interface.

Talks to the PX4 esc_4way server (USB port with SYS_USB_AUTO=1) or any other
MSP + 4way flight controller.

    Tools/esc_4way.py [--setup] [--no-exit] [-d DEVICE] [--target N] COMMAND

    check [--no-esc] [--variant BTFL] [--toggle-direction]
                          protocol self-check, every reply checked byte-exact
    info                  ESC signature, MCU, bootloader pin and flash layout
    read ADDR LEN [-o F]  raw flash read (byte offset from the flash start)
    write ADDR FILE       raw flash write
    dump FILE             whole flash (the bootloader region reads as 0xFF)
    flash FILE [--force]  AM32 firmware (Intel hex or bin), verified before the boot byte is set
    settings              AM32 settings block, raw and decoded
    direction set 0|1     AM32 motor direction (or: direction toggle)

A port that sends MAVLink heartbeats is first freed for the ESC configuration
server: the tool sets SYS_USB_AUTO=1, which stays set, and reboots the board
(needs pymavlink); --setup does that without listening for heartbeats first.
--no-exit leaves the session open: the port then
keeps DTR raised after it is closed, also for other programs, until a run without
--no-exit or the board's next reboot. Exit code 0 on success.

"""

import argparse
import os
import re
import select
import struct
import sys
import termios
import time
import tty

TIMEOUT = 2.0  # seconds per exchange

# MSP v1 command ids
MSP_API_VERSION = 1
MSP_FC_VARIANT = 2
MSP_FC_VERSION = 3
MSP_BOARD_INFO = 4
MSP_BUILD_INFO = 5
MSP_STATUS = 101
MSP_BATTERY_STATE = 130
MSP_MOTOR_CONFIG = 131
MSP_UID = 160
MSP_SET_PASSTHROUGH = 245

# 4way interface
FOURWAY_REQ = 0x2F
FOURWAY_REPLY = 0x2E
TEST_ALIVE = 0x30
GET_PROTOCOL_VERSION = 0x31
GET_NAME = 0x32
GET_IF_VERSION = 0x33
EXIT = 0x34
DEVICE_RESET = 0x35
DEVICE_INIT_FLASH = 0x37
DEVICE_READ = 0x3A
DEVICE_WRITE = 0x3B
ACK_OK = 0x00
ACK_I_INVALID_CMD = 0x02
ACK_I_INVALID_CRC = 0x03
ACK_I_INVALID_PARAM = 0x09
ACK_D_GENERAL_ERROR = 0x0F

AM32_SETTINGS_ADDR = 0x7C00
AM32_SETTINGS_LEN = 184
AM32_MOTOR_DIRECTION = 17  # settings offset of "motor direction reversed" (eeprom.ts MOTOR_DIRECTION 0x11)
AM32_MOTOR_POLES = 27      # eeprom.ts MOTOR_POLES 0x1B
# settings block wire address by bootloader signature; G431 (128 KiB) addresses in 4-byte units
AM32_SETTINGS_ADDR_BY_SIG = {0x1F06: 0x7C00, 0x3506: 0xF800, 0x1506: 0xE000, 0x2B06: 0x1F800 >> 2}
# AM32 flash layout by signature (am32 configurator mcu.ts): name, flash size, flash base address,
# application start, settings start; byte offsets from the flash base. A v3 bootloader reports its own.
AM32_MCUS = {
    0x1F06: ('STM32F051', 0x10000, 0x08000000, 0x1000, 0x7C00),
    0x3506: ('ARM64K', 0x10000, 0x08000000, 0x1000, 0xF800),
    0x1506: ('MCXA153', 0x10000, 0, 0x4000, 0xE000),
    0x2B06: ('STM32G431', 0x20000, 0x08000000, 0x4000, 0x1F800),
}
DEVINFO_ADDR = 0x23  # v3 bootloader devinfo block
DEVINFO_MAGIC = bytes.fromhex('dae32559d963b84e')
RX_FAILSAFE = 1 << 2  # arming-disable flag bit in MSP_STATUS


class CheckFailed(Exception):
    pass


def check(cond, msg):
    if not cond:
        raise CheckFailed(msg)


def crc16_xmodem(data, crc=0):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def xor8(data):
    x = 0
    for b in data:
        x ^= b
    return x


def read_n(fd, n, deadline, what='reply'):
    """Read exactly n bytes; deadline is a time.monotonic() value or None (block)."""
    buf = b''
    while len(buf) < n:
        wait = None if deadline is None else max(0.0, deadline - time.monotonic())
        if not select.select([fd], [], [], wait)[0]:
            raise CheckFailed(f'{what}: timeout after {len(buf)} of {n} bytes')
        chunk = os.read(fd, n - len(buf))
        check(chunk, f'{what}: link closed')
        buf += chunk
    return buf


def msp_encode(cmd, payload=b'', direction=b'<'):
    body = bytes([len(payload), cmd]) + bytes(payload)
    return b'$M' + direction + body + bytes([xor8(body)])


def msp_read(fd, deadline, what='MSP'):
    """Read an MSP v1 frame after its 3-byte header: (cmd, payload, checksum_ok)."""
    size, cmd = read_n(fd, 2, deadline, what)
    rest = read_n(fd, size + 1, deadline, what)
    return cmd, rest[:-1], xor8(bytes([size, cmd]) + rest[:-1]) == rest[-1]


def fourway_encode(start, cmd, addr, params, ack=None):
    """4way frame: start cmd addrH addrL n params [ack] crc16 (n=0 means 256)."""
    params = bytes(params) or b'\0'
    check(0 <= addr <= 0xFFFF, f'wire address 0x{addr:X} out of range')
    frame = bytes([start, cmd, addr >> 8, addr & 0xFF, len(params) & 0xFF]) + params
    if ack is not None:
        frame += bytes([ack])
    return frame + crc16_xmodem(frame).to_bytes(2, 'big')


def fourway_read(fd, deadline, start, what='4way'):
    """Read a 4way frame after its start byte: (cmd, addr, params, ack, crc_ok)."""
    head = read_n(fd, 4, deadline, what)
    n = head[3] or 256
    rest = read_n(fd, n + (start == FOURWAY_REPLY) + 2, deadline, what)
    frame = bytes([start]) + head + rest
    ack = rest[n] if start == FOURWAY_REPLY else None
    crc_ok = crc16_xmodem(frame[:-2]) == int.from_bytes(frame[-2:], 'big')
    return head[0], (head[1] << 8) | head[2], rest[:n], ack, crc_ok


def open_serial(path, listen=1.2, keep_dtr=False):
    """Raw 8N1 at 115200 (no echo, no line discipline): (fd, bytes received while listening).

    Listening drops a banner or late replies of an earlier run, and covers one 1 Hz heartbeat.
    Closing the port drops DTR, which ends a PX4 session; keep_dtr leaves it raised (--no-exit).
    """
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    attr = termios.tcgetattr(fd)
    tty.cfmakeraw(attr)
    attr[2] |= termios.CLOCAL | termios.CREAD
    attr[2] = attr[2] & ~termios.HUPCL if keep_dtr else attr[2] | termios.HUPCL
    attr[4] = attr[5] = termios.B115200
    attr[6][termios.VMIN], attr[6][termios.VTIME] = 1, 0  # select() does the timing
    termios.tcsetattr(fd, termios.TCSANOW, attr)
    seen, deadline = b'', time.monotonic() + listen
    while select.select([fd], [], [], max(0.0, deadline - time.monotonic()))[0]:
        chunk = os.read(fd, 4096)
        if not chunk:
            break
        seen += chunk
    return fd, seen


def mavlink_heartbeat_seen(data):
    """A MAVLink v2 or v1 HEARTBEAT header (payload length 9, message id 0) in raw bytes."""
    return any(data[i + 7:i + 10] == b'\0\0\0' if data[i] == 0xFD else data[i + 5:i + 6] == b'\0'
               for i in (m.start() for m in re.finditer(rb'[\xfd\xfe]\x09', data)))


def msp(fd, cmd, payload=b''):
    what = f'MSP {cmd}'
    os.write(fd, msp_encode(cmd, payload))
    deadline = time.monotonic() + TIMEOUT
    hdr = read_n(fd, 3, deadline, what)
    check(hdr == b'$M>', f'{what}: bad header {hdr!r}')
    rcmd, data, ok = msp_read(fd, deadline, what)
    check(ok, f'{what}: bad checksum')
    check(rcmd == cmd, f'{what}: reply for command {rcmd}')
    return data


def check_fourway_reply(fd, deadline, cmd, addr, what):
    rcmd, raddr, params, ack, ok = fourway_read(fd, deadline, FOURWAY_REPLY, what)
    check(ok, f'{what}: CRC mismatch')
    check(rcmd == cmd, f'{what}: reply cmd 0x{rcmd:02X}')
    check(raddr == addr, f'{what}: reply address 0x{raddr:04X}, expected 0x{addr:04X}')
    check(ack == ACK_OK, f'{what}: ack 0x{ack:02X}')
    return params


def fourway(fd, cmd, addr=0, params=b'\0', reply_addr=None):
    what = f'4way 0x{cmd:02X}'
    os.write(fd, fourway_encode(FOURWAY_REQ, cmd, addr, params))
    deadline = time.monotonic() + TIMEOUT
    start = read_n(fd, 1, deadline, what)[0]
    check(start == FOURWAY_REPLY, f'{what}: bad start byte 0x{start:02X}')
    return check_fourway_reply(fd, deadline, cmd, addr if reply_addr is None else reply_addr, what)


def stray_exit(fd):
    """InterfaceExit before MSP_SET_PASSTHROUGH: a 4way ack or silence, never a dropped link."""
    what = 'stray InterfaceExit'
    os.write(fd, fourway_encode(FOURWAY_REQ, EXIT, 0, b'\0'))
    deadline = time.monotonic() + TIMEOUT
    if not select.select([fd], [], [], TIMEOUT)[0]:
        return 'ignored'
    start = read_n(fd, 1, deadline, what)[0]
    if start == ord('$'):
        read_n(fd, 2, deadline, what)
        msp_read(fd, deadline, what)
        return 'MSP-level reply'
    check(start == FOURWAY_REPLY, f'{what}: bad start byte 0x{start:02X}')
    check_fourway_reply(fd, deadline, EXIT, 0, what)
    return 'acked'


def ms(t):
    return round((time.monotonic() - t) * 1000)


def init_flash(fd, target, passthrough_t):
    """DeviceInitFlash the way the configurator does it: [sigLo, sigHi, pinCode, interfaceMode]."""
    # the ESC app enters its bootloader 2 s after the signal stops; the configurator
    # waits 2 s after passthrough and then allows two InitFlash attempts
    time.sleep(max(0.0, passthrough_t + 2.0 - time.monotonic()))
    for attempt in (1, 2):
        t = time.monotonic()
        try:
            data = fourway(fd, DEVICE_INIT_FLASH, 0, bytes([target]), reply_addr=0)
            break
        except CheckFailed as e:
            if attempt == 2:
                raise
            print(f'      DeviceInitFlash target {target} attempt {attempt}: {e}')
        finally:
            print(f'      DeviceInitFlash target {target} attempt {attempt}: {ms(t)} ms')
    check(len(data) == 4, f'DeviceInitFlash: {len(data)} params, expected 4')
    return data


def run(fd, no_esc, variant, target, toggle_direction=False):
    def step(name, value=''):
        print(f'  ok  {name} {value}'.rstrip())

    data = msp(fd, MSP_API_VERSION)
    check(len(data) == 3, f'API_VERSION: length {len(data)}')
    step('MSP_API_VERSION', list(data))
    step('stray InterfaceExit', stray_exit(fd))

    data = msp(fd, MSP_FC_VARIANT)
    check(len(data) == 4 and data == variant, f'FC_VARIANT: {data!r}, expected {variant!r}')
    step('MSP_FC_VARIANT', repr(data.decode('ascii', 'replace')))

    data = msp(fd, MSP_BATTERY_STATE)
    check(len(data) >= 8, f'BATTERY_STATE: length {len(data)}, expected at least 8')
    step('MSP_BATTERY_STATE', f'{len(data)} bytes')

    data = msp(fd, MSP_MOTOR_CONFIG)
    check(len(data) >= 7, f'MOTOR_CONFIG: length {len(data)}, expected at least 7')
    check(data[6] >= 1, 'MOTOR_CONFIG: motor count 0')
    step('MSP_MOTOR_CONFIG', f'{data[6]} motors')

    data = msp(fd, MSP_STATUS)
    if variant == b'BTFL':  # Betaflight: longer layout, arming flags at another offset
        check(len(data) >= 22, f'STATUS: length {len(data)}, expected at least 22')
    else:
        # esc-configurator reads the reboot-required byte at offset 21
        check(len(data) == 22, f'STATUS: length {len(data)}, expected 22')
        # layout: cycle u16, i2c u16, sensors u16, modes u32, profile u8, load u16,
        # gyro u16, extra mode bytes u8, arming flag count u8, arming disable flags u32, reboot required u8
        check(int.from_bytes(data[17:21], 'little') & RX_FAILSAFE, 'STATUS: RX_FAILSAFE arming-disable bit not set')
    step('MSP_STATUS', f'{len(data)} bytes')

    data = msp(fd, MSP_UID)
    check(len(data) == 12, f'UID: length {len(data)}, expected 12')
    step('MSP_UID', data.hex())

    btfl = variant == b'BTFL'  # Betaflight's version and info replies are longer
    for cmd, name, size in ((MSP_FC_VERSION, 'FC_VERSION', 3), (MSP_BOARD_INFO, 'BOARD_INFO', 7),
                            (MSP_BUILD_INFO, 'BUILD_INFO', 19)):
        data = msp(fd, cmd)
        if btfl:
            size = {MSP_BOARD_INFO: 4}.get(cmd, size)
            check(len(data) >= size, f'{name}: length {len(data)}, expected at least {size}')
        else:
            check(len(data) == size, f'{name}: length {len(data)}, expected {size}')
        step(f'MSP_{name}', data.hex())

    t = time.monotonic()
    data = msp(fd, MSP_SET_PASSTHROUGH)
    check(len(data) == 1 and data[0] >= 1, f'SET_PASSTHROUGH: reply {data.hex()}, expected 1 byte ESC count')
    step('MSP_SET_PASSTHROUGH', f'{data[0]} ESCs, {ms(t)} ms')
    passthrough_t = time.monotonic()

    fourway(fd, TEST_ALIVE)
    step('InterfaceTestAlive')
    data = fourway(fd, GET_PROTOCOL_VERSION)
    step('ProtocolGetVersion', data[0])
    data = fourway(fd, GET_NAME)
    step('InterfaceGetName', repr(data.decode('ascii', 'replace')))

    if not no_esc:
        data = init_flash(fd, target, passthrough_t)
        sig = data[0] | data[1] << 8
        step('DeviceInitFlash', f'target {target} signature 0x{sig:04X} pin 0x{data[2]:02X} mode {data[3]}')
        addr = AM32_SETTINGS_ADDR_BY_SIG.get(sig, AM32_SETTINGS_ADDR)
        t = time.monotonic()
        data = fourway(fd, DEVICE_READ, addr, bytes([AM32_SETTINGS_LEN]))
        check(len(data) == AM32_SETTINGS_LEN, f'DeviceRead: {len(data)} bytes, expected {AM32_SETTINGS_LEN}')
        step('DeviceRead', f'{len(data)} bytes at wire address 0x{addr:04X}, {ms(t)} ms: {data[:16].hex()}...')
        # write the block back unchanged and read it again; the bootloader sets byte 2 itself
        t = time.monotonic()
        fourway(fd, DEVICE_WRITE, addr, data)
        step('DeviceWrite', f'{len(data)} bytes at wire address 0x{addr:04X}, {ms(t)} ms')
        again = fourway(fd, DEVICE_READ, addr, bytes([AM32_SETTINGS_LEN]))
        check(again[:2] + again[3:] == data[:2] + data[3:], f'write-back mismatch: {again.hex()} != {data.hex()}')
        step('DeviceRead after write', 'matches')

        if toggle_direction:
            # flip "motor direction reversed", verify, then restore the original block
            flipped = bytearray(data)
            flipped[AM32_MOTOR_DIRECTION] ^= 1
            for block, name in ((bytes(flipped), 'direction toggled'), (data, 'original restored')):
                fourway(fd, DEVICE_WRITE, addr, block)
                again = fourway(fd, DEVICE_READ, addr, bytes([AM32_SETTINGS_LEN]))
                check(again[:2] + again[3:] == block[:2] + block[3:], f'{name}: read back {again.hex()} != {block.hex()}')
                step(f'DeviceWrite {name}', f'direction {again[AM32_MOTOR_DIRECTION]}, bytes 0..24: {again[:25].hex(" ")}')

    fourway(fd, EXIT)
    step('InterfaceExit')


# MAVLink entry (PX4 only)

def _wait_port(path, timeout=30):
    """Wait for a serial device to drop and come back (a reboot); returns its current path.

    Follows the /dev/serial/by-id name when there is one: a bootloader that enumerates in between
    under another name is not mistaken for the application.
    """
    by_id = '/dev/serial/by-id'
    names = sorted(os.listdir(by_id)) if os.path.isdir(by_id) else []
    port = next((os.path.join(by_id, n) for n in names
                 if os.path.realpath(os.path.join(by_id, n)) == os.path.realpath(path)), path)
    deadline = time.monotonic() + 10
    while os.path.exists(port) and time.monotonic() < deadline:
        time.sleep(0.1)
    # back and present for 3 s: a bootloader may enumerate under the same name first
    deadline, since = time.monotonic() + timeout, None
    while since is None or time.monotonic() - since < 3:
        check(time.monotonic() < deadline, f'{port} did not come back within {timeout} s')
        since = (since or time.monotonic()) if os.path.exists(port) else None
        time.sleep(0.1)
    return os.path.realpath(port)


def _wait_gone(path, timeout=3.0):
    """Wait for the serial device of a board that reboots after its session to drop (or timeout)."""
    deadline = time.monotonic() + timeout
    while os.path.exists(path) and time.monotonic() < deadline:
        time.sleep(0.05)


def _heartbeat(mav, timeout):
    """First autopilot heartbeat within timeout, or None."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=deadline - time.monotonic())
        if hb and hb.get_srcSystem() != 0 and hb.get_srcComponent() == 1:
            return hb
    return None


def _mavutil():
    try:
        from pymavlink import mavutil
    except ImportError:
        raise CheckFailed('switching a MAVLink port needs pymavlink (pip install pymavlink)')
    return mavutil


def _after_reboot(path, timeout, baud):
    """Wait for the port to return after a reboot: (path, first heartbeat within timeout or None).

    A port that drops again while listening (another enumeration) is waited for once more.
    """
    mavutil = _mavutil()
    for attempt in (1, 2, 3):
        path = _wait_port(path)
        try:
            mav = mavutil.mavlink_connection(path, baud=baud)
            try:
                return path, _heartbeat(mav, timeout)
            finally:
                mav.close()
        except OSError as e:
            if attempt == 3:
                raise CheckFailed(f'{path}: {e}')


def mavlink_esc_mode(path, baud=115200):
    """Free the MAVLink USB port of a PX4 board for the ESC configuration server.

    SYS_USB_AUTO=1 (the port then starts whatever talks to it first, and stays that way), wait for
    its echo and the parameter autosave, reboot, wait for the port to return and confirm it is
    quiet (no heartbeat for 3 s). Returns the port path.
    """
    mavutil = _mavutil()
    mav = mavutil.mavlink_connection(path, baud=baud)
    hb = _heartbeat(mav, 3)
    if hb is None:
        mav.close()
        print(f'{path}: no heartbeat, assuming the port already waits for its first program')
        return path

    mav.target_system, mav.target_component = hb.get_srcSystem(), hb.get_srcComponent()
    print('Setting SYS_USB_AUTO=1 (stays set)')
    mav.mav.param_set_send(mav.target_system, mav.target_component, b'SYS_USB_AUTO',
                           struct.unpack('<f', struct.pack('<i', 1))[0], mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        msg = mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=deadline - time.monotonic())
        if msg and msg.param_id == 'SYS_USB_AUTO':
            check(struct.unpack('<i', struct.pack('<f', msg.param_value))[0] == 1,
                  f'SYS_USB_AUTO echo {msg.param_value}')
            break
    else:
        mav.close()
        raise CheckFailed('no PARAM_VALUE echo for SYS_USB_AUTO')
    time.sleep(3)  # parameter autosave
    print('Rebooting')
    for _ in range(5):  # commander may reject it briefly, e.g. right after boot
        mav.mav.command_long_send(mav.target_system, mav.target_component,
                                  mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
        ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                break
            print(f'reboot rejected (result {ack.result}), retrying')
            time.sleep(1)
    else:
        mav.close()
        raise CheckFailed('reboot not accepted')
    mav.close()
    path, hb = _after_reboot(path, 3, baud)
    check(hb is None, f'{path}: still sending MAVLink heartbeats')
    print(f'{path}: quiet for 3 s, waiting for its first program')
    return path


# Device commands through the 4way server

class Esc:
    """One ESC behind MSP_SET_PASSTHROUGH + DeviceInitFlash, with its AM32 flash layout."""

    def __init__(self, fd, target):
        self.fd = fd
        msp(fd, MSP_API_VERSION)
        count = msp(fd, MSP_SET_PASSTHROUGH)[0]
        print(f'{count} ESCs, waiting for ESC {target} bootloader')
        data = init_flash(fd, target, time.monotonic())
        self.signature, self.pin, self.mode = data[0] | data[1] << 8, data[2], data[3]
        self.name, self.flash_size, self.base, self.app, self.settings = AM32_MCUS.get(
            self.signature, ('unknown', 0, 0, 0, 0))
        self.shift, self.v3 = 0, False
        try:
            d = fourway(fd, DEVICE_READ, DEVINFO_ADDR, bytes([64])) if self.mode == 4 else b''
        except CheckFailed:
            d = b''
            while select.select([fd], [], [], 0.3)[0] and os.read(fd, 1024):
                pass  # drop the rest of a late or bad reply
        # magic[8] deviceInfo[9] length shift firmware_start filename_start eeprom_start (u16 LE)
        if d[:8] == DEVINFO_MAGIC and 27 <= d[17] <= 64 and d[18] <= 8:
            self.shift, self.v3 = d[18], True
            self.app, self.name_addr, self.settings = (int.from_bytes(d[o:o + 2], 'little') << self.shift
                                                       for o in (19, 21, 23))
        else:
            self.name_addr = self.settings - 32  # firmware name, 32 bytes before the settings
        self.layout_ok = self.flash_size and self.flash_size >> self.shift <= 0x10000

    def cmd(self, cmd, addr, params):
        """A 4way device command, sent again after a bad or lost reply (as the configurators do)."""
        for attempt in (1, 2, 3):
            try:
                return fourway(self.fd, cmd, addr, params)
            except CheckFailed as e:
                if attempt == 3:
                    raise
                print(f'  retrying after a bad reply: {e}')
                while select.select([self.fd], [], [], 0.3)[0] and os.read(self.fd, 1024):
                    pass  # drop the rest of it

    def describe(self):
        mode = {1: 'SiLabs', 4: 'ARM'}.get(self.mode, '?')
        return (f'signature 0x{self.signature:04X}, MCU {self.name}, pin P{chr(65 + (self.pin >> 4))}{self.pin & 15}, '
                f'interface mode {self.mode} ({mode}), flash {self.flash_size // 1024} KiB, '
                f'application 0x{self.app:05X}, settings 0x{self.settings:05X}' + (' (v3 bootloader)' if self.v3 else ''))

    def wire(self, off):
        check(off % (1 << self.shift) == 0, f'0x{off:X} not aligned to {1 << self.shift} bytes')
        return off >> self.shift

    def read(self, off, n, quiet=False):
        out = b''
        while len(out) < n:
            step = min(128, n - len(out))
            out += self.cmd(DEVICE_READ, self.wire(off + len(out)), bytes([step]))
            if not quiet and (len(out) % 4096 == 0 or len(out) == n):
                print(f'  read 0x{len(out):05X} of 0x{n:05X}', flush=True)
        return out

    def write(self, off, data, quiet=False):
        for i in range(0, len(data), 256):
            chunk = data[i:i + 256]
            chunk += b'\xff' * (-len(chunk) % 8)  # STM32G4 flash programs 8-byte words
            self.cmd(DEVICE_WRITE, self.wire(off + i), chunk)
            if not quiet and ((i + 256) % 4096 == 0 or i + 256 >= len(data)):
                print(f'  wrote 0x{min(i + 256, len(data)):05X} of 0x{len(data):05X}', flush=True)

    def write_settings(self, block):
        """Write the settings block and verify it; the bootloader sets byte 2 itself."""
        self.write(self.settings, block, quiet=True)
        again = self.read(self.settings, AM32_SETTINGS_LEN, quiet=True)
        check(again[:2] + again[3:] == block[:2] + block[3:], f'settings read back {again.hex()} != {block.hex()}')
        return again


def load_image(path, esc):
    """Application bytes from an Intel hex or bin file, as written from the application start."""
    raw = open(path, 'rb').read()
    if path.lower().endswith('.hex'):
        data, upper, eof = {}, 0, False
        for line in raw.decode('ascii', 'replace').split():
            try:
                rec = bytes.fromhex(line.lstrip(':'))
            except ValueError:
                raise CheckFailed(f'{path}: not an Intel hex line: {line!r}')
            check(len(rec) >= 5 and len(rec) == rec[0] + 5 and sum(rec) & 0xFF == 0, f'{path}: bad hex record {line}')
            n, addr, kind = rec[0], rec[1] << 8 | rec[2], rec[3]
            if kind == 0:
                check(esc.base <= upper + addr and upper + addr + n <= esc.base + esc.flash_size,
                      f'{path}: data at 0x{upper + addr:X} outside the ESC flash')
                for i, b in enumerate(rec[4:4 + n]):
                    data[upper + addr + i] = b
            elif kind in (2, 4):
                check(n == 2, f'{path}: bad hex record {line}')
                upper = int.from_bytes(rec[4:6], 'big') << (4 if kind == 2 else 16)
            elif kind == 1:
                eof = True
                break
        check(eof, f'{path}: no end-of-file record, the file is truncated')
        check(data, f'{path}: no data')
        image = bytearray(b'\xff' * (max(data) + 1 - esc.base))
        for a, b in data.items():
            image[a - esc.base] = b
        raw = bytes(image)  # from the flash base, like a dump
    elif len(raw) != esc.flash_size:
        raw = b'\xff' * esc.app + raw  # application image
    image = raw[esc.app:]
    check(len(raw) == esc.flash_size or len(raw) <= esc.settings,
          f'{path}: image reaches 0x{len(raw):X}, past the settings at 0x{esc.settings:X}')
    image = image[:esc.settings - esc.app]
    check(image.strip(b'\xff'), f'{path}: no application data at 0x{esc.app:X}')
    return image


def firmware_name(block):
    """AM32 firmware name from the 32 bytes before the settings ('' if none)."""
    return bytes(b for b in block.split(b'\0')[0] if 32 <= b < 127).decode()


def run_command(fd, args):
    esc = Esc(fd, args.target)
    print(esc.describe())
    if args.command != 'info':
        check(esc.layout_ok, 'unknown flash layout')
    if args.command in ('read', 'write'):
        size = args.len if args.command == 'read' else os.path.getsize(args.file)
        check(args.addr + size <= esc.flash_size, f'0x{size:X} bytes at 0x{args.addr:X} pass the flash end 0x{esc.flash_size:X}')

    if args.command == 'read':
        data = esc.read(args.addr, args.len)
        if args.output:
            open(args.output, 'wb').write(data)
            print(f'saved {args.output}')
        else:
            for i in range(0, len(data), 16):
                print(f'{args.addr + i:05X}: {data[i:i + 16].hex(" ")}')
    elif args.command == 'write':
        check(args.addr >= esc.app, f'refusing to write the bootloader (below 0x{esc.app:X})')
        esc.write(args.addr, open(args.file, 'rb').read())
    elif args.command == 'dump':
        # the bootloader does not read itself out, so the dump starts at the application
        data = b'\xff' * esc.app + esc.read(esc.app, esc.flash_size - esc.app)
        open(args.file, 'wb').write(data)
        print(f'saved {args.file} (bootloader region 0x00000-0x{esc.app - 1:05X} left as 0xFF)')
    elif args.command == 'flash':
        image = load_image(args.file, esc)
        tag = esc.name_addr - esc.app
        name = firmware_name(image[tag:tag + 32]) if 0 <= tag <= len(image) - 32 else ''
        current = firmware_name(esc.read(esc.name_addr, 32, quiet=True))
        check(args.force or name, f'{args.file} carries no firmware name, it may be truncated (--force to flash anyway)')
        check(args.force or not current or name == current,
              f'{args.file} is for {name!r}, the ESC runs {current!r} firmware (--force to flash anyway)')
        settings = bytearray(esc.read(esc.settings, AM32_SETTINGS_LEN, quiet=True))
        settings[0] = 0  # boot byte: cleared while the application is incomplete
        esc.write(esc.settings, settings, quiet=True)
        esc.write(esc.app, image)
        check(esc.read(esc.app, len(image), quiet=True) == image, 'read-back differs, the ESC stays in its bootloader')
        settings[0] = 1
        esc.write(esc.settings, settings, quiet=True)
        print(f'flashed and verified {len(image)} bytes at 0x{esc.app:05X}')
    elif args.command == 'settings':
        name = firmware_name(esc.read(esc.name_addr, 32, quiet=True))
        data = esc.read(esc.settings, AM32_SETTINGS_LEN, quiet=True)
        for i in range(0, len(data), 16):
            print(f'  {esc.settings + i:05X}: {data[i:i + 16].hex(" ")}')
        print(f'layout revision {data[1]}, firmware {data[3]}.{data[4]}, name {name!r}, '
              f'direction {data[AM32_MOTOR_DIRECTION]}, poles {data[AM32_MOTOR_POLES]}')
    elif args.command == 'direction':
        block = bytearray(esc.read(esc.settings, AM32_SETTINGS_LEN, quiet=True))
        old = block[AM32_MOTOR_DIRECTION]
        block[AM32_MOTOR_DIRECTION] = old ^ 1 if args.action == 'toggle' else args.value
        again = esc.write_settings(bytes(block))
        print(f'direction {old} -> {again[AM32_MOTOR_DIRECTION]}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    # the common options are accepted before and after the command
    common = argparse.ArgumentParser(add_help=False)
    for p, default in ((parser, None), (common, argparse.SUPPRESS)):
        p.add_argument('-d', '--device', default=default or '/dev/ttyACM0', help='serial device (default /dev/ttyACM0)')
        p.add_argument('--target', type=int, default=default or 0, help='ESC channel (default 0)')
        p.add_argument('--setup', action='store_true', default=default or False,
                       help='set SYS_USB_AUTO=1 and reboot over MAVLink without listening for heartbeats first')
        p.add_argument('--no-exit', action='store_true', default=default or False,
                       help='leave the session open (no InterfaceExit, no reboot)')
    sub = parser.add_subparsers(dest='command', required=True)
    c = sub.add_parser('check', parents=[common], help='protocol self-check')
    c.add_argument('dev', nargs='?', help='serial device path (same as -d)')
    c.add_argument('--variant', default='PX4 ', help='expected 4-byte MSP_FC_VARIANT (default %(default)r)')
    c.add_argument('--toggle-direction', action='store_true',
                   help='flip the AM32 motor direction setting, verify, then restore it')
    c.add_argument('--no-esc', action='store_true', help='skip DeviceInitFlash and DeviceRead')
    sub.add_parser('info', parents=[common], help='ESC signature, MCU and flash layout')
    c = sub.add_parser('read', parents=[common], help='raw flash read')
    c.add_argument('addr', type=lambda v: int(v, 0), help='byte offset from the flash start')
    c.add_argument('len', type=lambda v: int(v, 0))
    c.add_argument('-o', '--output', help='output file (hex on the console if omitted)')
    c = sub.add_parser('write', parents=[common], help='raw flash write')
    c.add_argument('addr', type=lambda v: int(v, 0), help='byte offset from the flash start')
    c.add_argument('file')
    sub.add_parser('dump', parents=[common], help='whole flash to a file').add_argument('file')
    c = sub.add_parser('flash', parents=[common], help='AM32 firmware, Intel hex or bin')
    c.add_argument('file')
    c.add_argument('--force', action='store_true', help='flash an image whose firmware name differs from the ESC\'s')
    sub.add_parser('settings', parents=[common], help='AM32 settings, raw and decoded')
    c = sub.add_parser('direction', parents=[common], help='AM32 motor direction')
    c.add_argument('action', choices=['set', 'toggle'])
    c.add_argument('value', nargs='?', type=int, choices=[0, 1])
    args = parser.parse_args(argv)
    if args.command == 'direction' and (args.action == 'set') != (args.value is not None):
        parser.error('direction set takes 0 or 1, toggle takes no value')
    device = getattr(args, 'dev', None) or args.device

    setup = args.setup
    try:
        if not setup:
            fd, seen = open_serial(device, keep_dtr=args.no_exit)
            setup = mavlink_heartbeat_seen(seen)
            if setup:
                os.close(fd)
                print(f'{device}: MAVLink heartbeats, freeing the port for the ESC configuration server')
        if setup:
            device = mavlink_esc_mode(device)
            fd = open_serial(device, keep_dtr=args.no_exit)[0]
    except (CheckFailed, OSError) as e:
        print(f'FAIL: {e}')
        return 1
    status, exited = 0, False
    try:
        if args.command == 'check':
            run(fd, args.no_esc, args.variant.encode(), args.target, args.toggle_direction)
            exited = True  # the check ends with InterfaceExit
        else:
            run_command(fd, args)
            if not args.no_exit:
                fourway(fd, EXIT)
                exited = True
                print('InterfaceExit: the flight controller reboots')
    except (Exception, KeyboardInterrupt) as e:
        print(f'FAIL: {e}' if isinstance(e, (CheckFailed, OSError)) else f'FAIL: {type(e).__name__}: {e}')
        try:  # leave the FC in MSP mode; an InterfaceExit before passthrough is harmless
            os.write(fd, fourway_encode(FOURWAY_REQ, EXIT, 0, b'\0'))
            time.sleep(0.5)
            exited = True
        except OSError:
            pass
        status = 1
    finally:
        os.close(fd)
    # a session ends in a reboot after InterfaceExit or, on USB, when the port closes
    if exited:
        _wait_gone(device)  # a next run must not open the port of a board that is rebooting
    if status:
        return status
    print('PASS' if args.command == 'check' else 'done')
    return 0


if __name__ == '__main__':
    sys.exit(main())
