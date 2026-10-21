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

"""Download @PARAM/param.pck over MAVLink FTP and optionally check it against PARAM_VALUE.

Requires pymavlink. Firmware without the virtual file NAKs File Not Found.

Examples:
    Tools/mavftp_param.py /dev/ttyACM0
    Tools/mavftp_param.py /dev/ttyUSB0 -b 57600 --save param.pck
    Tools/mavftp_param.py udp:127.0.0.1:14550 --ftp-only
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import tempfile
import time

try:
    from pymavlink import mavftp, mavutil
except ImportError as e:
    print("Failed to import pymavlink: " + str(e))
    print("")
    print("You may need to install it with:")
    print("    pip3 install --user pymavlink")
    print("")
    sys.exit(2)

# pymavlink: add_message() can TypeError on FILE_TRANSFER_PROTOCOL sharing a
# link with instanced telemetry (BATTERY_STATUS, ...). Wrap it.
_orig_add_message = mavutil.add_message


def _safe_add_message(messages, mtype, msg):
    try:
        _orig_add_message(messages, mtype, msg)
    except TypeError:
        messages[mtype] = msg


mavutil.add_message = _safe_add_message

# PX4 has one FTP session and always ACKs session 0. pymavlink increments the
# session on an Open retry, then discards the ACK as the wrong session.
_orig_ftp_send = mavftp.MAVFTP._MAVFTP__send


def _ftp_send_session0(self, op):
    self.session = 0
    op.session = 0
    return _orig_ftp_send(self, op)


mavftp.MAVFTP._MAVFTP__send = _ftp_send_session0

# packed type 3/4 vs MAV_PARAM_TYPE_INT32/REAL32
_MAV_TO_PCK = {6: 3, 9: 4}


def _param_name(msg) -> str:
    raw = msg.param_id
    if isinstance(raw, bytes):
        return raw.split(b"\x00", 1)[0].decode("ascii", "replace")
    return raw.split("\x00", 1)[0]


def collect_param_value(m, timeout: float) -> dict[str, tuple[int, int]]:
    """name -> (mav_type, uint32 bits). Skips _HASH_CHECK."""
    t0 = time.time()
    while time.time() - t0 < 0.3:
        m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.05)

    m.mav.param_request_list_send(m.target_system, m.target_component)
    got: dict[str, tuple[int, int]] = {}
    expected = None
    deadline = time.time() + timeout

    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            if expected is not None and len(got) >= expected:
                break
            continue
        name = _param_name(msg)
        expected = int(msg.param_count)
        if name == "_HASH_CHECK":
            continue
        bits = struct.unpack("<I", struct.pack("<f", msg.param_value))[0]
        got[name] = (int(msg.param_type), bits)
        if expected and len(got) >= expected:
            extra_end = time.time() + 0.4
            while time.time() < extra_end:
                msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.1)
                if msg is None:
                    break
                name = _param_name(msg)
                if name == "_HASH_CHECK":
                    continue
                bits = struct.unpack("<I", struct.pack("<f", msg.param_value))[0]
                got[name] = (int(msg.param_type), bits)
            break

    return got, expected


def _radio_link(port: str, baud: int) -> bool:
    p = port.lower()
    if p.startswith(("udp", "tcp", "udpin", "tcpin")):
        return False
    if "ttyacm" in p or "fmu" in p or "cdc" in p:
        return False
    return baud <= 115200


def _ftp_settings(port: str, baud: int):
    # MAVFTP.__init__ ResetSessions uses process_ftp_reply timeout=5, which asserts
    # timeout > idle_detection_time. idle must also be > read_retry_time.
    # On SiK, a 0.5s burst retry restarts the stream while radio frames are still
    # in flight and blows a hole the gap-filler then has to walk.
    radio = _radio_link(port, baud)
    if radio:
        idle, read_retry, retry = 2.5, 2.0, 2.0
    else:
        idle, read_retry, retry = 1.2, 1.0, 0.5
    return mavftp.MAVFTPSettings([
        ("debug", int, 0),
        ("pkt_loss_tx", int, 0),
        ("pkt_loss_rx", int, 0),
        ("max_backlog", int, 5),
        ("burst_read_size", int, 239),
        ("write_size", int, 80),
        ("write_qsize", int, 5),
        ("idle_detection_time", float, idle),
        ("read_retry_time", float, read_retry),
        ("retry_time", float, retry),
    ]), radio


def _complete_pck(data) -> bool:
    if not data or len(data) < 6:
        return False
    magic, num, _total = struct.unpack("<HHH", data[:6])
    if magic not in (0x671B, 0x671C):
        return False
    packed = decode_pck(data)
    return packed is not None and len(packed) == num


def _drain_ftp(m, quiet: float, cap: float) -> int:
    """Discard in-flight FILE_TRANSFER_PROTOCOL until the link is quiet."""
    n = 0
    deadline = time.time() + cap
    quiet_since = time.time()
    while time.time() < deadline:
        msg = m.recv_match(type="FILE_TRANSFER_PROTOCOL", blocking=True,
                           timeout=min(0.2, quiet))
        if msg is None:
            if time.time() - quiet_since >= quiet:
                break
            continue
        n += 1
        quiet_since = time.time()
    return n


def ftp_get(m, remote: str, timeout: float, settings, drain: bool):
    ftp = mavftp.MAVFTP(m, target_system=m.target_system, target_component=m.target_component,
                        settings=settings)
    if drain:
        _drain_ftp(m, quiet=0.4, cap=4.0)
    holder = {"data": None}

    def cb(fh):
        if fh is not None:
            pos = fh.tell()
            fh.seek(0)
            holder["data"] = fh.read()
            fh.seek(pos)

    # pymavlink writes self.filename after the callback even when using BytesIO
    fd, local = tempfile.mkstemp(prefix="mavftp-param-", suffix=".pck")
    os.close(fd)
    try:
        ret = ftp.cmd_get([remote, local], callback=cb)
        if ret.error_code != mavftp.FtpError.Success:
            return None, ret
        ret = ftp.process_ftp_reply("OpenFileRO", timeout=timeout)
        data = holder["data"]
        if not _complete_pck(data) and os.path.isfile(local):
            with open(local, "rb") as f:
                on_disk = f.read()
            if _complete_pck(on_disk):
                data = on_disk
        return data, ret
    finally:
        try:
            os.unlink(local)
        except OSError:
            pass


def decode_pck(data: bytes):
    pdata = mavftp.MAVFTP.ftp_param_decode(data)
    if pdata is None:
        return None
    values = {}
    for name, value, ptype in pdata.params:
        n = name.decode("utf-8") if isinstance(name, bytes) else name
        if ptype == 3:
            bits = struct.unpack("<I", struct.pack("<i", int(value)))[0]
        else:
            bits = struct.unpack("<I", struct.pack("<f", float(value)))[0]
        values[n] = (ptype, bits)
    return values


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("port", help="MAVLink connection (serial device, udp:HOST:PORT, tcp:HOST:PORT)")
    p.add_argument("--baudrate", "-b", type=int, default=57600, help="serial baud rate (default: %(default)s)")
    p.add_argument("--timeout", type=float, default=60, help="seconds for FTP and PARAM_VALUE (default: %(default)s)")
    p.add_argument("--attempts", type=int, default=3, help="FTP Open/burst attempts (default: %(default)s)")
    p.add_argument("--withdefaults", action="store_true", default=True,
                   help="request ?withdefaults=1 (default)")
    p.add_argument("--no-withdefaults", action="store_false", dest="withdefaults")
    p.add_argument("--ftp-only", action="store_true", help="skip the PARAM_VALUE comparison")
    p.add_argument("--save", metavar="FILE", help="write the packed file to FILE")
    args = p.parse_args()

    remote = "@PARAM/param.pck?withdefaults=1" if args.withdefaults else "@PARAM/param.pck"

    m = mavutil.mavlink_connection(args.port, baud=args.baudrate, autoreconnect=True, source_system=250)
    hb = m.wait_heartbeat(timeout=8)
    if hb is None:
        print(f"no heartbeat on {args.port}")
        return 1
    print(f"heartbeat sys={hb.get_srcSystem()} comp={hb.get_srcComponent()}")

    stream = {}
    expected = None
    if not args.ftp_only:
        t0 = time.time()
        stream, expected = collect_param_value(m, args.timeout)
        print(f"PARAM_VALUE: {len(stream)} params" +
              (f" (of {expected})" if expected else "") +
              f" in {time.time() - t0:.2f}s")

    settings, radio = _ftp_settings(args.port, args.baudrate)
    if radio:
        print(f"radio timings (baud={args.baudrate}): idle={settings.idle_detection_time}s "
              f"open-retry={settings.read_retry_time}s burst-retry={settings.retry_time}s")
        n = _drain_ftp(m, quiet=0.4, cap=4.0)
        if n:
            print(f"drained {n} leftover FTP frames")

    t0 = time.time()
    data, ret = None, None
    for attempt in range(1, max(1, args.attempts) + 1):
        data, ret = ftp_get(m, remote, args.timeout, settings, drain=radio)
        if _complete_pck(data):
            break
        nbytes = 0 if data is None else len(data)
        err = "?" if ret is None else f"error={ret.error_code} errno={ret.system_error}"
        print(f"FTP attempt {attempt}/{args.attempts} failed: {err} bytes={nbytes}")
        if attempt < args.attempts:
            if radio:
                _drain_ftp(m, quiet=0.4, cap=4.0)
            else:
                time.sleep(0.5)
    t_ftp = time.time() - t0
    nbytes = 0 if data is None else len(data)
    err = "?" if ret is None else f"error={ret.error_code} errno={ret.system_error}"
    print(f"FTP {remote}: {err} {t_ftp:.2f}s bytes={nbytes}")
    m.close()

    if not _complete_pck(data):
        return 1

    if args.save:
        with open(args.save, "wb") as f:
            f.write(data)
        print(f"wrote {args.save}")

    packed = decode_pck(data)
    if packed is None:
        print("failed to decode packed file")
        return 1

    magic, num, total = struct.unpack("<HHH", data[:6])
    print(f"packed magic=0x{magic:x} num={num} total={total} decoded={len(packed)}")

    if args.ftp_only:
        return 0 if len(packed) == num else 1

    only_stream = sorted(set(stream) - set(packed))
    only_packed = sorted(set(packed) - set(stream))
    mismatches = []
    compared = 0
    for name, (mav_type, bits) in stream.items():
        if name not in packed:
            continue
        ptype, pbits = packed[name]
        if _MAV_TO_PCK.get(mav_type) != ptype or bits != pbits:
            mismatches.append(name)
        compared += 1

    print(f"compared {compared}; only_stream={len(only_stream)} only_packed={len(only_packed)} mismatches={len(mismatches)}")
    if only_stream[:8]:
        print("  only_stream", only_stream[:8])
    if only_packed[:8]:
        print("  only_packed", only_packed[:8])
    if mismatches[:8]:
        print("  mismatches", mismatches[:8])

    stream_complete = expected is not None and len(stream) >= expected
    if mismatches or only_stream:
        return 1
    if stream_complete and only_packed:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
