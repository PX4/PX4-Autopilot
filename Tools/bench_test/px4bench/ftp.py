"""MAVFTP helpers: directory listing, bounded file download, ULog constants.

Built on pymavlink.mavftp. cmd_get only sends OpenFileRO and returns; the
transfer itself is driven by process_ftp_reply, so ftp_download drives it
within its own budget, with the idle detection of pymavlink as the stall
detection: a transfer that stops mid-file becomes an error naming the byte
count, never a hang.
"""

import sys
import time

from . import send_heartbeat

MAVFTP_INSTALL_HINT = """\
Failed to import pymavlink.mavftp: {err}

The mavftp module ships with pymavlink >= 2.4.42. Upgrade with:
    pip3 install --user --upgrade pymavlink
"""

try:
    from pymavlink import mavftp
except ImportError as e:
    print(MAVFTP_INSTALL_HINT.format(err=e))
    sys.exit(2)

LOG_ROOT = '/fs/microsd/log'
# ULog file header magic: 'U' 'L' 'o' 'g' 0x01 0x12 0x35, then a file-version
# byte. See src/modules/logger/logger.cpp:2099-2106.
ULOG_MAGIC7 = bytes([0x55, 0x4C, 0x6F, 0x67, 0x01, 0x12, 0x35])
FTP_BUDGET = 300.0              # overall MAVFTP budget in seconds


def ftp_list(ftp, path):
    """List a remote directory. cmd_list drives its own reply loop internally
    and populates ftp.list_result. Returns the list of DirectoryEntry.
    """
    ftp.cmd_list([path])
    return list(ftp.list_result)


def ftp_download(mav, ftp, remote, local, report):
    """Download one remote file to a local path over MAVFTP, bounded by
    FTP_BUDGET and reporting a stall by byte count.

    cmd_get only sends OpenFileRO and returns; the transfer itself is driven
    by process_ftp_reply, which loops recv_match -> __mavlink_packet (burst
    read handling, temp-file writes) -> __idle_task until the read completes,
    the link is idle for idle_detection_time (default 3.7s), or the timeout.
    It is driven in one call bounded by FTP_BUDGET: process_ftp_reply
    terminates the session when it times out, so shorter slices would abort
    any transfer that takes longer than one slice (a slow link, a big log).
    A stalled burst read ends on the idle detection and becomes a named FAIL
    with the byte count, never a hang. Heartbeats go out from the progress
    callback, which runs on every burst read reply.

    Returns (elapsed_seconds, None) on success, (None, error_string) on
    failure, so the caller can branch on the error explicitly instead of
    ever mixing a message string into arithmetic.
    """
    done = {'ok': False}
    next_hb = [0.0]

    def on_complete(fh):
        # fh is a BytesIO on success, or None if the session was terminated
        # (failure). cmd_get with a callback keeps the payload in memory, so we
        # persist it here ourselves.
        if fh is None:
            return
        try:
            fh.seek(0)
            with open(local, 'wb') as out:
                out.write(fh.read())
            done['ok'] = True
        except Exception as e:  # noqa: BLE001 - surface write failures to the caller
            report.info('local write failed: {}'.format(e))

    def on_progress(_fraction):
        now = time.monotonic()
        if now > next_hb[0]:
            send_heartbeat(mav)
            next_hb[0] = now + 1.0

    send_heartbeat(mav)
    start = time.monotonic()
    ftp.cmd_get([remote], callback=on_complete, progress_callback=on_progress)
    ret = ftp.process_ftp_reply('OpenFileRO', timeout=FTP_BUDGET)

    if not done['ok']:
        elapsed = time.monotonic() - start
        if elapsed >= FTP_BUDGET:
            return None, 'MAVFTP transfer stalled at {} bytes (budget {:.0f}s)'.format(
                ftp.read_total, FTP_BUDGET)
        return None, 'MAVFTP transfer stalled at {} bytes ({})'.format(
            ftp.read_total, ret.error_code.name)
    elapsed = time.monotonic() - start
    return elapsed, None
