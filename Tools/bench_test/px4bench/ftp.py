"""MAVFTP helpers: directory listing, bounded file download, ULog constants.

Built on pymavlink.mavftp. cmd_get only sends OpenFileRO and returns; the
transfer itself is driven by process_ftp_reply, so ftp_download pumps it in
bounded slices with its own budget and stall detection: a transfer that
stops mid-file becomes an error naming the byte count, never a hang.
"""

import time

from . import send_heartbeat

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
    read handling, temp-file writes) -> __idle_task until idle or timeout.
    We use a completion callback so we know the burst read finished, and we
    pump process_ftp_reply in bounded slices so a stalled burst becomes a
    named FAIL instead of a hang. process_ftp_reply requires its timeout to
    be greater than idle_detection_time (default 3.7s).

    Returns (elapsed_seconds, None) on success, (None, error_string) on
    failure, so the caller can branch on the error explicitly instead of
    ever mixing a message string into arithmetic.
    """
    done = {'ok': False}

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

    ftp.cmd_get([remote], callback=on_complete)

    start = time.monotonic()
    last_bytes = 0
    last_progress = start
    next_hb = 0.0
    # process_ftp_reply timeout must exceed idle_detection_time; use a short
    # slice and loop so we can enforce our own budget and stall detection.
    slice_timeout = max(4.0, ftp.ftp_settings.idle_detection_time + 0.5)
    while not done['ok']:
        now = time.monotonic()
        if now > next_hb:
            send_heartbeat(mav)
            next_hb = now + 1.0
        if now - start > FTP_BUDGET:
            cur = ftp.read_total if ftp.read_total else last_bytes
            return None, 'MAVFTP transfer stalled at {} bytes (budget {:.0f}s)'.format(
                cur, FTP_BUDGET)
        # drive one slice of the FTP state machine
        ftp.process_ftp_reply('OpenFileRO', timeout=slice_timeout)
        cur = ftp.read_total
        if cur > last_bytes:
            last_bytes = cur
            last_progress = now
        elif now - last_progress > 30.0 and not done['ok']:
            # no forward progress for 30s and callback never fired
            return None, 'MAVFTP transfer stalled at {} bytes (no progress 30s)'.format(cur)
        # if the session ended without completing, process_ftp_reply keeps
        # returning immediately on idle; break out via the budget/stall checks
    elapsed = time.monotonic() - start
    return elapsed, None
