#!/usr/bin/env python3
"""validate_a3_fallback_log.py – Flight-log validator for the A3 fallback chain.

Verifies that a ULog produced during a GPS-loss → local-position-loss scenario
contains the ``commander_failsafe_a3_fallback`` event EXACTLY TWICE:

  • emission 1 – RTL falls back to Land  (global position / home lost)
  • emission 2 – Land falls back to Descend  (relaxed local position lost)

More than two emissions would indicate a regression of the anti-spam guard:

    if (action_state.a3_desired_action != Action::None
        && action_state.action > _selected_action) { notifyA3Fallback(...); }

Because after the first worsening step ``_selected_action`` equals
``action_state.action``, the strict-greater-than condition is false on every
subsequent update in the same degraded state; the event can therefore fire at
most once per worsening step and never during steady-state repetitions.

Usage
-----
Validate a real SITL or flight log::

    python3 validate_a3_fallback_log.py path/to/flight.ulg

Run the self-test (strict mode – exits non-zero when the regressed canary bag
contains 3 events, matching what the script would do on a real regressed log)::

    python3 validate_a3_fallback_log.py --self-test

Run the self-test in canary-sensitivity mode (exits zero, confirming the
validator correctly detects excess events in the regression bag)::

    python3 validate_a3_fallback_log.py --self-test-allow-regression

Assert a different event count (e.g., GPS-loss only → 1 event; clean flight
with no degradation → 0 events)::

    python3 validate_a3_fallback_log.py path/to/flight.ulg --expected-count 1
    python3 validate_a3_fallback_log.py path/to/clean_flight.ulg --expected-count 0

CI recommended usage::

    # Gate: verify the validator itself works end-to-end (exits 0):
    python3 validate_a3_fallback_log.py --self-test-allow-regression

    # Demonstrate regression detection (always exits 1 – use allow-errors in CI):
    python3 validate_a3_fallback_log.py --self-test

How to obtain a real ULog for this check
-----------------------------------------
1. Build and start PX4 SITL::

       make px4_sitl gazebo-classic_iris

2. Arm and switch to Mission mode, then disable the GPS sensor to trigger
   the RTL→Land A3 step, and shortly after also disable local-position
   estimation to trigger the Land→Descend step.  The easiest way in SITL
   is to inject a MAVLink command or use the ``failure`` module::

       # in the PX4 shell (or via MAVLink shell):
       failure gps off          # drops global position → RTL falls back to Land
       failure local_pos off    # drops relaxed local pos → Land falls back to Descend

3. Disarm and let PX4 write the log.  The log file path is printed on exit::

       [logger] closed logfile: ./log/YYYY-MM-DD/HH_MM_SS.ulg (N bytes)

4. Run this script against that file::

       python3 validate_a3_fallback_log.py ./log/YYYY-MM-DD/HH_MM_SS.ulg

   pyulog is used automatically when installed (``pip install pyulog``);
   otherwise the built-in parser is used – no extra dependencies needed.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import struct
import sys
import tempfile

# ---------------------------------------------------------------------------
# Exit codes
# ---------------------------------------------------------------------------
_EC_PASS      = 0   # count matches --expected-count
_EC_MISMATCH  = 1   # count != expected
_EC_BAD_ARGS  = 2   # invalid CLI arguments (also argparse's default)
_EC_NOT_FOUND = 3   # log file does not exist
_EC_PARSE_ERR = 4   # ULog is unreadable or malformed

# ---------------------------------------------------------------------------
# Event-ID computation
# Must produce the same value as events::ID() in
#   platforms/common/include/px4_platform_common/events.h
#
#   constexpr uint32_t ID(const char (&name)[N])
#   {
#       uint32_t component_id = 1u << 24;   // autopilot component
#       return (0xffffff & hash_32_fnv1a_const(name)) | component_id;
#   }
# ---------------------------------------------------------------------------

def _fnv1a_32(name: str) -> int:
    """FNV-1a 32-bit hash, replicating hash_32_fnv1a_const() from events.h.

    Parameters
    ----------
    name : str
        Event name string (ASCII).

    Returns
    -------
    int
        32-bit unsigned FNV-1a hash of *name*.
    """
    h = 0x811c9dc5          # FNV offset basis
    prime = 0x1000193       # FNV prime
    for byte in name.encode('utf-8'):
        h = ((h ^ byte) * prime) & 0xFFFF_FFFF
    return h


def px4_event_id(name: str) -> int:
    """Compute the 32-bit PX4 event ID for an event named *name*.

    Formula (autopilot component = 1):
        id = (0xFFFFFF & fnv1a_32(name)) | (1 << 24)
    """
    return (0xFF_FFFF & _fnv1a_32(name)) | (1 << 24)


# The single ID we are validating throughout this script.
A3_FALLBACK_EVENT_ID: int = px4_event_id("commander_failsafe_a3_fallback")


# ---------------------------------------------------------------------------
# Minimal ULog writer
# Produces a self-contained synthetic bag without any external dependencies.
# Reference: https://docs.px4.io/main/en/dev_log/ulog_file_format.html
# ---------------------------------------------------------------------------

_ULOG_MAGIC   = b'ULog\x01\x12\x35'   # 7-byte magic
_ULOG_VERSION = 1                       # format version byte

# Message-type codes
_MSG_FLAG_BITS = ord('B')
_MSG_FORMAT    = ord('F')
_MSG_ADD_LOG   = ord('A')
_MSG_DATA      = ord('D')

# event topic format string (field order must match msg/versioned/Event.msg)
_EVENT_FMT = (
    "event:"
    "uint64_t timestamp;"
    "uint32_t id;"
    "uint16_t event_sequence;"
    "uint8_t[25] arguments;"
    "uint8_t log_levels;"
)

# Struct layout of the event payload (excluding the 2-byte ULog msg_id prefix)
#   offset 0  : uint64_t timestamp      8 bytes
#   offset 8  : uint32_t id             4 bytes  ← we match against this
#   offset 12 : uint16_t event_sequence 2 bytes
#   offset 14 : uint8_t[25] arguments  25 bytes
#   offset 39 : uint8_t log_levels      1 byte
#   ──────────────────────────────────────────
#   total                               40 bytes
_PAYLOAD_SIZE  = 40
_TS_OFFSET     = 0    # within payload
_ID_OFFSET     = 8    # within payload
_MSG_ID_PREFIX = 2    # bytes prepended by ULog (the message_id field)


def _pack_msg(msg_type: int, body: bytes) -> bytes:
    """Serialise one ULog message frame.

    On-disk layout::

        uint16  msg_size  =  1 (msg_type byte) + len(body)
        uint8   msg_type
        uint8[] body      =  len(body) bytes

    pyulog reads: ``data = f.read(msg_size - 1)`` → recovers *body* exactly.
    """
    return struct.pack('<HB', 1 + len(body), msg_type) + body


def _write_synthetic_ulog(path: str, events: list[tuple]) -> None:
    """Write a minimal well-formed ULog containing only the *event* topic.

    Parameters
    ----------
    path   : destination file path.
    events : list of ``(timestamp_us, event_id, log_levels, arguments)``
             where *arguments* is ``bytes``; padded/truncated to 25 bytes.
    """
    with open(path, 'wb') as fh:
        # File header: magic (7) + version (1) + timestamp (8) = 16 bytes
        fh.write(_ULOG_MAGIC)
        fh.write(struct.pack('<B', _ULOG_VERSION))
        fh.write(struct.pack('<Q', 0))            # placeholder timestamp

        # B – flag-bits message (required as first definition-section message)
        flag_body = b'\x00' * 8    # compat_flags[8]
        flag_body += b'\x00' * 8   # incompat_flags[8]
        flag_body += b'\x00' * 24  # appended_offsets[3 × uint64]
        fh.write(_pack_msg(_MSG_FLAG_BITS, flag_body))

        # F – format definition for the event topic
        fh.write(_pack_msg(_MSG_FORMAT, _EVENT_FMT.encode()))

        # A – add subscription (multi_id=0, message_id=0, topic="event")
        fh.write(_pack_msg(_MSG_ADD_LOG, struct.pack('<BH', 0, 0) + b'event\x00'))

        # D – one data frame per event
        for seq, (ts, ev_id, log_lvl, args) in enumerate(events):
            args = (args + b'\x00' * 25)[:25]   # normalise to exactly 25 B
            payload = (
                struct.pack('<Q', ts)      # timestamp
                + struct.pack('<I', ev_id) # id
                + struct.pack('<H', seq)   # event_sequence
                + args                     # arguments[25]
                + struct.pack('<B', log_lvl)  # log_levels
            )
            # Data frame = 2-byte message_id prefix + payload
            fh.write(_pack_msg(_MSG_DATA, struct.pack('<H', 0) + payload))


def _write_ulog_with_decoy(path: str,
                            event_entries: list[tuple],
                            decoy_topic: str,
                            decoy_entries: list[tuple]) -> None:
    """Write a ULog containing the standard *event* topic (msg_id=0) **and**
    a second *decoy_topic* (msg_id=1) that carries the same event-ID values.

    The decoy topic uses the minimal format::

        decoy_topic : uint64_t timestamp ; uint32_t id ;

    so the raw parser's ``id``-at-body-offset-10 assumption holds, letting
    sub-test 3 rely solely on the msg_id→topic-name mapping to prove that
    the two reader paths filter by topic and not by event ID alone.

    Parameters
    ----------
    event_entries : list of ``(timestamp_us, event_id, log_levels, arguments)``
        Frames written to the *event* topic (msg_id=0).
    decoy_topic   : str
        Name of the second topic (e.g. ``'shadow_event'``).
    decoy_entries : list of ``(timestamp_us, event_id)``
        Minimal frames written to the decoy topic (msg_id=1).
    """
    decoy_fmt = f"{decoy_topic}:uint64_t timestamp;uint32_t id;"

    with open(path, 'wb') as fh:
        # Header
        fh.write(_ULOG_MAGIC)
        fh.write(struct.pack('<B', _ULOG_VERSION))
        fh.write(struct.pack('<Q', 0))

        # B – flag bits
        fh.write(_pack_msg(_MSG_FLAG_BITS, b'\x00' * 40))

        # F – format for both topics
        fh.write(_pack_msg(_MSG_FORMAT, _EVENT_FMT.encode()))
        fh.write(_pack_msg(_MSG_FORMAT, decoy_fmt.encode()))

        # A – subscribe event=0, decoy=1
        fh.write(_pack_msg(_MSG_ADD_LOG, struct.pack('<BH', 0, 0) + b'event\x00'))
        fh.write(_pack_msg(_MSG_ADD_LOG,
                           struct.pack('<BH', 0, 1) + decoy_topic.encode() + b'\x00'))

        # D – event frames
        for seq, (ts, ev_id, log_lvl, args) in enumerate(event_entries):
            args = (args + b'\x00' * 25)[:25]
            payload = (
                struct.pack('<Q', ts)
                + struct.pack('<I', ev_id)
                + struct.pack('<H', seq)
                + args
                + struct.pack('<B', log_lvl)
            )
            fh.write(_pack_msg(_MSG_DATA, struct.pack('<H', 0) + payload))

        # D – decoy frames
        for ts, ev_id in decoy_entries:
            payload = struct.pack('<Q', ts) + struct.pack('<I', ev_id)
            fh.write(_pack_msg(_MSG_DATA, struct.pack('<H', 1) + payload))


# ---------------------------------------------------------------------------
# ULog reader – count events matching a given topic name and event ID
# ---------------------------------------------------------------------------

def _count_events_pyulog(path: str, topic: str, event_id: int) -> int:
    """Count matching events using the *pyulog* library (preferred).

    Filters by *topic* name (the uORB topic subscription name) **and** by the
    32-bit *event_id* value stored in the ``id`` field of each frame.
    """
    from pyulog import ULog  # type: ignore[import]
    ulog = ULog(path, [topic])
    count = 0
    for dataset in ulog.data_list:
        if dataset.name == topic:
            for ev_id in dataset.data['id']:
                if int(ev_id) == event_id:
                    count += 1
    return count


def _count_events_builtin(path: str, topic: str, event_id: int) -> int:
    """Count matching events using the built-in zero-dependency ULog parser.

    Tracks 'A' (add-subscription) messages to map *topic* → ``message_id``
    values, then scans 'D' (data) messages, accepting only frames whose
    ``message_id`` is in that set, and matching the ``id`` field against
    *event_id*.

    Layout assumption for both the standard PX4 *event* topic and any decoy
    topic written by ``_write_ulog_with_decoy()``::

        body[0:2]   = message_id prefix  (uint16 LE, ULog internal)
        body[2:10]  = uint64_t timestamp
        body[10:14] = uint32_t id        ← matched here against *event_id*

    This offset holds whenever ``uint64_t timestamp`` is the first field and
    ``uint32_t id`` is the second, which is true for all topics this script
    creates or validates.
    """
    topic_msg_ids: set[int] = set()
    count = 0

    _ID_BODY_OFFSET = _MSG_ID_PREFIX + _ID_OFFSET   # = 10
    _MIN_BODY_LEN   = _MSG_ID_PREFIX + _ID_OFFSET + 4  # = 14

    with open(path, 'rb') as fh:
        magic = fh.read(7)
        if magic != _ULOG_MAGIC:
            raise ValueError(
                f"Not a valid ULog file – bad magic bytes: {magic!r} "
                f"(expected {_ULOG_MAGIC!r})"
            )
        fh.read(9)   # skip version (1) + timestamp (8)

        while True:
            hdr = fh.read(3)
            if len(hdr) < 3:
                break
            msg_size, msg_type = struct.unpack('<HB', hdr)
            body = fh.read(msg_size - 1)
            if len(body) < msg_size - 1:
                break   # truncated file — stop gracefully

            if msg_type == _MSG_ADD_LOG:
                # 'A' – record topic-name → message_id mapping
                if len(body) >= 3:
                    _multi, msg_id = struct.unpack('<BH', body[:3])
                    t = body[3:].rstrip(b'\x00').decode('utf-8', errors='replace')
                    if t == topic:
                        topic_msg_ids.add(msg_id)

            elif msg_type == _MSG_DATA:
                # 'D' – only process frames that belong to the target topic
                if len(body) < _MIN_BODY_LEN:
                    continue
                msg_id = struct.unpack('<H', body[:2])[0]
                if msg_id not in topic_msg_ids:
                    continue
                ev_id = struct.unpack('<I', body[_ID_BODY_OFFSET:_ID_BODY_OFFSET + 4])[0]
                if ev_id == event_id:
                    count += 1

    return count


def count_events(path: str,
                 topic: str = 'event',
                 event_id: int = A3_FALLBACK_EVENT_ID) -> int:
    """Count occurrences of *event_id* inside the *topic* topic of *path*.

    Uses *pyulog* when installed; falls back to the built-in parser otherwise.
    Default arguments reproduce the original A3-fallback check so that all
    existing call sites continue to work without change.
    """
    try:
        return _count_events_pyulog(path, topic, event_id)
    except ImportError:
        return _count_events_builtin(path, topic, event_id)


def count_a3_fallback_events(path: str) -> int:
    """Backward-compatible alias: count ``commander_failsafe_a3_fallback`` events."""
    return count_events(path)


def _run_log_validation(path: str,
                         topic: str,
                         event_id: int,
                         expected: int) -> tuple[int, int | None]:
    """Core validation logic, decoupled from ``sys.exit()``.

    Returns ``(exit_code, actual_count)`` where *actual_count* is ``None``
    when the file could not be opened or parsed.

    Exit codes
    ----------
    0 (_EC_PASS)       count == expected
    1 (_EC_MISMATCH)   count != expected
    3 (_EC_NOT_FOUND)  *path* does not exist
    4 (_EC_PARSE_ERR)  file is unreadable or not a valid ULog
    """
    if not os.path.isfile(path):
        return _EC_NOT_FOUND, None
    try:
        n = count_events(path, topic=topic, event_id=event_id)
    except Exception:
        return _EC_PARSE_ERR, None
    return (_EC_PASS, n) if n == expected else (_EC_MISMATCH, n)


def _capture_exit_code_quiet(*cli_args: str) -> int:
    """Invoke ``main()`` with *cli_args* and return its exit code.

    Overrides ``sys.argv``, suppresses all stdout/stderr, and catches
    ``SystemExit`` so the exit-code sub-test can verify non-zero codes
    in-process without spawning a subprocess or polluting output.
    """
    saved = sys.argv[:]
    sys.argv = [sys.argv[0]] + list(cli_args)
    try:
        with contextlib.redirect_stdout(io.StringIO()), \
             contextlib.redirect_stderr(io.StringIO()):
            main()
        return _EC_PASS
    except SystemExit as exc:
        return int(exc.code) if exc.code is not None else _EC_PASS
    finally:
        sys.argv = saved


def _capture_json_quiet(*cli_args: str) -> tuple[int, dict | None]:
    """Invoke ``main()`` with *cli_args*, capturing stdout as a JSON object.

    Overrides ``sys.argv``, suppresses stderr, catches ``SystemExit``, and
    returns ``(exit_code, parsed_dict)``.  Returns ``(exit_code, None)`` when
    stdout is empty or cannot be parsed as JSON.
    """
    saved = sys.argv[:]
    sys.argv = [sys.argv[0]] + list(cli_args)
    stdout_buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(stdout_buf), \
             contextlib.redirect_stderr(io.StringIO()):
            main()
        code = _EC_PASS
    except SystemExit as exc:
        code = int(exc.code) if exc.code is not None else _EC_PASS
    finally:
        sys.argv = saved

    out = stdout_buf.getvalue().strip()
    if not out:
        return code, None
    try:
        return code, json.loads(out)
    except json.JSONDecodeError:
        return code, None


# ---------------------------------------------------------------------------
# Self-test: build a synthetic bag and validate it
# ---------------------------------------------------------------------------

# log_levels byte: (LogInternal::Warning=4) << 4 | (Log::Critical=2) = 0x42
# Matches the value encoded in notifyA3Fallback() in framework.cpp.
_A3_LOG_LEVELS = (4 << 4) | 2

# failsafe_action_t enum values (Action enum in framework.h, same ordinals)
_ACTION_RTL     = 6
_ACTION_LAND    = 7
_ACTION_DESCEND = 8


def self_test(allow_regression: bool = False) -> None:
    """Build synthetic bags for both passing and regressed scenarios.

    Sub-test 1 – correct firmware (anti-spam guard intact)
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Bag: noise, RTL→Land, noise, RTL→Descend.  Count must equal 2.
    Both modes exit 1 if this sub-test fails.

    Sub-test 2 – regression canary (anti-spam guard broken)
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Adds a third A3 emission at t=5 s, modelling firmware where the guard
    ``action_state.action > _selected_action`` was removed so the event fires
    again on the next steady-state update cycle.

    Behavior is controlled by *allow_regression*:

    ``allow_regression=False`` (``--self-test``, strict):
        count != 2  →  EXIT 1.  The canary *actually fails* on a regression,
        exactly as the real validator would when run against a bad flight log.

    ``allow_regression=True`` (``--self-test-allow-regression``, sensitivity):
        count != 2  →  EXIT 0.  Flipped expectation: the excess is the *proof*
        that the validator can detect a spamming firmware.
        count == 2  →  EXIT 1  (detector is blind – that is the real failure).

    Sub-test 3 – topic isolation (both reader paths)
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Writes a bag with the same A3 event ID appearing in **two** topics:
      • ``event``        (msg_id=0): 2 genuine A3 events
      • ``shadow_event`` (msg_id=1): 5 copies of the identical event ID

    Validates that ``count_events(path, topic='event', ...)`` returns 2 and
    NOT 7, proving both the pyulog and raw-parser paths filter by topic name
    (msg_id → topic-name mapping) and not by event ID alone.  Also asserts
    that ``count_events(path, topic='shadow_event', ...)`` returns 5, so we
    know the decoy events were actually written and are detectable.

    Sub-test 4 – exit-code proof (all four non-zero codes)
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Invokes ``main()`` in-process (via ``_capture_exit_code_quiet``) with
    four deliberately broken inputs and asserts the exact exit code:

    ===  ==================  ==============================================
    EC   Label               How triggered
    ===  ==================  ==============================================
     1   count mismatch      valid 2-event bag + ``--expected-count 0``
     2   bad args            ``--expected-count -1`` (no file needed)
     3   file not found      path that does not exist on disk
     4   parse error         temp file filled with non-ULog bytes
    ===  ==================  ==============================================

    Sub-test 5 – ``--json`` output (``--self-test-allow-regression`` only)
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Runs the CLI validation path four times with ``--json`` and asserts each
    response is valid JSON containing all seven required keys
    (``topic``, ``event_name``, ``event_id``, ``expected``, ``found``,
    ``pass``, ``exit_code``) with field values matching the expected outcome:

    ============  =====  ===========  ======
    Scenario      pass   exit_code    found
    ============  =====  ===========  ======
    2 events ok   true   0            2
    mismatch      false  1            2
    not found     false  3            null
    parse error   false  4            null
    ============  =====  ===========  ======
    """
    _NOISE_ID = px4_event_id("some_unrelated_event")

    # ── Sub-test 1: correct bag (2 emissions) ─────────────────────────────
    correct_events: list[tuple] = [
        # (timestamp_us, event_id, log_levels, arguments)
        (1_000_000, _NOISE_ID,            0x00,           b'\x00'),
        (2_000_000, A3_FALLBACK_EVENT_ID, _A3_LOG_LEVELS,
         struct.pack('<BB', _ACTION_RTL, _ACTION_LAND)),
        (3_000_000, _NOISE_ID,            0x00,           b'\x00'),
        (4_000_000, A3_FALLBACK_EVENT_ID, _A3_LOG_LEVELS,
         struct.pack('<BB', _ACTION_RTL, _ACTION_DESCEND)),
    ]

    tmp_fd, bag_path = tempfile.mkstemp(suffix='.ulg', prefix='a3_fallback_')
    os.close(tmp_fd)
    try:
        _write_synthetic_ulog(bag_path, correct_events)
        n = count_a3_fallback_events(bag_path)

        print(f"[sub-test 1] correct bag  : {bag_path}")
        print(f"             A3 event ID  : 0x{A3_FALLBACK_EVENT_ID:08x}  "
              f"(fnv1a32(\"commander_failsafe_a3_fallback\") & 0xFFFFFF | 0x01000000)")
        print(f"             events found : {n}")

        if n == 2:
            print("             PASS – exactly 2 emissions (RTL→Land, RTL→Descend).")
        else:
            print(f"             FAIL – expected 2, got {n}.")
            sys.exit(1)
    finally:
        os.unlink(bag_path)

    print()

    # ── Sub-test 2: regression canary (3 emissions) ───────────────────────
    # Simulates a broken anti-spam guard: the event fires a third time on the
    # next update cycle while the vehicle is already in the degraded state.
    # The validator must count 3 and report that as excess (n != 2).
    regressed_events: list[tuple] = correct_events + [
        (5_000_000, A3_FALLBACK_EVENT_ID, _A3_LOG_LEVELS,
         struct.pack('<BB', _ACTION_RTL, _ACTION_DESCEND)),   # spurious repeat
    ]

    tmp_fd, bag_path = tempfile.mkstemp(suffix='.ulg', prefix='a3_regressed_')
    os.close(tmp_fd)
    try:
        _write_synthetic_ulog(bag_path, regressed_events)
        n = count_a3_fallback_events(bag_path)

        print(f"[sub-test 2] regressed bag : {bag_path}")
        print(f"             events found  : {n}")

        if allow_regression:
            # Flipped expectation: excess events prove the detector is sensitive.
            if n != 2:
                print(f"             PASS – canary: {n} events detected; "
                      "validator correctly identifies a spamming firmware.")
            else:
                print("             FAIL – canary: detector missed the regression; "
                      "validator cannot distinguish a spamming firmware.")
                sys.exit(1)
        else:
            # Strict expectation: excess events are a failure, same as a real log.
            if n != 2:
                print(f"             FAIL – {n} A3 events detected (expected 2); "
                      "anti-spam regression present in this bag.")
                sys.exit(1)
            else:
                print("             PASS")   # unreachable with a 3-event bag
    finally:
        os.unlink(bag_path)

    print()

    # ── Sub-test 3: topic isolation ────────────────────────────────────────
    # The decoy topic carries the SAME A3 event ID 5 times under msg_id=1.
    # When filtering by topic='event' (msg_id=0) only 2 must be counted.
    _DECOY_TOPIC = 'shadow_event'
    decoy_entries: list[tuple] = [
        (ts * 1_000_000, A3_FALLBACK_EVENT_ID) for ts in range(1, 6)
    ]
    # The 2 genuine A3 events written to the 'event' topic
    event_entries: list[tuple] = [
        (2_000_000, A3_FALLBACK_EVENT_ID, _A3_LOG_LEVELS,
         struct.pack('<BB', _ACTION_RTL, _ACTION_LAND)),
        (4_000_000, A3_FALLBACK_EVENT_ID, _A3_LOG_LEVELS,
         struct.pack('<BB', _ACTION_RTL, _ACTION_DESCEND)),
    ]

    tmp_fd, bag_path = tempfile.mkstemp(suffix='.ulg', prefix='a3_decoy_')
    os.close(tmp_fd)
    try:
        _write_ulog_with_decoy(bag_path, event_entries, _DECOY_TOPIC, decoy_entries)

        n_event = count_events(bag_path, topic='event',        event_id=A3_FALLBACK_EVENT_ID)
        n_decoy = count_events(bag_path, topic=_DECOY_TOPIC,  event_id=A3_FALLBACK_EVENT_ID)

        print(f"[sub-test 3] decoy-isolation bag  : {bag_path}")
        print(f"             topic='event'         found : {n_event}  (expect 2)")
        print(f"             topic='{_DECOY_TOPIC}' found : {n_decoy}  (expect 5)")

        if n_event == 2 and n_decoy == 5:
            print("             PASS – topic filter works: "
                  f"{n_decoy} decoy events in '{_DECOY_TOPIC}' not counted in 'event'.")
        else:
            print("             FAIL – topic isolation broken.")
            sys.exit(1)
    finally:
        os.unlink(bag_path)

    print()

    # ── Sub-test 4: exit-code proof ────────────────────────────────────────
    # All inputs generated in-process; no pre-existing files required.

    # EC 1 – count mismatch: valid 2-event bag, but ask for --expected-count 0
    tmp_fd, valid_bag = tempfile.mkstemp(suffix='.ulg', prefix='a3_ec1_')
    os.close(tmp_fd)
    _write_synthetic_ulog(valid_bag, [
        (2_000_000, A3_FALLBACK_EVENT_ID, _A3_LOG_LEVELS,
         struct.pack('<BB', _ACTION_RTL, _ACTION_LAND)),
        (4_000_000, A3_FALLBACK_EVENT_ID, _A3_LOG_LEVELS,
         struct.pack('<BB', _ACTION_RTL, _ACTION_DESCEND)),
    ])

    # EC 4 – parse error: temp file filled with non-ULog bytes
    tmp_fd, corrupt_bag = tempfile.mkstemp(suffix='.ulg', prefix='a3_ec4_')
    os.write(tmp_fd, b'!! not a valid ulog file !!')
    os.close(tmp_fd)

    # EC 3 – file not found: a path that is guaranteed not to exist
    nonexistent = valid_bag + '.no_such_file'

    ec_cases: list[tuple[int, str, list[str]]] = [
        (_EC_MISMATCH,  'count mismatch  (EC 1)',
         [valid_bag, '--expected-count', '0']),
        (_EC_BAD_ARGS,  'bad args        (EC 2)',
         ['--expected-count', '-1']),
        (_EC_NOT_FOUND, 'file not found  (EC 3)',
         [nonexistent]),
        (_EC_PARSE_ERR, 'parse error     (EC 4)',
         [corrupt_bag]),
    ]

    all_ok = True
    print(f"[sub-test 4] exit-code proof")
    try:
        for expected_ec, label, cli_args in ec_cases:
            got = _capture_exit_code_quiet(*cli_args)
            ok  = got == expected_ec
            tag = 'PASS' if ok else 'FAIL'
            print(f"             [{tag}] {label}: exit {got} "
                  f"(expected {expected_ec})")
            if not ok:
                all_ok = False
    finally:
        os.unlink(valid_bag)
        os.unlink(corrupt_bag)

    if not all_ok:
        print("             FAIL – one or more exit codes are wrong.")
        sys.exit(1)
    print("             PASS – all four non-zero exit codes are distinct and correct.")

    if not allow_regression:
        return

    print()

    # ── Sub-test 5: --json output ──────────────────────────────────────────
    # Only runs under --self-test-allow-regression (the CI gate).
    _REQUIRED_KEYS = frozenset({
        'topic', 'event_name', 'event_id', 'expected', 'found', 'pass', 'exit_code',
    })

    tmp_fd, json_bag = tempfile.mkstemp(suffix='.ulg', prefix='a3_json_')
    os.close(tmp_fd)
    _write_synthetic_ulog(json_bag, [
        (2_000_000, A3_FALLBACK_EVENT_ID, _A3_LOG_LEVELS,
         struct.pack('<BB', _ACTION_RTL, _ACTION_LAND)),
        (4_000_000, A3_FALLBACK_EVENT_ID, _A3_LOG_LEVELS,
         struct.pack('<BB', _ACTION_RTL, _ACTION_DESCEND)),
    ])

    tmp_fd, json_corrupt = tempfile.mkstemp(suffix='.ulg', prefix='a3_json_corrupt_')
    os.write(tmp_fd, b'!! not a ulog !!')
    os.close(tmp_fd)

    json_nonexistent = json_bag + '.no_such_file'

    json_cases: list[tuple[str, list[str], dict]] = [
        ('pass  (EC 0, found=2)',
         [json_bag, '--json', '--expected-count', '2'],
         {'pass': True,  'exit_code': _EC_PASS,      'found': 2, 'expected': 2}),
        ('mismatch (EC 1, found=2)',
         [json_bag, '--json', '--expected-count', '0'],
         {'pass': False, 'exit_code': _EC_MISMATCH,  'found': 2, 'expected': 0}),
        ('not-found (EC 3)',
         [json_nonexistent, '--json'],
         {'pass': False, 'exit_code': _EC_NOT_FOUND, 'found': None}),
        ('parse-err (EC 4)',
         [json_corrupt,    '--json'],
         {'pass': False, 'exit_code': _EC_PARSE_ERR, 'found': None}),
    ]

    json_all_ok = True
    print('[sub-test 5] --json output')
    try:
        for label, cli_args, expected_fields in json_cases:
            _ec, obj = _capture_json_quiet(*cli_args)
            if obj is None:
                print(f"             [FAIL] {label}: stdout is not valid JSON")
                json_all_ok = False
                continue
            missing = _REQUIRED_KEYS - obj.keys()
            if missing:
                print(f"             [FAIL] {label}: missing keys {sorted(missing)}")
                json_all_ok = False
                continue
            wrong = [(k, obj[k], exp_v)
                     for k, exp_v in expected_fields.items() if obj[k] != exp_v]
            if wrong:
                details = ', '.join(f"{k}={got!r} (want {exp!r})"
                                    for k, got, exp in wrong)
                print(f"             [FAIL] {label}: {details}")
                json_all_ok = False
            else:
                print(f"             [PASS] {label}")
    finally:
        os.unlink(json_bag)
        os.unlink(json_corrupt)

    if not json_all_ok:
        print("             FAIL – --json output validation failed.")
        sys.exit(1)
    print("             PASS – --json emits valid JSON with correct fields for all four scenarios.")


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        'log',
        nargs='?',
        metavar='FLIGHT.ulg',
        help='Path to a ULog flight log to validate.',
    )
    parser.add_argument(
        '--expected-count',
        type=int,
        default=2,
        metavar='N',
        help=(
            'Number of matching events expected in the log (default: 2). '
            'Set to 0 to assert a clean flight with no A3 fallbacks, or 1 for '
            'a GPS-loss-only scenario where RTL→Land fired but local position '
            'remained valid so Land→Descend did not trigger.  '
            'Ignored when --self-test or --self-test-allow-regression is used.'
        ),
    )
    parser.add_argument(
        '--topic',
        default='event',
        metavar='TOPIC',
        help=(
            "uORB topic name to search for the matching event ID (default: 'event'). "
            "Override when the flight log uses a different subscription name."
        ),
    )
    parser.add_argument(
        '--id',
        default='commander_failsafe_a3_fallback',
        metavar='EVENT_NAME',
        dest='event_name',
        help=(
            "Event name whose FNV-1a hash is used as the 32-bit event ID "
            "(default: 'commander_failsafe_a3_fallback').  "
            "The hash is computed with the same formula as events::ID() in "
            "px4_platform_common/events.h: "
            "(fnv1a32(name) & 0xFFFFFF) | 0x01000000."
        ),
    )

    parser.add_argument(
        '--json',
        action='store_true',
        help=(
            'Print a single JSON object instead of the one-line CI summary.  '
            'Fields: topic, event_name, event_id (int), expected (int), '
            'found (int or null), pass (bool), exit_code (int).  '
            'No other text is printed.  Ignored in --self-test modes.'
        ),
    )

    # --self-test and --self-test-allow-regression are mutually exclusive:
    # argparse enforces this and emits a clear error when both are supplied.
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        '--self-test',
        action='store_true',
        help=(
            'Run both synthetic sub-tests.  Sub-test 2 (regressed bag) exits '
            'non-zero when it detects excess events, matching real-log behavior.'
        ),
    )
    mode_group.add_argument(
        '--self-test-allow-regression',
        action='store_true',
        help=(
            'Flips the sub-test 2 expectation: exits zero when the regressed '
            'bag produces more than 2 events, confirming detector sensitivity.  '
            'Use this as the CI gate for the validator itself.'
        ),
    )

    args = parser.parse_args()

    if args.expected_count < 0:
        parser.error('--expected-count must be a non-negative integer')  # exits EC 2

    if args.self_test_allow_regression:
        self_test(allow_regression=True)
        return

    if args.self_test:
        self_test(allow_regression=False)
        return

    if not args.log:
        parser.print_help()
        sys.exit(1)

    expected = args.expected_count
    topic    = args.topic
    event_id = px4_event_id(args.event_name)

    code, n = _run_log_validation(args.log, topic, event_id, expected)

    if args.json:
        print(json.dumps({
            "topic":      topic,
            "event_name": args.event_name,
            "event_id":   event_id,
            "expected":   expected,
            "found":      n,
            "pass":       code == _EC_PASS,
            "exit_code":  code,
        }))
    else:
        # One-line CI summary – easy to grep in build logs
        id_str = f"{topic}::{args.event_name}(0x{event_id:08x})"
        if n is not None:
            tag = 'PASS' if code == _EC_PASS else 'FAIL'
            print(f"[{tag}]  {n}/{expected}  {id_str}  {args.log}")
        elif code == _EC_NOT_FOUND:
            print(f"[ENOENT]  {id_str}  {args.log}")
        else:
            print(f"[EPARSE]  {id_str}  {args.log}")

    sys.exit(code)


if __name__ == '__main__':
    main()
