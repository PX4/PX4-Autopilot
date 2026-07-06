"""Mission protocol helpers: item generation, upload/download/compare/clear.

Mission protocol (verified in src/modules/mavlink/mavlink_mission.cpp):
  upload:   MISSION_COUNT -> per-item MISSION_REQUEST_INT / MISSION_REQUEST
            -> we reply MISSION_ITEM_INT -> final MISSION_ACK(ACCEPTED)
  download: MISSION_REQUEST_LIST -> MISSION_COUNT -> per-item
            MISSION_REQUEST_INT -> we reply, then send MISSION_ACK(ACCEPTED)
  clear:    MISSION_CLEAR_ALL -> MISSION_ACK(ACCEPTED)

The autopilot may request items with either MISSION_REQUEST_INT or the
deprecated MISSION_REQUEST; both are answered with MISSION_ITEM_INT, and
duplicate or out-of-order requests are answered as asked.
"""

import math
import time

from pymavlink import mavutil

MAV_CMD_NAV_WAYPOINT = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
MAV_FRAME = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
MAV_MISSION_TYPE_MISSION = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
MAV_MISSION_ACCEPTED = mavutil.mavlink.MAV_MISSION_ACCEPTED

BASE_LAT = 47.397742
BASE_LON = 8.545594

STEP_TIMEOUT_S = 5.0
TRANSACTION_DEADLINE_S = 60.0
COUNT_RETRANSMITS = 3

UPLOAD_TYPES = ['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK']
DOWNLOAD_COUNT_TYPES = ['MISSION_COUNT', 'MISSION_ACK']
DOWNLOAD_ITEM_TYPES = ['MISSION_ITEM_INT', 'MISSION_ITEM', 'MISSION_ACK']


def transaction_deadline(num_items):
    """Transaction budget scaled for large missions on slow links.

    220 items over a 57600 baud radio shared with telemetry streams can
    legitimately take more than a minute; 0.5s per item is still tight
    enough that a genuine stall (per-step 5s silence) fails much earlier.
    """
    return max(TRANSACTION_DEADLINE_S, 0.5 * num_items)


class Item:
    """A single generated waypoint; comparison uses the fields we round-trip."""

    def __init__(self, seq, lat_int, lon_int, alt, current):
        self.seq = seq
        self.command = MAV_CMD_NAV_WAYPOINT
        self.frame = MAV_FRAME
        self.current = current
        self.autocontinue = 1
        self.param1 = 0.0
        self.param2 = 2.0     # acceptance radius
        self.param3 = 0.0
        self.param4 = 0.0
        self.x = lat_int      # int31, 1e-7 deg
        self.y = lon_int
        self.z = alt


def generate_mission(iteration, k):
    """Deterministic mission of k items, keyed on (iteration, k)."""
    items = []
    for seq in range(k):
        lat_int = int((BASE_LAT + 0.0001 * seq) * 1e7) + iteration
        lon_int = int((BASE_LON + 0.0001 * seq) * 1e7) + iteration
        alt = 20.0 + (seq % 50)
        items.append(Item(seq, lat_int, lon_int, alt, 1 if seq == 0 else 0))
    return items


def send_item(mav, item):
    mav.mav.mission_item_int_send(
        mav.target_system, mav.target_component,
        item.seq, item.frame, item.command,
        item.current, item.autocontinue,
        item.param1, item.param2, item.param3, item.param4,
        item.x, item.y, item.z,
        MAV_MISSION_TYPE_MISSION)


def upload_mission(report, mav, items, iteration):
    """Run the upload handshake. Returns (ok, duration, detail)."""
    start = time.monotonic()
    deadline = start + transaction_deadline(len(items))

    mav.mav.mission_count_send(
        mav.target_system, mav.target_component,
        len(items), MAV_MISSION_TYPE_MISSION)

    count_retransmits = 0
    seq_reached = -1
    saw_any_request = False

    while time.monotonic() < deadline:
        m = mav.recv_match(type=UPLOAD_TYPES, blocking=True, timeout=STEP_TIMEOUT_S)

        if m is None:
            # Initial silence: our MISSION_COUNT may have been dropped; retry.
            if not saw_any_request and count_retransmits < COUNT_RETRANSMITS:
                count_retransmits += 1
                report.info('iter {}: no request, retransmitting MISSION_COUNT '
                            '({}/{})'.format(iteration, count_retransmits, COUNT_RETRANSMITS))
                mav.mav.mission_count_send(
                    mav.target_system, mav.target_component,
                    len(items), MAV_MISSION_TYPE_MISSION)
                continue
            return (False, time.monotonic() - start,
                    'upload stalled waiting for request/ack at seq {}'.format(seq_reached))

        t = m.get_type()

        if t in ('MISSION_REQUEST_INT', 'MISSION_REQUEST'):
            saw_any_request = True
            seq = m.seq
            # Respond to whatever seq is requested (handles dup / out-of-order).
            if 0 <= seq < len(items):
                send_item(mav, items[seq])
                seq_reached = max(seq_reached, seq)
            else:
                return (False, time.monotonic() - start,
                        'autopilot requested out-of-range seq {} (have {})'.format(
                            seq, len(items)))

        elif t == 'MISSION_ACK':
            ok = (m.type == MAV_MISSION_ACCEPTED)
            detail = 'ACK type {} at seq {}'.format(m.type, seq_reached)
            return ok, time.monotonic() - start, detail

    return (False, time.monotonic() - start,
            'upload transaction deadline {:.0f}s hit at seq {}'.format(
                transaction_deadline(len(items)), seq_reached))


def download_mission(mav):
    """Request and collect the mission. Returns (items_or_None, duration, detail)."""
    start = time.monotonic()
    deadline = start + TRANSACTION_DEADLINE_S

    mav.mav.mission_request_list_send(
        mav.target_system, mav.target_component, MAV_MISSION_TYPE_MISSION)

    count = None
    while time.monotonic() < deadline:
        m = mav.recv_match(type=DOWNLOAD_COUNT_TYPES, blocking=True, timeout=STEP_TIMEOUT_S)
        if m is None:
            return None, time.monotonic() - start, 'download stalled waiting for MISSION_COUNT'
        t = m.get_type()
        if t == 'MISSION_COUNT':
            count = m.count
            # extend the budget now that the item count is known
            deadline = start + transaction_deadline(count)
            break
        if t == 'MISSION_ACK':
            # An error ACK here means the request was rejected.
            return None, time.monotonic() - start, 'got MISSION_ACK type {} instead of COUNT'.format(m.type)

    if count is None:
        return None, time.monotonic() - start, 'download stalled before MISSION_COUNT'

    items = {}
    next_seq = 0
    while next_seq < count and time.monotonic() < deadline:
        mav.mav.mission_request_int_send(
            mav.target_system, mav.target_component, next_seq, MAV_MISSION_TYPE_MISSION)
        m = mav.recv_match(type=DOWNLOAD_ITEM_TYPES, blocking=True, timeout=STEP_TIMEOUT_S)
        if m is None:
            return None, time.monotonic() - start, \
                'download stalled waiting for item seq {} of {}'.format(next_seq, count)
        t = m.get_type()
        if t in ('MISSION_ITEM_INT', 'MISSION_ITEM'):
            items[m.seq] = m
            if m.seq == next_seq:
                next_seq += 1
        elif t == 'MISSION_ACK':
            return None, time.monotonic() - start, \
                'unexpected MISSION_ACK type {} at seq {}'.format(m.type, next_seq)

    if len(items) < count:
        return None, time.monotonic() - start, \
            'download incomplete: got {}/{} items'.format(len(items), count)

    # Terminate the download transaction with our own ACK.
    mav.mav.mission_ack_send(
        mav.target_system, mav.target_component,
        MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION)

    ordered = [items[s] for s in range(count)]
    return ordered, time.monotonic() - start, 'downloaded {} items'.format(count)


def compare_mission(expected, downloaded):
    """Compare item-by-item. Returns (ok, detail with first differing seq)."""
    if len(downloaded) != len(expected):
        return False, 'count mismatch: expected {} got {}'.format(
            len(expected), len(downloaded))

    for exp, got in zip(expected, downloaded):
        if got.seq != exp.seq:
            return False, 'seq {}: seq field mismatch (got {})'.format(exp.seq, got.seq)
        if got.command != exp.command:
            return False, 'seq {}: command {} != {}'.format(exp.seq, got.command, exp.command)
        if got.frame != exp.frame:
            return False, 'seq {}: frame {} != {}'.format(exp.seq, got.frame, exp.frame)
        if got.x != exp.x:
            return False, 'seq {}: x {} != {}'.format(exp.seq, got.x, exp.x)
        if got.y != exp.y:
            return False, 'seq {}: y {} != {}'.format(exp.seq, got.y, exp.y)
        if not math.isclose(got.z, exp.z, abs_tol=1e-3):
            return False, 'seq {}: z {} != {} (tol 1e-3)'.format(exp.seq, got.z, exp.z)

    return True, 'all {} items match'.format(len(expected))


def clear_mission(mav):
    """MISSION_CLEAR_ALL -> MISSION_ACK(ACCEPTED). Returns (ok, detail)."""
    deadline = time.monotonic() + TRANSACTION_DEADLINE_S
    mav.mav.mission_clear_all_send(
        mav.target_system, mav.target_component, MAV_MISSION_TYPE_MISSION)
    while time.monotonic() < deadline:
        m = mav.recv_match(type='MISSION_ACK', blocking=True, timeout=STEP_TIMEOUT_S)
        if m is None:
            return False, 'clear stalled waiting for MISSION_ACK'
        if m.type == MAV_MISSION_ACCEPTED:
            return True, 'cleared'
        return False, 'clear rejected: MISSION_ACK type {}'.format(m.type)
    return False, 'clear transaction deadline hit'


def verify_cleared(mav):
    """After clear, MISSION_REQUEST_LIST should yield MISSION_COUNT == 0."""
    deadline = time.monotonic() + TRANSACTION_DEADLINE_S
    mav.mav.mission_request_list_send(
        mav.target_system, mav.target_component, MAV_MISSION_TYPE_MISSION)
    while time.monotonic() < deadline:
        m = mav.recv_match(type=DOWNLOAD_COUNT_TYPES, blocking=True, timeout=STEP_TIMEOUT_S)
        if m is None:
            return False, 'verify-clear stalled waiting for MISSION_COUNT'
        t = m.get_type()
        if t == 'MISSION_COUNT':
            # Terminate the (empty) list transaction.
            mav.mav.mission_ack_send(
                mav.target_system, mav.target_component,
                MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION)
            if m.count == 0:
                return True, 'count 0 after clear'
            return False, 'count {} after clear (expected 0)'.format(m.count)
        if t == 'MISSION_ACK':
            # Empty mission may be reported via ACK; treat non-error as cleared.
            if m.type == MAV_MISSION_ACCEPTED:
                return True, 'empty mission (ACK) after clear'
            return False, 'verify-clear got MISSION_ACK type {}'.format(m.type)
    return False, 'verify-clear transaction deadline hit'
