#!/usr/bin/env python3
"""Bench-only RID bridge test; run against freshly booted PX4.

Requires pymavlink, plus dronecan for --can. This script replaces QGC and
emulates a RID module's FAIL status; it does not validate a real module or RF.
Use two independent PX4 MAVLink ports for --gcs and --rid, or --gcs and --can.
Enable normal/onboard streams and forwarding on MAVLink ports beforehand.
Do not run concurrently with QGC. No arming or parameter commands are sent.

--identity-mode must match ODID_BID_SRC (legacy=0, locked=1); this script does
not change the parameter. Locked mode without --expect-identity assumes blank
identity storage and provisions PRODUCT below. It never erases identity storage.
After manually rebooting, rerun with --identity-mode locked --expect-identity
ABCDF123456789ABCDEF to verify the stored identity before sending any Basic ID.
Use --expect-ua-type if that identity was provisioned with a UA type other than 2.
"""
import argparse
import os
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil  # noqa: E402

NAMES = {12900: 'BasicID', 12904: 'System', 12905: 'OperatorID'}
ZERO_ID = bytes(20)
PRODUCT = b'ABCDF123456789ABCDEF'
ALTERNATE_PRODUCT = b'WXYZF123456789ABCDEF'
INVALID_PRODUCT = b'ABCD1I' + bytes(14)
OPERATOR = b'00001234' + bytes(12)
UNIX_2019 = 1546300800
SYSTEM_FRESHNESS_SECONDS = 5
SYSTEM_REFRESH_SECONDS = 0.8


def system_time():
    return int(time.time()) - UNIX_2019


def serial_number(text):
    """Parse a CTA-2063 serial supplied for a previously provisioned board."""
    allowed = '0123456789ABCDEFGHJKLMNPQRSTUVWXYZ'
    if (not 6 <= len(text) <= 20 or any(char not in allowed for char in text)
            or text[4] not in '123456789ABCDEF'
            or int(text[4], 16) != len(text) - 5):
        raise argparse.ArgumentTypeError('expected a valid 6-20 character CTA-2063 serial number')
    return text.encode('ascii').ljust(20, b'\0')


def basic_identity(product, ua_type=2):
    return dict(id_or_mac=ZERO_ID, id_type=1, ua_type=ua_type, uas_id=product)


def octets(value):
    if isinstance(value, str):
        return value.encode('ascii')
    return bytes(value)


class Bench:
    def __init__(self, args):
        self.args = args
        self.gcs = mavutil.mavlink_connection(
            args.gcs, baud=args.baud, source_system=args.source_system,
            source_component=args.source_component, dialect='common')
        self.received = []
        self.echoes = []
        self.statuses = []
        self.locations = 0
        self.last_heartbeat = 0
        self.last_system_refresh = 0
        self.system_fields = None
        self.refresh_system = True
        self.system_timestamps = set()
        self.accepted_inputs = set()
        self.source_type = mavutil.mavlink.MAV_TYPE_GCS
        self.can = None
        self.rid = None
        if args.can:
            import dronecan
            self.dronecan = dronecan
            self.can = dronecan.make_node(args.can, node_id=args.node_id,
                                         bitrate=args.can_bitrate)
            for name in ('BasicID', 'System', 'OperatorID', 'Location'):
                dtype = getattr(dronecan.dronecan.remoteid, name)
                self.can.add_handler(dtype, self.can_message)
        else:
            self.rid = mavutil.mavlink_connection(
                args.rid, baud=args.baud, source_system=args.target_system,
                source_component=mavutil.mavlink.MAV_COMP_ID_ODID_TXRX_1,
                dialect='common')

    def can_message(self, event):
        if event.transfer.source_node_id != self.args.px4_node_id:
            return
        message = event.message
        name = str(message._type.full_name).split('.')[-1]
        if name == 'Location':
            self.locations += 1
        else:
            self.received.append((time.monotonic(), name, message))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            now = time.monotonic()
            if now - self.last_heartbeat >= 0.8:
                self.gcs.mav.heartbeat_send(self.source_type, 8, 0, 0, 0)
                if self.rid:
                    self.rid.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ODID, 8, 0, 0, 0)
                    self.rid.mav.open_drone_id_arm_status_send(1, b'RID bench failure')
                else:
                    status = self.dronecan.dronecan.remoteid.ArmStatus()
                    status.status = 1
                    status.error = b'RID bench failure'
                    self.can.broadcast(status)
                self.last_heartbeat = now
            if (self.refresh_system and self.system_fields is not None
                    and now - self.last_system_refresh >= SYSTEM_REFRESH_SECONDS):
                self.send_system(self.system_fields)
            if self.can:
                self.can.spin(0.01)
            if self.rid:
                for _ in range(100):
                    message = self.rid.recv_match(blocking=False)
                    if message is None:
                        break
                    if message.get_srcSystem() != self.args.target_system:
                        continue
                    if message.get_msgId() in NAMES:
                        self.received.append((now, NAMES[message.get_msgId()], message))
                        assert message.target_system == 0 and message.target_component == 0
                        assert message.get_srcComponent() == self.args.target_component
                    elif message.get_msgId() == 12901:
                        self.locations += 1
            for _ in range(100):
                message = self.gcs.recv_match(blocking=False)
                if message is None:
                    break
                if message.get_srcSystem() != self.args.target_system:
                    continue
                if message.get_msgId() in NAMES:
                    self.echoes.append(NAMES[message.get_msgId()])
                elif message.get_msgId() == 12918:
                    self.statuses.append(message)
            time.sleep(0.005)

    def send_system(self, fields):
        fields = dict(fields, timestamp=system_time())
        self.gcs.mav.open_drone_id_system_send(**fields)
        self.system_timestamps.add(fields['timestamp'])
        self.last_system_refresh = time.monotonic()

    def system(self, classification=2, ua_class=1, refresh=True, **overrides):
        fields = dict(target_system=self.args.target_system,
                      target_component=self.args.target_component, id_or_mac=ZERO_ID,
                      operator_location_type=2, classification_type=classification,
                      operator_latitude=312345678, operator_longitude=1212345678,
                      area_count=3, area_radius=42, area_ceiling=123.5,
                      area_floor=-1000., category_eu=1, class_eu=ua_class,
                      operator_altitude_geo=88.25)
        fields.update(overrides)
        self.send_system(fields)
        if refresh:
            self.system_fields = fields
            self.accepted_inputs.add('System')
        # None means a live timestamp from one of this bench's SYSTEM samples.
        return dict({key: value for key, value in fields.items() if not key.startswith('target_')},
                    timestamp=None)

    def basic_id(self, product, ua_type=2, accepted=True):
        self.gcs.mav.open_drone_id_basic_id_send(
            self.args.target_system, self.args.target_component, ZERO_ID, 1, ua_type, product)
        if accepted:
            self.accepted_inputs.add('BasicID')
        return basic_identity(product, ua_type)

    def identities(self, product=PRODUCT):
        self.gcs.mav.open_drone_id_operator_id_send(
            self.args.target_system, self.args.target_component, ZERO_ID, 1, OPERATOR)
        self.accepted_inputs.add('OperatorID')
        return {'OperatorID': dict(id_or_mac=ZERO_ID, operator_id_type=1, operator_id=OPERATOR),
                'BasicID': self.basic_id(product)}

    def check(self, expected, label, forbidden=(), settle=1.2, duration=3.4):
        forbidden = set(forbidden)
        if 'OperatorID' not in expected:
            forbidden.add('OperatorID')
        # Allow a queued prior sample, then check several complete output periods.
        self.spin(settle)
        for _, name, _ in self.received:
            assert name not in forbidden, (label, 'unexpected output before observation', name)
        self.received.clear()
        self.echoes.clear()
        self.statuses.clear()
        self.spin(duration)
        reception_times = {name: [] for name in expected}
        for received_at, name, message in self.received:
            assert name not in forbidden, (label, 'unexpected output', name)
            if name not in expected:
                continue
            reception_times[name].append(received_at)
            for field, value in expected[name].items():
                actual = getattr(message, field)
                if name == 'System' and field == 'timestamp' and value is None:
                    assert actual in self.system_timestamps, (label, 'unknown SYSTEM timestamp', actual)
                    received_utc = time.time() - (time.monotonic() - received_at)
                    assert 0 <= int(received_utc) - UNIX_2019 - actual <= SYSTEM_FRESHNESS_SECONDS, (
                        label, 'stale SYSTEM timestamp', actual)
                    continue
                if isinstance(value, bytes):
                    if field == 'operator_id' and self.rid:
                        # pymavlink exposes char[] as a string without trailing NULs.
                        actual = getattr(message, '_operator_id_raw', actual)
                        actual = octets(actual).ljust(20, b'\0')
                    else:
                        actual = octets(actual)
                assert actual == value, (label, name, field, actual, value)
        counts = {name: len(times) for name, times in reception_times.items()}
        assert all(count >= 2 for count in counts.values()), (label, counts)
        # A duplicate burst cannot prove periodic output: samples must span most
        # of the observation window, allowing one 1 Hz period at either edge.
        for name, times in reception_times.items():
            assert times[-1] - times[0] >= duration - 1.5, (label, name, 'output stopped or burst only', times)
        assert not self.accepted_inputs.intersection(self.echoes), (label, 'input echo', self.echoes)
        assert self.statuses, (label, 'ARM_STATUS missing')
        for status in self.statuses:
            assert status.status == 1
            assert octets(status.error).rstrip(b'\0') == b'RID bench failure'
        print('PASS', label, counts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gcs', required=True, help='Host endpoint of the PX4 GCS link')
    transport = parser.add_mutually_exclusive_group(required=True)
    transport.add_argument('--rid', help='Host endpoint of a second PX4 MAVLink link')
    transport.add_argument('--can', help='DroneCAN adapter, e.g. can0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--target-system', required=True, type=int)
    parser.add_argument('--target-component', type=int, default=1)
    parser.add_argument('--source-system', type=int, default=245)
    parser.add_argument('--source-component', type=int, default=190)
    parser.add_argument('--node-id', type=int, default=100, help='Unused bench adapter node ID')
    parser.add_argument('--px4-node-id', type=int, default=1)
    parser.add_argument('--can-bitrate', type=int, default=1000000)
    parser.add_argument('--identity-mode', choices=('legacy', 'locked'), default='legacy',
                        help='Must match ODID_BID_SRC: legacy=0 (default), locked=1; parameter is not changed')
    parser.add_argument('--expect-identity', type=serial_number,
                        help='Locked mode: verify this stored serial before any Basic ID write, e.g. after a manual reboot')
    parser.add_argument('--expect-ua-type', type=int, default=2,
                        help='UA type of --expect-identity (default: 2)')
    parser.add_argument('--test-stale-system', action='store_true',
                        help='Pause SYSTEM refresh and verify unknown dynamic fields after 5 seconds')
    parser.add_argument('--require-location', action='store_true',
                        help='Require LOCATION when valid flight/GNSS data is available')
    args = parser.parse_args()
    if args.expect_identity is not None and args.identity_mode != 'locked':
        parser.error('--expect-identity requires --identity-mode locked')
    if not 0 <= args.expect_ua_type <= 15:
        parser.error('--expect-ua-type must be in 0..15')
    if args.expect_identity is None and args.expect_ua_type != 2:
        parser.error('--expect-ua-type requires --expect-identity')

    bench = Bench(args)
    bench.spin(2)
    expected = {'System': bench.system()}

    if args.identity_mode == 'locked':
        if args.expect_identity is not None:
            expected['BasicID'] = basic_identity(args.expect_identity, args.expect_ua_type)
            bench.check(expected, 'stored identity after boot, before Basic ID input')
        else:
            print('Locked provisioning assumes blank identity storage; no erase is performed.')
            bench.check(expected, 'unprovisioned locked mode has no GUID fallback', forbidden=('BasicID',))
            for invalid, label in ((ZERO_ID, 'empty'), (INVALID_PRODUCT, 'invalid character')):
                bench.basic_id(invalid, accepted=False)
                bench.check(expected, f'{label} cannot provision locked identity', forbidden=('BasicID',))
    else:
        bench.check(expected, 'SYSTEM without identities')

    identities = bench.identities()  # SYSTEM -> OPERATOR_ID -> BASIC_ID
    if args.expect_identity is not None:
        identities['BasicID'] = basic_identity(args.expect_identity, args.expect_ua_type)
    expected.update(identities)
    bench.check(expected, 'out-of-order identity and persistent output')

    if args.identity_mode == 'locked':
        bench.basic_id(INVALID_PRODUCT, accepted=False)
        bench.check(expected, 'invalid input cannot change locked identity')

    expected['System'] = bench.system(ua_class=0)
    bench.check(expected, 'CN micro')
    expected['System'] = bench.system(classification=1, ua_class=1)
    bench.check(expected, 'EU regression')
    locked_identity = expected['BasicID']
    expected.update(bench.identities(ALTERNATE_PRODUCT))
    if args.identity_mode == 'locked':
        expected['BasicID'] = locked_identity
    expected['System'] = bench.system(ua_class=1)
    bench.check(expected, 'alternate valid identity cannot replace locked identity'
                if args.identity_mode == 'locked' else 'updated configuration persists')
    expected['System'] = bench.system(target_component=0)
    bench.check(expected, 'component broadcast uses stream only')
    # Pause valid refresh while probing rejection so a later valid packet cannot
    # overwrite an incorrectly accepted injection before the assertion sees it.
    bench.send_system(bench.system_fields)
    bench.refresh_system = False
    bench.system(refresh=False, target_system=(args.target_system % 255) + 1, class_eu=4)
    bench.check(expected, 'wrong vehicle cannot replace cache', settle=0.2)
    bench.refresh_system = True
    # Exact autopilot target prevents generic routing of the surrounding-aircraft sample.
    bench.send_system(bench.system_fields)
    bench.refresh_system = False
    bench.system(refresh=False, id_or_mac=b'X' + bytes(19), class_eu=4)
    bench.check(expected, 'surrounding aircraft cannot replace cache', settle=0.2)
    bench.refresh_system = True

    bench.send_system(bench.system_fields)
    bench.spin(0.2)
    bench.refresh_system = False
    bench.source_type = mavutil.mavlink.MAV_TYPE_ADSB
    bench.spin(0.9)
    bench.system(refresh=False, class_eu=4)
    bench.check(expected, 'non-GCS source cannot replace cache', settle=0.1, duration=2.6)
    bench.source_type = mavutil.mavlink.MAV_TYPE_GCS
    # Wait for the GCS heartbeat before resuming valid configuration refresh.
    bench.spin(0.9)
    bench.refresh_system = True

    if args.test_stale_system:
        bench.send_system(bench.system_fields)
        bench.spin(0.2)
        bench.refresh_system = False
        bench.spin(SYSTEM_FRESHNESS_SECONDS + 1)
        expected['System'] = dict(expected['System'], operator_latitude=0, operator_longitude=0,
                                  operator_altitude_geo=-1000., timestamp=0)
        bench.check(expected, 'stale SYSTEM keeps classification and marks location unknown')

    if args.require_location:
        assert bench.locations > 0, 'LOCATION missing'
    if args.identity_mode == 'locked' and args.expect_identity is None:
        print('To verify persistence, reboot manually and rerun with --identity-mode locked '
              '--expect-identity ABCDF123456789ABCDEF using the same connection options.')
    print('Bench checks complete; real module behavior and RF remain unverified.')


if __name__ == '__main__':
    main()
