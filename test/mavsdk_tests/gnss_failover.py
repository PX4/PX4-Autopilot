#!/usr/bin/env python3
"""gnss_failover: injects one GNSS failure from a companion computer while a pilot flies, and
records what the vehicle does. It never arms or changes the mode, refuses to inject unless the
vehicle is flying in a position-controlled mode with both receivers at a 3D fix, and ends the
injection on exit and on SIGINT or SIGTERM.
"""

import argparse
import signal
import sys
import threading
from types import FrameType
from typing import Optional, TextIO

from mavsdk import ComponentType, Configuration, ConnectionResult, Mavsdk  # type: ignore
from mavsdk.plugins.failure import FailureType  # type: ignore
from mavsdk.plugins.telemetry import FlightMode  # type: ignore

from gnss_failover_scenario import RECEIVER_MESSAGES, GnssFailover, Injection, WrongPayload

FAILURE_TYPES = {
    "off": FailureType.OFF,
    "wrong": FailureType.WRONG,
    "slow": FailureType.SLOW,
    "stuck": FailureType.STUCK,
}

stop_requested = threading.Event()


def handle_signal(_signal: int, _frame: Optional[FrameType]) -> None:
    stop_requested.set()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inject one GNSS failure from a companion computer while a pilot flies.")
    parser.add_argument("--url", default="udpin://0.0.0.0:14540",
                        help="MAVLink connection (default udpin://0.0.0.0:14540)")
    parser.add_argument("--instance", type=int, required=True,
                        help="receiver to fail, 1-based (sensor_gnss instance + 1)")
    parser.add_argument("--type", choices=sorted(FAILURE_TYPES), required=True)
    parser.add_argument("--duration", type=float, default=20.0,
                        help="seconds of vehicle time to keep the failure (default 20)")
    parser.add_argument("--observe", type=float, default=20.0,
                        help="seconds to keep recording after the failure ends (default 20)")
    parser.add_argument("--record", default="",
                        help="CSV record: wall time [us], vehicle time [us], what (default stdout)")
    parser.add_argument("--ground", action="store_true",
                        help="allow injecting on the ground or disarmed (bench test, props off)")
    parser.add_argument("--fix", type=int, default=2,
                        help="wrong: SYS_FAIL_GPS_WRG, 0 = unchanged (default 2)")
    parser.add_argument("--eph", type=float, default=0.0, help="wrong: SYS_FAIL_GPS_EPH [m]")
    parser.add_argument("--epv", type=float, default=0.0, help="wrong: SYS_FAIL_GPS_EPV [m]")
    parser.add_argument("--sacc", type=float, default=0.0, help="wrong: SYS_FAIL_GPS_SAC [m/s]")
    parser.add_argument("--sats", type=int, default=0, help="wrong: SYS_FAIL_GPS_SAT")
    parser.add_argument("--jamming", type=int, default=0, help="wrong: SYS_FAIL_GPS_JAM")
    parser.add_argument("--spoofing", type=int, default=0, help="wrong: SYS_FAIL_GPS_SPF")
    parser.add_argument("--divider", type=int, default=4, help="slow: SYS_FAIL_GPS_DIV (default 4)")
    args = parser.parse_args()

    # One receiver at a time: failing all of them is a total loss, not a failover
    if args.instance < 1:
        parser.error("--instance must be 1 or more")

    return args


def record_receivers(gnss: GnssFailover) -> None:
    now_us = gnss.vehicle_time_us()
    texts = []

    for i, name in enumerate(RECEIVER_MESSAGES):
        receiver = gnss.receiver(i)
        texts.append("{} fix {} sats {} eph {:g} age {} ms".format(
            name, receiver.fix_type, receiver.satellites, receiver.eph,
            (now_us - receiver.vehicle_time_us) // 1000))

    gnss.record(" ".join(texts))


def run_for(gnss: GnssFailover, seconds: float) -> None:
    end_us = gnss.vehicle_time_us() + int(seconds * 1e6)

    while not stop_requested.is_set() and gnss.vehicle_time_us() < end_us:
        record_receivers(gnss)
        gnss.wait_until(stop_requested.is_set, 1.0)


def run(args: argparse.Namespace, record: TextIO, gnss: GnssFailover) -> int:
    injection = Injection(
        failure_type=FAILURE_TYPES[args.type], instance=args.instance,
        wrong=WrongPayload(fix_type=args.fix, eph=args.eph, epv=args.epv,
                           speed_accuracy=args.sacc, satellites=args.sats,
                           jamming_state=args.jamming, spoofing_state=args.spoofing),
        slow_divider=args.divider)

    gnss.set_record(record)

    if not gnss.request_streams():
        print("No ODOMETRY from the vehicle", file=sys.stderr)
        return 1

    # Both receivers' messages, at their requested rate
    gnss.sleep(2.0)

    refusal = gnss.injection_refusal(not args.ground)

    if refusal:
        print("Not injecting: {}".format(refusal), file=sys.stderr)
        gnss.record("refused: {}".format(refusal))
        return 1

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    start_us = gnss.vehicle_time_us()
    resets_before = gnss.reset_count()
    injected = gnss.inject(injection)

    if injected:
        run_for(gnss, args.duration)

    # The failure stays latched in the vehicle until cleared, so clear it however the run ends
    cleared = gnss.clear(injection.instance)
    restored = gnss.restore_parameters()

    if not stop_requested.is_set():
        run_for(gnss, args.observe)

    summary = ("summary: injected {}, cleared {}, parameters restored {}, estimator resets {}, "
               "position ok throughout {}, modes {}".format(
                   "yes" if injected else "no", "yes" if cleared else "no",
                   "yes" if restored else "no", gnss.reset_count() - resets_before,
                   "yes" if gnss.position_ok_since(start_us) else "no",
                   " ".join(FlightMode(mode).name for mode in gnss.modes_since(start_us))))
    gnss.record(summary)
    print(summary, file=sys.stderr)

    return 0 if injected and cleared and restored else 1


def main() -> int:
    args = parse_args()

    try:
        record = open(args.record, "a") if args.record else sys.stdout
    except OSError as e:
        print("Cannot open {}: {}".format(args.record, e), file=sys.stderr)
        return 1

    mavsdk = Mavsdk(Configuration.create_with_component_type(ComponentType.COMPANION_COMPUTER))

    try:
        if mavsdk.add_any_connection(args.url) != ConnectionResult.SUCCESS:
            print("Cannot connect to {}".format(args.url), file=sys.stderr)
            return 1

        system = mavsdk.first_autopilot(timeout_s=10.0)

        if system is None:
            print("No autopilot on {}".format(args.url), file=sys.stderr)
            return 1

        return run(args, record, GnssFailover(system))

    finally:
        mavsdk.destroy()
        if record is not sys.stdout:
            record.close()


if __name__ == "__main__":
    sys.exit(main())
