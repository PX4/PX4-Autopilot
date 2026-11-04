#!/usr/bin/env python3
"""
MAVSDK mission test for PX4 SIH SITL in Docker.

Takes off to 100m, flies a short 4-waypoint box mission, then lands.
Validates that the SIH Docker container works end-to-end with MAVSDK.

Prerequisites:
  - Docker container running:
      docker run --rm --network host px4io/px4-sitl:v1.17.0-alpha1
  - pip install -r Tools/setup/test-requirements.txt (MAVSDK v4)
  - mavsim-viewer running (optional):
      /path/to/mavsim-viewer -n 1

Usage:
  python3 Tools/packaging/containers/test_sih_mission.py
  python3 Tools/packaging/containers/test_sih_mission.py --speed 10   # faster-than-realtime
"""

import argparse
import math
import sys
import time

from mavsdk import ComponentType, Configuration, ConnectionResult, Mavsdk
from mavsdk.plugins.action import Action, ActionError
from mavsdk.plugins.mission import Mission, MissionItem, MissionPlan
from mavsdk.plugins.telemetry import Telemetry

CONNECTION_URL = "udpin://0.0.0.0:14540"


def make_mission_item(latitude_deg, longitude_deg):
    # All fields need to be set.
    nan = float("nan")
    return MissionItem(
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        relative_altitude_m=100,
        speed_m_s=10,
        is_fly_through=True,
        gimbal_pitch_deg=nan,
        gimbal_yaw_deg=nan,
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=nan,
        camera_photo_interval_s=nan,
        acceptance_radius_m=nan,
        yaw_deg=nan,
        camera_photo_distance_m=nan,
        vehicle_action=MissionItem.VehicleAction.NONE,
    )


def wait_for(condition, poll_s=0.1):
    while not condition():
        time.sleep(poll_s)


def run_mission(speed_factor):
    mavsdk = Mavsdk(Configuration.create_with_component_type(ComponentType.GROUND_STATION))

    print(f"Connecting to drone on {CONNECTION_URL} ...")
    result = mavsdk.add_any_connection(CONNECTION_URL)
    if result != ConnectionResult.SUCCESS:
        raise RuntimeError(f"Connection failed: {result.name}")

    print("Waiting for drone to connect...")
    drone = mavsdk.first_autopilot(timeout_s=-1)
    print(f"Connected (system ID: {drone.get_system_id()})")

    action = Action(drone)
    mission = Mission(drone)
    telemetry = Telemetry(drone)

    print("Waiting for global position estimate...")
    wait_for(lambda: telemetry.health().is_global_position_ok
             and telemetry.health().is_home_position_ok)
    print("Global position OK")

    # Get home position for reference
    wait_for(lambda: math.isfinite(telemetry.position().latitude_deg))
    position = telemetry.position()
    home_lat = position.latitude_deg
    home_lon = position.longitude_deg
    print(f"Home position: {home_lat:.6f}, {home_lon:.6f}")

    # Build a small box mission at 100m AGL
    # ~100m offset in each direction
    offset = 0.001  # roughly 111m at equator
    mission_items = [
        make_mission_item(home_lat + offset, home_lon),
        make_mission_item(home_lat + offset, home_lon + offset),
        make_mission_item(home_lat, home_lon + offset),
        make_mission_item(home_lat, home_lon),
    ]

    print(f"Uploading mission ({len(mission_items)} waypoints, 100m AGL)...")
    mission.upload_mission(MissionPlan(mission_items=mission_items))
    print("Mission uploaded")

    print("Arming...")
    action.arm()
    print("Armed")

    t0 = time.time()
    print("Starting mission...")
    mission.start_mission()

    # Monitor mission progress
    last_current = -1
    while True:
        progress = mission.mission_progress()
        elapsed = time.time() - t0
        if progress.current != last_current:
            print(f"  [{elapsed:6.1f}s] Waypoint {progress.current}/{progress.total}")
            last_current = progress.current
        if progress.total > 0 and progress.current == progress.total:
            print(f"Mission complete in {elapsed:.1f}s (speed factor: {speed_factor}x)")
            break
        time.sleep(0.1)

    print("Returning to launch...")
    action.return_to_launch()

    # Wait for landing
    wait_for(lambda: not telemetry.in_air())
    print("Landed")

    print("Disarming...")
    try:
        action.disarm()
    except ActionError:
        # PX4 usually disarms by itself after landing.
        if telemetry.armed():
            raise
    print("Test PASSED")

    mavsdk.destroy()


def main():
    parser = argparse.ArgumentParser(description="PX4 SIH Docker mission test")
    parser.add_argument("--speed", type=int, default=1,
                        help="PX4_SIM_SPEED_FACTOR (must match container)")
    args = parser.parse_args()

    try:
        run_mission(args.speed)
    except KeyboardInterrupt:
        print("\nInterrupted")
        sys.exit(1)
    except Exception as e:
        print(f"Test FAILED: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
