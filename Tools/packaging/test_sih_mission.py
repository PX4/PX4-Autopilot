#!/usr/bin/env python3
"""
MAVSDK mission test for PX4 SIH SITL in Docker.

Takes off to 100m, flies a short 4-waypoint box mission, then lands.
Validates that the SIH Docker container works end-to-end with MAVSDK.

Prerequisites:
  - Docker container running:
      docker run --rm --network host px4io/px4-sitl:v1.17.0-alpha1
  - pip install mavsdk
  - mavsim-viewer running (optional):
      /path/to/mavsim-viewer -n 1

Usage:
  python3 Tools/packaging/test_sih_mission.py
  python3 Tools/packaging/test_sih_mission.py --speed 10   # faster-than-realtime
"""

import asyncio
import argparse
import sys
import time

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan


async def run_mission(speed_factor: int = 1):
    drone = System()
    print(f"Connecting to drone on udp://:14540 ...")
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Connected (UUID: {state.uuid if hasattr(state, 'uuid') else 'N/A'})")
            break

    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position OK")
            break

    # Get home position for reference
    async for pos in drone.telemetry.position():
        home_lat = pos.latitude_deg
        home_lon = pos.longitude_deg
        print(f"Home position: {home_lat:.6f}, {home_lon:.6f}")
        break

    # Build a small box mission at 100m AGL
    # ~100m offset in each direction
    offset = 0.001  # roughly 111m at equator
    mission_items = [
        MissionItem(
            home_lat + offset, home_lon,
            100, 10, True, float('nan'), float('nan'),
            MissionItem.CameraAction.NONE,
            float('nan'), float('nan'), float('nan'),
            float('nan'), float('nan'),
            MissionItem.VehicleAction.NONE,
        ),
        MissionItem(
            home_lat + offset, home_lon + offset,
            100, 10, True, float('nan'), float('nan'),
            MissionItem.CameraAction.NONE,
            float('nan'), float('nan'), float('nan'),
            float('nan'), float('nan'),
            MissionItem.VehicleAction.NONE,
        ),
        MissionItem(
            home_lat, home_lon + offset,
            100, 10, True, float('nan'), float('nan'),
            MissionItem.CameraAction.NONE,
            float('nan'), float('nan'), float('nan'),
            float('nan'), float('nan'),
            MissionItem.VehicleAction.NONE,
        ),
        MissionItem(
            home_lat, home_lon,
            100, 10, True, float('nan'), float('nan'),
            MissionItem.CameraAction.NONE,
            float('nan'), float('nan'), float('nan'),
            float('nan'), float('nan'),
            MissionItem.VehicleAction.NONE,
        ),
    ]

    mission_plan = MissionPlan(mission_items)

    print(f"Uploading mission ({len(mission_items)} waypoints, 100m AGL)...")
    await drone.mission.upload_mission(mission_plan)
    print("Mission uploaded")

    print("Arming...")
    await drone.action.arm()
    print("Armed")

    t0 = time.time()
    print("Starting mission...")
    await drone.mission.start_mission()

    # Monitor mission progress
    async for progress in drone.mission.mission_progress():
        elapsed = time.time() - t0
        print(f"  [{elapsed:6.1f}s] Waypoint {progress.current}/{progress.total}")
        if progress.current == progress.total:
            print(f"Mission complete in {elapsed:.1f}s (speed factor: {speed_factor}x)")
            break

    print("Returning to launch...")
    await drone.action.return_to_launch()

    # Wait for landing
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Landed")
            break

    print("Disarming...")
    await drone.action.disarm()
    print("Test PASSED")


def main():
    parser = argparse.ArgumentParser(description="PX4 SIH Docker mission test")
    parser.add_argument("--speed", type=int, default=1,
                        help="PX4_SIM_SPEED_FACTOR (must match container)")
    args = parser.parse_args()

    try:
        asyncio.run(run_mission(args.speed))
    except KeyboardInterrupt:
        print("\nInterrupted")
        sys.exit(1)
    except Exception as e:
        print(f"Test FAILED: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
