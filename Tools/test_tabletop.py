#!/usr/bin/env python3
"""
test_tabletop.py — Test motor tren ban bang van toc
====================================================
Workflow:
  1. Chay script -> script cho
  2. QGC: arm + chuyen ALTITUDE mode
  3. Script tu detect -> chay tung bai test 10s
  4. Quan sat / nghe motor tung bai

Chay:
  python3 test_tabletop.py
  python3 test_tabletop.py --connection serial:///dev/ttyACM0:57600
"""

import asyncio
import sys
import os
import math
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from px4_inverse_rc import position_mode_to_mavlink

from mavsdk import System
from mavsdk.manual_control import ManualControlError

CONNECTION    = "serial:///dev/ttyACM0:57600"
RATE_HZ       = 20
INTERVAL      = 1.0 / RATE_HZ
TEST_DURATION = 5.0   # s -- ngan de motor khong qua nhiet
THR_IDLE      = 0.15  # throttle idle -- du motor quay, khong nhat drone

# Gia tri van toc test (body frame)
VX_TEST  = 2.0   # m/s tien/lui theo mui drone
VY_TEST  = 2.0   # m/s phai/trai theo than drone
VZ_TEST  = 0.5   # m/s len/xuong (NED)
YAW_TEST = 20.0  # deg/s xoay


async def read_params(drone):
    defaults = dict(vel_max=10.0, vel_max_up=3.0, vel_max_dn=1.5,
                    yaw_max=150.0, deadzone=0.0)
    PMAP = {
        'MPC_VEL_MANUAL':   'vel_max',
        'MPC_Z_VEL_MAX_UP': 'vel_max_up',
        'MPC_Z_VEL_MAX_DN': 'vel_max_dn',
        'MPC_MAN_Y_MAX':    'yaw_max',
        'MAN_DEADZONE':     'deadzone',
    }
    print("Doc param tu drone...")
    for px4_name, key in PMAP.items():
        try:
            defaults[key] = await drone.param.get_param_float(px4_name)
            print("  {} = {}".format(px4_name, defaults[key]))
        except Exception:
            print("  {} = {} (default)".format(px4_name, defaults[key]))
    print()
    return defaults


def to_input(vx=0.0, vy=0.0, vz=0.0, yaw_dps=0.0, p=None):
    """Van toc body frame -> MAVSDK input.
    vx: tien(+)/lui(-) theo mui drone
    vy: phai(+)/trai(-) theo than drone
    vz: NED -- xuong(+)/len(-)
    """
    p = p or {}
    x, y, z, r = position_mode_to_mavlink(
        vx=vx, vy=vy, vz=vz, yaw_rate_dps=yaw_dps,
        vel_max=p.get('vel_max', 10.0),
        vel_max_up=p.get('vel_max_up', 3.0),
        vel_max_dn=p.get('vel_max_dn', 1.5),
        yaw_max=p.get('yaw_max', 150.0),
        deadzone=p.get('deadzone', 0.0),
    )
    # Mapping da xac nhan tu thuc nghiem POSCTL:
    # pitch = +x/1000, roll = +y/1000
    return +x/1000.0, +y/1000.0, z/1000.0, r/1000.0


async def send(drone, pitch, roll, thr, yaw, duration):
    steps = int(duration * RATE_HZ)
    for i in range(steps):
        await drone.manual_control.set_manual_control_input(pitch, roll, thr, yaw)
        await asyncio.sleep(INTERVAL)
        remaining = duration - i * INTERVAL
        if i % (RATE_HZ * 2) == 0 and remaining > 0:
            print("  ... {:.0f}s con lai".format(remaining))


async def hover(drone, duration=2.0):
    steps = int(duration * RATE_HZ)
    for _ in range(steps):
        await drone.manual_control.set_manual_control_input(0.0, 0.0, THR_IDLE, 0.0)
        await asyncio.sleep(INTERVAL)


async def run(connection):
    drone = System()
    print("Ket noi {}...".format(connection))
    await drone.connect(system_address=connection)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Ket noi OK!\n")
            break

    current_mode = {"val": "UNKNOWN"}
    async def watch_mode():
        async for m in drone.telemetry.flight_mode():
            s = str(m)
            if s != current_mode["val"]:
                print("  [Mode] {} -> {}".format(current_mode["val"], s))
                current_mode["val"] = s
    asyncio.ensure_future(watch_mode())
    await asyncio.sleep(1.0)

    params = await read_params(drone)

    print("=" * 60)
    print("Dung QGC: arm -> chuyen ALTITUDE mode")
    print("Script tu bat dau khi detect ALTCTL")
    print("=" * 60)
    while True:
        if "ALT" in current_mode["val"].upper():
            print("\nPhat hien {} -> bat dau!".format(current_mode["val"]))
            break
        await asyncio.sleep(0.3)

    print("\n[WARMUP] Nha joystick SIYI -- gui hover 3s...")
    try:
        await drone.manual_control.start_position_control()
        print("Position control OK")
    except Exception as e:
        print("Warning: {}".format(e))
    await hover(drone, 3.0)

    # Danh sach bai test
    tests = [
        {
            "name": "TIEN vx=+{} m/s".format(VX_TEST),
            "desc": "Motor SAU (duoi) to hon motor TRUOC (mui)",
            "vel":  {"vx": +VX_TEST},
        },
        {
            "name": "LUI vx=-{} m/s".format(VX_TEST),
            "desc": "Motor TRUOC (mui) to hon motor SAU (duoi)",
            "vel":  {"vx": -VX_TEST},
        },
        {
            "name": "SANG PHAI vy=+{} m/s".format(VY_TEST),
            "desc": "Motor ben TRAI to hon motor ben PHAI",
            "vel":  {"vy": +VY_TEST},
        },
        {
            "name": "SANG TRAI vy=-{} m/s".format(VY_TEST),
            "desc": "Motor ben PHAI to hon motor ben TRAI",
            "vel":  {"vy": -VY_TEST},
        },
        {
            "name": "LEN vz=-{} m/s".format(VZ_TEST),
            "desc": "TAT CA motor to hon deu -- GIU CHAT DRONE!",
            "vel":  {"vz": -VZ_TEST},
        },
        {
            "name": "XUONG vz=+{} m/s".format(VZ_TEST),
            "desc": "TAT CA motor nho hon",
            "vel":  {"vz": +VZ_TEST},
        },
        {
            "name": "YAW PHAI +{}deg/s".format(YAW_TEST),
            "desc": "Motor cheo CW to hon CCW",
            "vel":  {"yaw_dps": +YAW_TEST},
        },
        {
            "name": "YAW TRAI -{}deg/s".format(YAW_TEST),
            "desc": "Motor cheo CCW to hon CW",
            "vel":  {"yaw_dps": -YAW_TEST},
        },
    ]

    for i, test in enumerate(tests):
        print("\n" + "=" * 60)
        print("[{}/{}] {}".format(i+1, len(tests), test["name"]))
        print("  Nghe: {}".format(test["desc"]))

        vel = {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw_dps": 0.0}
        vel.update(test["vel"])
        p, r, t, y = to_input(**vel, p=params)

        if vel["vz"] == 0.0:
            t = THR_IDLE

        print("  Input: vx={:+.1f} vy={:+.1f} vz={:+.1f} yaw={:+.1f}deg/s".format(
            vel["vx"], vel["vy"], vel["vz"], vel["yaw_dps"]))
        print("  MAVSDK: pitch={:+.3f} roll={:+.3f} thr={:.3f} yaw={:+.3f}".format(p, r, t, y))
        print("  Bat dau {}s...".format(int(TEST_DURATION)))

        await send(drone, p, r, t, y, TEST_DURATION)

        print("  Xong -- hover 2s...")
        await hover(drone, 2.0)

    print("\n" + "=" * 60)
    print("HOAN THANH -- Dung QGC de disarm.")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connection', '-c', default=CONNECTION)
    args = parser.parse_args()
    asyncio.run(run(args.connection))


if __name__ == '__main__':
    main()
