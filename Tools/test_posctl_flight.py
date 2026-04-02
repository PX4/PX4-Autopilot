#!/usr/bin/env python3
"""
test_posctl_flight.py -- Test bay thuc te o POSCTL mode
========================================================
Quy trinh:
  1. Ket noi UAV
  2. Doc params
  3. Cho POSCTL mode -> trigger chay code tu dong
  4. Chay tien/lui/trai/phai cham (1.5 m/s)
  5. Ghi log CSV

POSCTL: PX4 tu bu gio ben trong -> van toc thuc sat voi van toc dat

Chay:
  python3 test_posctl_flight.py
  python3 test_posctl_flight.py --connection serial:///dev/ttyACM0:57600
  python3 test_posctl_flight.py --connection serial:///dev/ttyACM0:57600 --rc-port /dev/ttyUSB0
"""

import asyncio
import sys
import os
import argparse
import csv
import math
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from px4_inverse_rc import position_mode_to_mavlink
from mavsdk import System
from mavsdk.manual_control import ManualControlError

try:
    import serial_asyncio
    from crc import calculate_crc16
    HAS_UART = True
except ImportError:
    HAS_UART = False

# ── Cau hinh ──────────────────────────────────────────────────────────────────
DRONE_CONN  = "serial:///dev/ttyACM0:57600"
RC_PORT     = "/dev/ttyUSB0"
RC_BAUDRATE = 115200
RATE_HZ     = 20
INTERVAL    = 1.0 / RATE_HZ

VX        = 1.0  # m/s tien/lui
VY        = 1.0   # m/s trai/phai
MOVE_TIME = 10.0   # s moi buoc -- du de PX4 on dinh van toc
HOVER_TIME = 3.0  # s hover giua cac buoc
WARMUP_TIME = 3.0 # s warmup RC source


# ── UART RC ───────────────────────────────────────────────────────────────────

class SerialRC:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.writer = None
        self.p = bytearray(10)

    async def connect(self):
        try:
            _, self.writer = await serial_asyncio.open_serial_connection(
                url=self.port, baudrate=self.baudrate)
            print("  RC UART OK: {}".format(self.port))
            return True
        except Exception as e:
            print("  RC UART loi: {}".format(e))
            return False

    def close(self):
        if self.writer:
            self.writer.close()

    async def send(self, pitch, roll, throttle, yaw):
        if not self.writer:
            return
        self.p[0] = 0xAA
        self.p[1] = 0x55
        self.p[2] = int(((roll + 1) / 2) * 254)
        self.p[3] = int(((pitch + 1) / 2) * 254)
        self.p[4] = int(throttle * 254)
        self.p[5] = int(((yaw + 1) / 2) * 254)
        self.p[6] = 0x7F
        self.p[7] = 0x02
        crc = calculate_crc16(self.p[:8])
        self.p[8] = crc & 0xFF
        self.p[9] = (crc >> 8) & 0xFF
        try:
            self.writer.write(self.p)
            await self.writer.drain()
        except Exception as e:
            print("  UART loi: {}".format(e))


# ── Flight Logger ─────────────────────────────────────────────────────────────

class FlightLogger:
    HEADER = [
        'timestamp',
        'heading_deg',                    # huong drone (deg)
        'vx_ned', 'vy_ned', 'vz_ned',    # van toc that NED (m/s)
        'vx_body', 'vy_body',             # van toc that body frame (m/s)
        'vx_set', 'vy_set', 'vz_set',    # van toc dat (m/s)
        'pitch_cmd', 'roll_cmd', 'thr_cmd',
        'event',
    ]

    def __init__(self, filename):
        self.filename = filename
        self.f = open(filename, 'w', newline='')
        self.writer = csv.writer(self.f)
        self.writer.writerow(self.HEADER)
        self.f.flush()
        print("  Log -> {}".format(filename))

    def write(self, heading_deg,
              vx_ned, vy_ned, vz_ned,
              vx_set, vy_set, vz_set,
              pitch_cmd, roll_cmd, thr_cmd,
              event=''):
        yaw = math.radians(heading_deg)
        vx_body =  vx_ned * math.cos(yaw) + vy_ned * math.sin(yaw)
        vy_body = -vx_ned * math.sin(yaw) + vy_ned * math.cos(yaw)
        row = [
            datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            round(heading_deg, 1),
            round(vx_ned, 3), round(vy_ned, 3), round(vz_ned, 3),
            round(vx_body, 3), round(vy_body, 3),
            round(vx_set, 3), round(vy_set, 3), round(vz_set, 3),
            round(pitch_cmd, 4), round(roll_cmd, 4), round(thr_cmd, 4),
            event,
        ]
        self.writer.writerow(row)
        self.f.flush()

    def close(self):
        self.f.close()


# ── Doc params ────────────────────────────────────────────────────────────────

async def read_params(drone):
    p = dict(vel_max=10.0, vel_max_up=3.0, vel_max_dn=1.5,
             yaw_max=150.0, deadzone=0.0)
    MAP = {
        'MPC_VEL_MANUAL':   'vel_max',
        'MPC_Z_VEL_MAX_UP': 'vel_max_up',
        'MPC_Z_VEL_MAX_DN': 'vel_max_dn',
        'MPC_MAN_Y_MAX':    'yaw_max',
        'MAN_DEADZONE':     'deadzone',
    }
    print("2. Doc params...")
    for name, key in MAP.items():
        try:
            p[key] = await drone.param.get_param_float(name)
            print("   {} = {:.3f}".format(name, p[key]))
        except Exception:
            print("   {} = {:.3f} (default)".format(name, p[key]))
    print()
    return p


# ── Tinh lenh ─────────────────────────────────────────────────────────────────

def calc(vx=0.0, vy=0.0, vz=0.0, yaw_dps=0.0, p=None):
    """Van toc body frame -> (pitch, roll, throttle, yaw) MAVSDK"""
    p = p or {}
    x, y, z, r = position_mode_to_mavlink(
        vx=vx, vy=vy, vz=vz, yaw_rate_dps=yaw_dps,
        vel_max=p.get('vel_max', 10.0),
        vel_max_up=p.get('vel_max_up', 3.0),
        vel_max_dn=p.get('vel_max_dn', 1.5),
        yaw_max=p.get('yaw_max', 150.0),
        deadzone=p.get('deadzone', 0.0),
    )
    return +x/1000.0, +y/1000.0, z/1000.0, r/1000.0


# ── Gui lenh ──────────────────────────────────────────────────────────────────

async def send_cmd(rc, drone, pitch, roll, thr, yaw):
    if rc:
        # Mach trung gian: send(roll, pitch, thr, yaw)
        # p[2]=roll, p[3]=pitch -- doi cho de dung voi protocol UART
        await rc.send(pitch, roll, thr, yaw)
    else:
        await drone.manual_control.set_manual_control_input(pitch, roll, thr, yaw)


async def move(rc, drone, pitch, roll, thr, yaw, duration, label,
               logger, telem, vx_set=0.0, vy_set=0.0, vz_set=0.0):
    print("  >>> {} ({:.0f}s)".format(label, duration))
    steps = int(duration * RATE_HZ)
    for i in range(steps):
        await send_cmd(rc, drone, pitch, roll, thr, yaw)
        logger.write(
            telem["hdg"],
            telem["vx"], telem["vy"], telem["vz"],
            vx_set, vy_set, vz_set,
            pitch, roll, thr,
            event=label if i == 0 else '')
        await asyncio.sleep(INTERVAL)
        remaining = duration - i * INTERVAL
        if i % (RATE_HZ * 2) == 0 and remaining > 0.5:
            # Tinh vx_body, vy_body de hien thi
            yaw_rad = math.radians(telem["hdg"])
            bx = telem["vx"]*math.cos(yaw_rad) + telem["vy"]*math.sin(yaw_rad)
            by = -telem["vx"]*math.sin(yaw_rad) + telem["vy"]*math.cos(yaw_rad)
            print("      {:.0f}s | body fwd={:+.2f} right={:+.2f} | set fwd={:+.1f} right={:+.1f}".format(
                remaining, bx, by, vx_set, vy_set))


async def hover(rc, drone, duration, logger, telem):
    print("  [hover {:.0f}s]".format(duration))
    for i in range(int(duration * RATE_HZ)):
        await send_cmd(rc, drone, 0.0, 0.0, 0.5, 0.0)
        logger.write(
            telem["hdg"],
            telem["vx"], telem["vy"], telem["vz"],
            0, 0, 0, 0.0, 0.0, 0.5,
            event='HOVER' if i == 0 else '')
        await asyncio.sleep(INTERVAL)


# ── Main ──────────────────────────────────────────────────────────────────────

async def run(args):
    # 1. Ket noi UAV
    print("1. Ket noi {}...".format(args.connection))
    drone = System()
    await drone.connect(system_address=args.connection)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("   OK!\n")
            break

    # 2. Doc params
    params = await read_params(drone)

    # Telemetry state
    telem = {"vx": 0.0, "vy": 0.0, "vz": 0.0, "hdg": 0.0}

    async def watch_vel():
        async for v in drone.telemetry.velocity_ned():
            telem["vx"] = v.north_m_s
            telem["vy"] = v.east_m_s
            telem["vz"] = v.down_m_s

    async def watch_att():
        async for a in drone.telemetry.attitude_euler():
            telem["hdg"] = a.yaw_deg

    asyncio.ensure_future(watch_vel())
    asyncio.ensure_future(watch_att())
    await asyncio.sleep(0.5)

    # Ket noi UART RC
    rc = None
    if HAS_UART and not args.no_rc:
        rc = SerialRC(args.rc_port, RC_BAUDRATE)
        if not await rc.connect():
            print("  Khong ket noi UART -- dung MAVSDK fallback")
            rc = None

    # 3. Cho POSCTL mode lam trigger
    print("3. Cho POSCTL mode...")
    print("   Dung QGC: arm -> takeoff -> chuyen POSITION mode")
    print("   Code tu dong chay ngay khi vao POSCTL\n")

    async for mode in drone.telemetry.flight_mode():
        mode_str = str(mode).upper()
        if mode_str not in ("UNKNOWN", ""):
            print("   [Mode] {}".format(mode_str))
        if "POS" in mode_str:
            print("\n   POSCTL detected! Chay code tu dong...\n")
            break

    # Warmup RC source -- quan trong voi POSCTL
    print("[WARMUP] {}s -- nha joystick...".format(WARMUP_TIME))
    for _ in range(int(WARMUP_TIME * RATE_HZ)):
        await send_cmd(rc, drone, 0.0, 0.0, 0.5, 0.0)
        await asyncio.sleep(INTERVAL)

    # Kich hoat MAVSDK position control (neu dung MAVSDK)
    if rc is None:
        for attempt in range(3):
            try:
                await drone.manual_control.start_position_control()
                print("start_position_control OK")
                break
            except ManualControlError as e:
                print("  Lan {}: {} -- warmup them...".format(attempt+1, e))
                for _ in range(int(2.0 * RATE_HZ)):
                    await send_cmd(rc, drone, 0.0, 0.0, 0.5, 0.0)
                    await asyncio.sleep(INTERVAL)

    # Khoi tao logger
    log_filename = "posctl_{}.csv".format(
        datetime.now().strftime("%Y%m%d_%H%M%S"))
    logger = FlightLogger(log_filename)
    logger.write(telem["hdg"],
                 telem["vx"], telem["vy"], telem["vz"],
                 0, 0, 0, 0, 0, 0.5, event="POSCTL_TRIGGER")

    # 4. Chay tu dong
    fwd  = calc(vx=+VX, p=params)
    back = calc(vx=-VX, p=params)
    rgt  = calc(vy=+VY, p=params)
    lft  = calc(vy=-VY, p=params)

    print("=" * 55)
    print("BAT DAU POSCTL  |  VX={} m/s  VY={} m/s".format(VX, VY))
    print("  heading hien tai: {:.1f} deg".format(telem["hdg"]))
    print("  Tien  : pitch={:+.3f}".format(fwd[0]))
    print("  Lui   : pitch={:+.3f}".format(back[0]))
    print("  Phai  : roll={:+.3f}".format(rgt[1]))
    print("  Trai  : roll={:+.3f}".format(lft[1]))
    print("  POSCTL se tu bu gio -- van toc thuc sat voi van toc dat")
    print("=" * 55 + "\n")

    steps = [
        (fwd,  "TIEN  vx=+{} m/s".format(VX), +VX,  0.0, 0.0),
        (back, "LUI   vx=-{} m/s".format(VX), -VX,  0.0, 0.0),
        (rgt,  "PHAI  vy=+{} m/s".format(VY),  0.0, +VY, 0.0),
        (lft,  "TRAI  vy=-{} m/s".format(VY),  0.0, -VY, 0.0),
    ]

    for i, (cmd, label, vx_s, vy_s, vz_s) in enumerate(steps):
        print("[{}/{}] {}".format(i+1, len(steps), label))
        await move(rc, drone, cmd[0], cmd[1], cmd[2], cmd[3],
                   MOVE_TIME, label, logger, telem,
                   vx_set=vx_s, vy_set=vy_s, vz_set=vz_s)
        await hover(rc, drone, HOVER_TIME, logger, telem)

    logger.write(telem["hdg"],
                 telem["vx"], telem["vy"], telem["vz"],
                 0, 0, 0, 0, 0, 0.5, event="DONE")
    logger.close()
    print("\nLog da luu: {}".format(log_filename))
    print("HOAN THANH -- Phi cong nhan lai dieu khien")

    if rc:
        rc.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connection', '-c', default=DRONE_CONN)
    parser.add_argument('--rc-port', default=RC_PORT)
    parser.add_argument('--no-rc', action='store_true',
                        help='Dung MAVSDK truc tiep, khong qua UART')
    args = parser.parse_args()

    print("test_posctl_flight.py")
    print("  drone  : {}".format(args.connection))
    print("  rc uart: {}".format("disabled" if args.no_rc else args.rc_port))
    print()
    asyncio.run(run(args))


if __name__ == '__main__':
    main()