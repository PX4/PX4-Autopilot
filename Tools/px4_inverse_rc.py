#!/usr/bin/env python3
"""
PX4 Inverse RC: Chuyển đổi giá trị vật lý (vận tốc, góc nghiêng, tốc độ xoay)
sang giá trị RC stick để gửi qua MAVLink MANUAL_CONTROL (msg #69).

Thư viện toán thuần — không phụ thuộc pymavlink.
Không sửa firmware PX4. Yêu cầu: COM_RC_IN_MODE = 1 hoặc ≥ 2.

Cơ sở toán học: docs/px4_co_so_toan_hoc_ham_nguoc_rc.md
Mã nguồn PX4 tham khảo:
  - src/lib/mathlib/math/Functions.hpp (expo, deadzone, expo_deadzone)
  - src/lib/sticks/Sticks.hpp (expo = 0.6 hardcoded)
  - src/modules/mavlink/mavlink_receiver.cpp (MAVLink → stick mapping)
  - src/modules/flight_mode_manager/tasks/ManualPosition/FlightTaskManualPosition.cpp
  - src/modules/flight_mode_manager/tasks/ManualAltitude/FlightTaskManualAltitude.cpp
  - src/modules/flight_mode_manager/tasks/Utility/StickTiltXY.cpp
"""

import math
import numpy as np

# Giá trị mặc định từ mã nguồn PX4
EXPO_DEFAULT = 0.6            # Sticks.hpp:67 — hardcoded
DEADZONE_DEFAULT = 0.0        # MAN_DEADZONE
VEL_MANUAL_DEFAULT = 10.0     # MPC_VEL_MANUAL (m/s)
TILT_MAX_DEFAULT = 35.0       # MPC_MAN_TILT_MAX (độ)
Z_VEL_MAX_UP_DEFAULT = 3.0    # MPC_Z_VEL_MAX_UP (m/s)
Z_VEL_MAX_DN_DEFAULT = 1.5    # MPC_Z_VEL_MAX_DN (m/s)
YAW_MAX_DEFAULT = 150.0       # MPC_MAN_Y_MAX (°/s)


# ============================================================================
# 1. HÀM THUẬN (Forward) — Tái tạo logic PX4 trong Python để kiểm chứng
# ============================================================================

def px4_deadzone(value: float, dz: float = DEADZONE_DEFAULT) -> float:
    """Hàm deadzone của PX4. Nguồn: Functions.hpp:124-133"""
    x = np.clip(value, -1.0, 1.0)
    dz = np.clip(dz, 0.0, 0.99)
    out = (x - math.copysign(dz, x)) / (1.0 - dz)
    return out * (abs(x) > dz)


def px4_expo(value: float, e: float = EXPO_DEFAULT) -> float:
    """Hàm expo của PX4. Nguồn: Functions.hpp:83-89"""
    x = np.clip(value, -1.0, 1.0)
    e = np.clip(e, 0.0, 1.0)
    return (1.0 - e) * x + e * x * x * x


def px4_expo_deadzone(value: float, e: float = EXPO_DEFAULT,
                      dz: float = DEADZONE_DEFAULT) -> float:
    """Hàm hợp expo_deadzone. Nguồn: Functions.hpp:136-139"""
    return px4_expo(px4_deadzone(value, dz), e)


# ============================================================================
# 2. HÀM NGƯỢC (Inverse) — Giải ngược từ giá trị vật lý → stick
# ============================================================================

def inv_expo(y_hat: float, e: float = EXPO_DEFAULT) -> float:
    """Giải ngược hàm expo bằng Newton-Raphson.

    Tìm d từ: e*d^3 + (1-e)*d - y_hat = 0
    Khởi tạo d0 = y_hat, hội tụ sau 5-10 vòng lặp.
    """
    y_hat = np.clip(y_hat, -1.0, 1.0)
    if abs(e) < 1e-10:
        return y_hat

    d = y_hat
    one_minus_e = 1.0 - e
    for _ in range(20):
        f = e * d * d * d + one_minus_e * d - y_hat
        f_prime = 3.0 * e * d * d + one_minus_e
        if abs(f_prime) < 1e-15:
            break
        delta = f / f_prime
        d -= delta
        if abs(delta) < 1e-12:
            break

    return np.clip(d, -1.0, 1.0)


def inv_deadzone(d: float, dz: float = DEADZONE_DEFAULT) -> float:
    """Giải ngược hàm deadzone: s = d*(1-dz) + sign(d)*dz"""
    dz = np.clip(dz, 0.0, 0.99)
    if abs(d) < 1e-15:
        return 0.0
    return d * (1.0 - dz) + math.copysign(dz, d)


def inv_expo_deadzone(y_hat: float, e: float = EXPO_DEFAULT,
                      dz: float = DEADZONE_DEFAULT) -> float:
    """Giải ngược hàm hợp expo_deadzone: y_hat → stick value."""
    return inv_deadzone(inv_expo(y_hat, e), dz)


# ============================================================================
# 3. CHUYỂN ĐỔI THEO TRỤC — Giá trị vật lý → stick [-1, 1]
# ============================================================================

def velocity_to_stick(velocity: float,
                      vel_max: float = VEL_MANUAL_DEFAULT,
                      expo: float = EXPO_DEFAULT,
                      deadzone: float = DEADZONE_DEFAULT) -> float:
    """Position Mode pitch/roll: vận tốc (m/s) → stick [-1, 1].

    Chuỗi thuận PX4: Y = F(s) × vel_max
    Ngược: s = F⁻¹(Y / vel_max)
    """
    y_hat = np.clip(velocity / vel_max, -1.0, 1.0)
    return inv_expo_deadzone(y_hat, expo, deadzone)


def tilt_to_stick(angle_deg: float,
                  tilt_max_deg: float = TILT_MAX_DEFAULT) -> float:
    """Altitude Mode pitch/roll: góc nghiêng (°) → stick [-1, 1].

    Công thức trực tiếp (KHÔNG expo): s = tan(θ) / tan(θ_max)
    """
    theta = math.radians(angle_deg)
    theta_max = math.radians(np.clip(tilt_max_deg, 0.0, 89.0))
    s = math.tan(theta) / math.tan(theta_max)
    return float(np.clip(s, -1.0, 1.0))


def vz_to_throttle_stick(vz: float,
                         vel_max_up: float = Z_VEL_MAX_UP_DEFAULT,
                         vel_max_dn: float = Z_VEL_MAX_DN_DEFAULT,
                         expo: float = EXPO_DEFAULT,
                         deadzone: float = DEADZONE_DEFAULT) -> float:
    """Vận tốc đứng (m/s, NED: dương=xuống) → throttle stick [-1, 1].

    Dấu trừ vì NED: Z dương = xuống, stick dương = lên.
    """
    stick_dir = -vz
    vel_max = vel_max_up if stick_dir >= 0 else vel_max_dn
    y_hat = np.clip(stick_dir / vel_max, -1.0, 1.0)
    return inv_expo_deadzone(y_hat, expo, deadzone)


def yaw_rate_to_stick(yaw_rate_dps: float,
                      yaw_max_dps: float = YAW_MAX_DEFAULT,
                      expo: float = EXPO_DEFAULT,
                      deadzone: float = DEADZONE_DEFAULT) -> float:
    """Tốc độ xoay yaw (°/s) → yaw stick [-1, 1]."""
    y_hat = np.clip(yaw_rate_dps / yaw_max_dps, -1.0, 1.0)
    return inv_expo_deadzone(y_hat, expo, deadzone)


# ============================================================================
# 4. CHUYỂN ĐỔI STICK → MAVLINK
# ============================================================================

def stick_to_mavlink(pitch_stick: float, roll_stick: float,
                     throttle_stick: float, yaw_stick: float) -> tuple:
    """Chuyển stick [-1, 1] sang MAVLink MANUAL_CONTROL values.

    Nguồn: mavlink_receiver.cpp:2102-2113

    Returns:
        (x, y, z, r):
            x (pitch):    [-1000, 1000]
            y (roll):     [-1000, 1000]
            z (throttle): [0, 1000]
            r (yaw):      [-1000, 1000]
    """
    x = int(round(np.clip(pitch_stick, -1.0, 1.0) * 1000))
    y = int(round(np.clip(roll_stick, -1.0, 1.0) * 1000))
    z = int(round((np.clip(throttle_stick, -1.0, 1.0) + 1.0) * 500))
    r = int(round(np.clip(yaw_stick, -1.0, 1.0) * 1000))
    return (x, y, z, r)


# ============================================================================
# 5. API CẤP CAO — Giá trị vật lý → MAVLink (x, y, z, r)
# ============================================================================

def position_mode_to_mavlink(vx: float, vy: float,
                             vz: float = 0.0, yaw_rate_dps: float = 0.0,
                             vel_max: float = VEL_MANUAL_DEFAULT,
                             vel_max_up: float = Z_VEL_MAX_UP_DEFAULT,
                             vel_max_dn: float = Z_VEL_MAX_DN_DEFAULT,
                             yaw_max: float = YAW_MAX_DEFAULT,
                             expo: float = EXPO_DEFAULT,
                             deadzone: float = DEADZONE_DEFAULT) -> tuple:
    """Position Mode: vận tốc mong muốn → MAVLink (x, y, z, r).

    Args:
        vx: Vận tốc tiến/lùi (m/s), dương = tiến
        vy: Vận tốc trái/phải (m/s), dương = phải
        vz: Vận tốc đứng (m/s, NED: dương = xuống)
        yaw_rate_dps: Tốc độ xoay yaw (°/s)
        vel_max: MPC_VEL_MANUAL
        vel_max_up: MPC_Z_VEL_MAX_UP
        vel_max_dn: MPC_Z_VEL_MAX_DN
        yaw_max: MPC_MAN_Y_MAX
        expo: Hệ số expo (0.6 hardcoded)
        deadzone: MAN_DEADZONE

    Returns:
        (x, y, z, r) — giá trị MAVLink MANUAL_CONTROL
    """
    ps = velocity_to_stick(vx, vel_max, expo, deadzone)
    rs = velocity_to_stick(vy, vel_max, expo, deadzone)
    ts = vz_to_throttle_stick(vz, vel_max_up, vel_max_dn, expo, deadzone)
    ys = yaw_rate_to_stick(yaw_rate_dps, yaw_max, expo, deadzone)
    return stick_to_mavlink(ps, rs, ts, ys)


def altitude_mode_to_mavlink(pitch_deg: float, roll_deg: float,
                             vz: float = 0.0, yaw_rate_dps: float = 0.0,
                             tilt_max: float = TILT_MAX_DEFAULT,
                             vel_max_up: float = Z_VEL_MAX_UP_DEFAULT,
                             vel_max_dn: float = Z_VEL_MAX_DN_DEFAULT,
                             yaw_max: float = YAW_MAX_DEFAULT,
                             expo: float = EXPO_DEFAULT,
                             deadzone: float = DEADZONE_DEFAULT) -> tuple:
    """Altitude Mode: góc nghiêng mong muốn → MAVLink (x, y, z, r).

    Args:
        pitch_deg: Góc nghiêng pitch (°), dương = tới
        roll_deg: Góc nghiêng roll (°), dương = phải
        vz: Vận tốc đứng (m/s, NED: dương = xuống)
        yaw_rate_dps: Tốc độ xoay yaw (°/s)
        tilt_max: MPC_MAN_TILT_MAX
        vel_max_up: MPC_Z_VEL_MAX_UP
        vel_max_dn: MPC_Z_VEL_MAX_DN
        yaw_max: MPC_MAN_Y_MAX
        expo: Hệ số expo (0.6 hardcoded)
        deadzone: MAN_DEADZONE

    Returns:
        (x, y, z, r) — giá trị MAVLink MANUAL_CONTROL
    """
    ps = tilt_to_stick(pitch_deg, tilt_max)
    rs = tilt_to_stick(roll_deg, tilt_max)
    ts = vz_to_throttle_stick(vz, vel_max_up, vel_max_dn, expo, deadzone)
    ys = yaw_rate_to_stick(yaw_rate_dps, yaw_max, expo, deadzone)
    return stick_to_mavlink(ps, rs, ts, ys)


# ============================================================================
# 6. KIỂM CHỨNG — Forward(Inverse(target)) == target
# ============================================================================

def verify_conversion():
    """Kiểm chứng hàm ngược bằng hàm thuận. Chạy offline, không cần MAVLink."""
    e = EXPO_DEFAULT
    dz = DEADZONE_DEFAULT

    print("=" * 70)
    print("KIỂM CHỨNG HÀM NGƯỢC PX4")
    print("=" * 70)
    all_pass = True

    # --- Test 1: inv_expo_deadzone ↔ px4_expo_deadzone ---
    print("\n--- Test inv_expo_deadzone → px4_expo_deadzone ---")
    for y_hat in [-1.0, -0.667, -0.5, -0.3, -0.1, 0.0, 0.1, 0.3, 0.5, 0.667, 1.0]:
        s = inv_expo_deadzone(y_hat, e, dz)
        y_check = px4_expo_deadzone(s, e, dz)
        err = abs(y_check - y_hat)
        ok = err < 1e-6
        if not ok:
            all_pass = False
        print(f"  y_hat={y_hat:+.3f} → s={s:+.6f} → F(s)={y_check:+.6f}"
              f"  err={err:.2e} {'✅' if ok else '❌'}")

    # --- Test 2: Position Mode ---
    print("\n--- Test Position Mode: velocity → MAVLink → verify ---")
    for vx, vy, vz, yr in [(5, 0, 0, 0), (-3, 2, 0, 0), (0, 0, -1.5, 45),
                            (10, -10, 1.5, -150), (0, 0, 0, 0)]:
        x, y, z, r = position_mode_to_mavlink(vx, vy, vz, yr)
        # Kiểm chứng thuận
        ps, rs, ts, ys = x/1000, y/1000, z/500 - 1, r/1000
        vx_c = px4_expo_deadzone(ps, e, dz) * VEL_MANUAL_DEFAULT
        vy_c = px4_expo_deadzone(rs, e, dz) * VEL_MANUAL_DEFAULT
        vz_max = Z_VEL_MAX_UP_DEFAULT if ts >= 0 else Z_VEL_MAX_DN_DEFAULT
        vz_c = -px4_expo_deadzone(ts, e, dz) * vz_max
        yr_c = px4_expo_deadzone(ys, e, dz) * YAW_MAX_DEFAULT
        me = max(abs(vx_c-vx), abs(vy_c-vy), abs(vz_c-vz), abs(yr_c-yr))
        ok = me < 0.2
        if not ok:
            all_pass = False
        print(f"  vx={vx:+.0f} vy={vy:+.0f} vz={vz:+.1f} yr={yr:+.0f}"
              f"  → ({x:+5d},{y:+5d},{z:4d},{r:+5d})"
              f"  err={me:.4f} {'✅' if ok else '❌'}")

    # --- Test 3: Altitude Mode ---
    print("\n--- Test Altitude Mode: tilt → MAVLink → verify ---")
    for pd, rd, vz, yr in [(15, 0, 0, 0), (0, -10, 1, 45),
                            (35, 35, 0, 0), (-20, 5, -3, -100)]:
        x, y, z, r = altitude_mode_to_mavlink(pd, rd, vz, yr)
        ps, rs = x/1000, y/1000
        tm = math.radians(np.clip(TILT_MAX_DEFAULT, 0, 89))
        pc = math.degrees(math.atan(ps * math.tan(tm)))
        rc = math.degrees(math.atan(rs * math.tan(tm)))
        me = max(abs(pc-pd), abs(rc-rd))
        ok = me < 0.5
        if not ok:
            all_pass = False
        print(f"  p={pd:+.0f}° r={rd:+.0f}° vz={vz:+.1f} yr={yr:+.0f}"
              f"  → ({x:+5d},{y:+5d},{z:4d},{r:+5d})"
              f"  err={me:.4f}° {'✅' if ok else '❌'}")

    # --- Test 4: Với deadzone ---
    print("\n--- Test với deadzone = 0.05 ---")
    x, _, z, r = altitude_mode_to_mavlink(15, 0, 1, 45, deadzone=0.05)
    print(f"  pitch=15° → x={x} (≈383)  vz=+1 → z={z} (≈83)  yr=45 → r={r} (≈553)")

    # --- Test 5: Ví dụ vx=5 m/s ---
    print("\n--- Test ví dụ: vx=5 m/s ---")
    x, *_ = position_mode_to_mavlink(5, 0)
    s = x / 1000.0
    v_check = px4_expo_deadzone(s, e, dz) * VEL_MANUAL_DEFAULT
    err = abs(v_check - 5.0)
    ok = err < 0.05
    if not ok:
        all_pass = False
    print(f"  x={x}  F({s:.3f})×10 = {v_check:.3f} m/s  err={err:.4f} {'✅' if ok else '❌'}")

    print("\n" + "=" * 70)
    print(f"KẾT QUẢ: {'TẤT CẢ PASS ✅' if all_pass else 'CÓ LỖI ❌'}")
    print("=" * 70)
    return all_pass


if __name__ == '__main__':
    verify_conversion()
