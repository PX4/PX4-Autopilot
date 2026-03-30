# Hướng Dẫn Sử Dụng `px4_inverse_rc.py`

Thư viện toán thuần — chuyển **giá trị vật lý** → **RC stick** → **MAVLink MANUAL_CONTROL**.  
Không phụ thuộc pymavlink. Không sửa firmware.

## Cài Đặt

```bash
pip install numpy
```

---

## 1. Dùng Nhanh — Giá Trị Mặc Định

```python
from px4_inverse_rc import position_mode_to_mavlink, altitude_mode_to_mavlink

# Position Mode: bay tiến 5 m/s, lên 1 m/s
x, y, z, r = position_mode_to_mavlink(vx=5.0, vy=0.0, vz=-1.0, yaw_rate_dps=0.0)

# Altitude Mode: nghiêng pitch 15°, xoay yaw 30°/s
x, y, z, r = altitude_mode_to_mavlink(pitch_deg=15.0, roll_deg=0.0, yaw_rate_dps=30.0)
```

## 2. Truyền Param Từ PX4

Đọc param bên ngoài bằng pymavlink, rồi truyền vào hàm:

```python
from pymavlink import mavutil
from px4_inverse_rc import position_mode_to_mavlink

# Kết nối và đọc param
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
master.wait_heartbeat()

def read_param(name):
    master.mav.param_request_read_send(
        master.target_system, master.target_component, name.encode(), -1)
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
    return msg.param_value if msg else None

vel_max = read_param('MPC_VEL_MANUAL') or 10.0
deadzone = read_param('MAN_DEADZONE') or 0.0
vel_up = read_param('MPC_Z_VEL_MAX_UP') or 3.0
vel_dn = read_param('MPC_Z_VEL_MAX_DN') or 1.5
yaw_max = read_param('MPC_MAN_Y_MAX') or 150.0

# Truyền param thực vào hàm
x, y, z, r = position_mode_to_mavlink(
    vx=5.0, vy=0.0,
    vel_max=vel_max, vel_max_up=vel_up, vel_max_dn=vel_dn,
    yaw_max=yaw_max, deadzone=deadzone,
)

# Gửi MAVLink
master.mav.manual_control_send(master.target_system, x, y, z, r, 0)
```

## 3. Dùng Hàm Từng Trục

```python
from px4_inverse_rc import (
    velocity_to_stick, tilt_to_stick,
    vz_to_throttle_stick, yaw_rate_to_stick,
    stick_to_mavlink,
)

# Tính stick riêng từng trục
pitch_s = velocity_to_stick(5.0, vel_max=10.0)       # → 0.711
roll_s  = tilt_to_stick(15.0, tilt_max_deg=35.0)      # → 0.383
thr_s   = vz_to_throttle_stick(-1.0)                  # lên 1 m/s
yaw_s   = yaw_rate_to_stick(45.0)                      # 45°/s

# Ghép thành MAVLink
x, y, z, r = stick_to_mavlink(pitch_s, roll_s, thr_s, yaw_s)
```

## 4. Gửi Liên Tục

```python
import time

for _ in range(50):
    x, y, z, r = position_mode_to_mavlink(vx=3.0, vy=0.0)
    master.mav.manual_control_send(master.target_system, x, y, z, r, 0)
    time.sleep(0.1)

# Hover
master.mav.manual_control_send(master.target_system, 0, 0, 500, 0, 0)
```

---

## Bảng Tham Số

| Tham số hàm | Param PX4 | Default | Ý nghĩa |
|-------------|-----------|---------|---------|
| `vel_max` | `MPC_VEL_MANUAL` | 10.0 m/s | Vận tốc ngang tối đa |
| `tilt_max` | `MPC_MAN_TILT_MAX` | 35° | Góc nghiêng tối đa |
| `vel_max_up` | `MPC_Z_VEL_MAX_UP` | 3.0 m/s | Vận tốc lên tối đa |
| `vel_max_dn` | `MPC_Z_VEL_MAX_DN` | 1.5 m/s | Vận tốc xuống tối đa |
| `yaw_max` | `MPC_MAN_Y_MAX` | 150 °/s | Tốc độ xoay tối đa |
| `expo` | — | 0.6 | Hardcoded trong PX4 |
| `deadzone` | `MAN_DEADZONE` | 0.0 | Vùng chết stick |

## Quy Ước Dấu

| Trục | Dương (+) | Âm (−) |
|------|-----------|--------|
| `vx` | Tiến | Lùi |
| `vy` | Phải | Trái |
| `vz` | Xuống (NED) | **Lên** |
| `yaw_rate` | Xoay phải | Xoay trái |
| `pitch_deg` | Nghiêng tới | Nghiêng lui |
| `roll_deg` | Nghiêng phải | Nghiêng trái |

## Kiểm Chứng

```bash
python3 Tools/px4_inverse_rc.py
```

Chi tiết toán học: `docs/px4_co_so_toan_hoc_ham_nguoc_rc.md`
