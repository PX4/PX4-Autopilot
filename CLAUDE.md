# CLAUDE.md — PX4-Autopilot Project Context

## Người dùng
- Tên: Thang (ThangDuc3101 trên GitHub)
- Làm việc trên nhánh: `feature/acc-control-position-altitude`
- Ngôn ngữ giao tiếp: Tiếng Việt

---

## Mục tiêu dự án

Thêm tính năng **external acceleration control** vào PX4 firmware, cho phép controller bên ngoài gửi NED acceleration setpoints vào Position Mode / Altitude Mode **mà không cần OFFBOARD mode**.

Hai hướng tiếp cận song song:
1. **Firmware path (acc_sp_external)** — dùng uORB topic `AccSpExternal` + bit 12 trong MAVLink type_mask
2. **Inverse RC path (px4_inverse_rc)** — tính ngược RC stick values từ physical setpoints, gửi qua `MANUAL_CONTROL`

---

## Kiến trúc đã implement

### Firmware side (C++)
- **`msg/AccSpExternal.msg`** — uORB topic: `timestamp, acc[3], yaw_rad, timeout_ms`
- **`src/modules/mavlink/mavlink_receiver.cpp`** — detect bit 12 trong `SET_POSITION_TARGET_LOCAL_NED`, publish `acc_sp_external` không qua OFFBOARD gate
- **`FlightTaskManualAltitude`** — thêm `_applyExternalAcceleration()`: watchdog timeout, z_valid check, clamp theo `MPC_ACC_HOR`
- **`FlightTaskManualAcceleration`** — gọi `_applyExternalAcceleration()` sau stick computation (override stick)
- **`AccSpExternalTest.cpp`** — unit tests: clamping, watchdog, NaN rejection, yaw passthrough

### Python side (Tools/)
- **`acceleration_control.py`** — class `AccelerationControl`: connect/arm/land/mode-switch + `send_acceleration_ned(ax, ay, az, yaw_rad=None)`
- **`test_acceleration_control.py`** — 21 unit tests (type_mask encoding, dataclass, hover, disconnected guards)
- **`sitl_acc_test.py`** — SITL integration test runner, 5 test cases (xem bên dưới)
- **`px4_inverse_rc.py`** — inverse RC math: physical values → RC stick → `MANUAL_CONTROL`
- **`params_real.json`** — PX4 params thực tế từ hardware

### Trạng thái commit hiện tại
Tất cả file đã được commit. Không còn untracked files liên quan đến feature này.

### Docs (Vietnamese)
- `docs/dieu_khien_uav_quadrotor_gia_toc.md` — kiến trúc cascaded control, acc→thrust math
- `docs/px4_co_so_toan_hoc_ham_nguoc_rc.md` — toán học inverse RC
- `docs/thu_nghiem_sitl_inverse_rc.md` — kết quả thử nghiệm SITL

---

## SITL Integration Tests (`sitl_acc_test.py`)

| Test | Mô tả | Pass condition |
|------|--------|----------------|
| TC-01 | Smoke test — acc_sp_external nhận được | tilt > 1° |
| TC-02 | Clamping — 20 m/s² bị clamp về MPC_ACC_HOR | tilt ≤ 30° |
| TC-03 | Kinematics — step 2 m/s² × 3s vs theoretical | sai lệch ≤ 30% |
| TC-04 | Watchdog — dừng publish 1.5s | velocity không tăng |
| TC-05 | Square trajectory — 4 legs | quay về origin trong 3m |

Chạy:
```bash
python3 Tools/sitl_acc_test.py --mode position
python3 Tools/sitl_acc_test.py --mode altitude --connection udp://:14550
```

---

## Thông số PX4 quan trọng

| Param | Giá trị mặc định | Ý nghĩa |
|-------|-----------------|---------|
| `MPC_ACC_HOR` | 3.0 m/s² | Giới hạn gia tốc ngang (dùng để clamp) |
| `MPC_ACC_UP_MAX` | 4.0 m/s² | Gia tốc lên tối đa |
| `MPC_VEL_MANUAL` | 10.0 m/s | Velocity tối đa Position Mode |
| `MPC_MAN_TILT_MAX` | 35° | Góc nghiêng tối đa |
| `MPC_MAN_Y_MAX` | 150 °/s | Yaw rate tối đa |
| `MAN_DEADZONE` | 0.0 | Vùng chết stick |

---

## Quy ước code

- Commit prefix: `feat(...)`, `fix(...)`, `test(...)`, `docs(...)`
- Python: dùng `pymavlink` trực tiếp, không dùng MAVSDK
- SITL test infrastructure: **hybrid mavsdk + pymavlink** — mavsdk (`udp://:14540`) cho arm/takeoff/land/mode-switch; pymavlink (`udpin:0.0.0.0:14550`) cho acc commands + telemetry
- Unit test Python: `unittest` standard library
- MAVLink frame: `MAV_FRAME_LOCAL_NED` (NED convention: Z dương = xuống)
- Bit 12 trong `type_mask` là custom PX4 extension cho acc_sp_external
- **QUAN TRỌNG — type_mask cho acc_sp_external**: Bit 9 = `FORCE_SET` (KHÔNG được set). Bit 10 = YAW_IGNORE. Bit 11 = YAW_RATE_IGNORE. Đã fix bug trong `acceleration_control.py` ngày 2026-03-30.

---

## Workflow thông thường

1. Build firmware: `make px4_sitl gazebo-classic_iris`
2. Chạy SITL: `make px4_sitl_default gazebo-classic`
3. Chạy test Python: `python3 Tools/sitl_acc_test.py --mode position`
4. Unit test Python: `python3 -m pytest Tools/test_acceleration_control.py`
5. Unit test C++: `make tests`

---

## Bug đã tìm ra — chưa verify (cần test ngày mai)

### Root cause: type_mask bit 9 = FORCE_SET
- **Triệu chứng**: `listener acc_sp_external` → "never published"; PX4 log: `SET_POSITION_TARGET_LOCAL_NED force not supported`
- **Nguyên nhân**: `acceleration_control.py` set bit 9 (`1 << 9`) gán nhầm nhãn "ignore yaw" nhưng thực ra là `POSITION_TARGET_TYPEMASK_FORCE_SET`. Firmware `mavlink_receiver.cpp:1121` reject ngay khi thấy acceleration + FORCE_SET, không bao giờ đến check bit 12 ở line 1133.
- **Fix đã apply**: Xóa `(1 << 9)`, thêm `(1 << 11)` (YAW_RATE_IGNORE) vào `_TYPE_MASK_ACC_ONLY` và `_TYPE_MASK_ACC_WITH_YAW` trong `Tools/acceleration_control.py`
- **Bước tiếp theo**: Chạy SITL → verify `listener acc_sp_external` có output → chạy full TC-01..TC-05
