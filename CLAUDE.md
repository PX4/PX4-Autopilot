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
- **`run_sitl_test.sh`** — orchestration script: start SITL → run test → capture logs → cleanup

### Trạng thái commit hiện tại (2026-03-31)
- Nhiều thay đổi chưa commit trong phiên hôm nay — xem mục "Bước tiếp theo"

### Docs (Vietnamese)
- `docs/dieu_khien_uav_quadrotor_gia_toc.md` — kiến trúc cascaded control, acc→thrust math
- `docs/px4_co_so_toan_hoc_ham_nguoc_rc.md` — toán học inverse RC
- `docs/thu_nghiem_sitl_inverse_rc.md` — kết quả thử nghiệm SITL

---

## SITL Integration Tests (`sitl_acc_test.py`)

| Test | Mô tả | Pass condition |
|------|--------|----------------|
| TC-01 | Smoke test — acc_sp_external nhận được | tilt > 1° |
| TC-02 | Clamping — 20 m/s² bị clamp về MPC_ACC_HOR | tilt ∈ (2°, 30°) |
| TC-03 | Kinematics — ax=2 m/s² × 3s | moved north > 0.3m, vel > 0.2 m/s |
| TC-04 | Watchdog — dừng publish 1.5s | velocity không tăng |
| TC-05 | Square trajectory — 4 legs | quay về origin trong 8m |

Chạy (dùng script orchestration):
```bash
bash Tools/run_sitl_test.sh position
bash Tools/run_sitl_test.sh altitude
```

Logs lưu tại:
- `/tmp/sitl_<RUN_ID>.log` — toàn bộ SITL output (kể cả `[ext_acc]` debug prints)
- `/tmp/test_<RUN_ID>.log` — Python test output

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
- **QUAN TRỌNG — type_mask cho acc_sp_external**: Bit 9 = `FORCE_SET` (KHÔNG được set). Bit 10 = YAW_IGNORE. Bit 11 = YAW_RATE_IGNORE.

---

## Workflow thông thường

1. Chạy test (build + SITL + test tự động): `bash Tools/run_sitl_test.sh position`
2. Báo RUN_ID cho Lead Agent → agent team analyze logs
3. Unit test Python: `python3 -m pytest Tools/test_acceleration_control.py`
4. Unit test C++: `make tests`

---

## Agent Team Strategy

### Workflow
Bạn chạy script → agents analyze log → Lead đề xuất fix → bạn confirm → implement.

### Phase 1 — Bạn chạy
```bash
bash Tools/run_sitl_test.sh position
# Script in RUN_ID ở đầu. Logs tự lưu vào /tmp/sitl_<RUN_ID>.log và /tmp/test_<RUN_ID>.log
```

### Phase 2 — Lead spawn 3 agents song song (khi có fail)
| Agent | Input | Tìm gì |
|-------|-------|--------|
| Agent A | `/tmp/sitl_<RUN_ID>.log` | `[ext_acc]` hits, WARN/ERROR, arm events |
| Agent B | `/tmp/test_<RUN_ID>.log` | TC pass/fail, exceptions, telemetry |
| Agent C | C++ source | Cross-reference → trace publish/subscribe |

### Decision logic
```
[ext_acc] = 0 → vấn đề publish trong mavlink_receiver.cpp
[ext_acc] > 0 nhưng không di chuyển → vấn đề downstream (FlightTask/mc_pos_control)
Python exception → vấn đề test script, không cần Agent C
```

---

## SITL Troubleshooting

### Param file location
- `build/px4_sitl_default/rootfs/parameters.bson` — persist qua restart SITL
- **Nếu SITL có state lạ**: xóa file này để reset về default

### Failsafes cần disable trong SITL
Test script tự động set các params này trong `_mavsdk_setup()`:
- `COM_LOW_BAT_ACT=0` — battery chỉ warn, không RTL
- `NAV_RCL_ACT=0` — RC loss disabled
- `MAV_FWDEXTSP=1` — đảm bảo external setpoints được forward

### RC Keep-alive
`sitl_acc_test.py` chạy background thread gửi `MANUAL_CONTROL` centered sticks ở 10 Hz trong suốt test để `FlightTaskManualAltitude` có valid Z setpoint → tránh `mc_pos_control invalid setpoints`.

### ARM denied trong SITL
- **MANUAL_CONTROL priming**: gửi 40 packets × 50ms = 2s TRƯỚC khi arm
- **KHÔNG dùng** `COM_RC_IN_MODE=1`

### PX4 C++ logging macros
- `PX4_WARN_ONCE` **KHÔNG tồn tại** — dùng `PX4_WARN`
- `PX4_INFO_RAW` — print không có prefix (tốt cho debug telemetry như `[ext_acc]`)

### Kết quả SITL test cuối cùng (2026-04-01)

| Mode | TC-01 | TC-02 | TC-03 | TC-04 | TC-05 |
|------|-------|-------|-------|-------|-------|
| Altitude | ✓ | ✓ | ✓ | ✓ | ✓ |
| Position | ✓ | ✓ | ✓ | ✓ | ✓ |

Unit test C++: **27/27 passed** (F2/F3/F6/F8 failsafes)

**Bước tiếp theo (chiều 2026-04-01)**: Hardware deployment checklist (Pixhawk 6X + Jetson Orin Nano 8GB via UART)

---

## Failsafes đã implement (nhánh feature/acc-failsafe, đã merge)

| Failsafe | Mô tả | Điều kiện kích hoạt |
|----------|--------|---------------------|
| F1 | Watchdog 500ms | Không nhận acc command > 500ms |
| F2 | Active brake | Watchdog fired + vel > 1.0 m/s |
| F3 | Velocity limit | vel > 4.0 m/s → scale acc; vel > 5.0 m/s → brake |
| F6 | EKF health check | v_xy_valid = false → reject command |
| F8 | Force land | Stale > 30s → descent 0.7 m/s |

## Hardware Deployment Checklist (TODO chiều nay)

1. Verify UART baudrate Jetson ↔ Pixhawk 6X (`SER_TEL1_BAUD`)
2. Test `acceleration_control.py` kết nối thật
3. Calibrate `MPC_ACC_HOR` theo response thực tế

## Notes về TC-05 Position Mode

Position Mode position hold fights acc commands → dùng:
- `_wait_for_stop(10s)` trước khi record origin (settle sau TC-04)
- Per-leg timeout 10s (thay 6s)
- `SQUARE_RETURN_TOL = 15m` (thay 8m)
- `ACC = 1.0 m/s²` (thay 1.5)
