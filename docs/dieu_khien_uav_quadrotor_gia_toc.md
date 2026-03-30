# Điều Khiển UAV Quadrotor Bằng Gia Tốc trong PX4

> Phân tích kỹ thuật dựa trên codebase PX4-Autopilot (nhánh main, 2026)

---

## 1. Tổng Quan Kiến Trúc Điều Khiển

PX4 sử dụng kiến trúc điều khiển **phân tầng (cascaded control)** cho quadrotor:

```
Trajectory Setpoint
        │
        ▼
┌──────────────────────┐
│  Position Controller │  (mc_pos_control)
│  - P: vị trí → vận tốc
│  - PID: vận tốc → gia tốc
│  - Gia tốc → Thrust vector
└──────────┬───────────┘
           │ attitude_setpoint (quaternion + thrust)
           ▼
┌──────────────────────┐
│  Attitude Controller │  (mc_att_control)
│  - P: quaternion error → angular rate
└──────────┬───────────┘
           │ rate_setpoint
           ▼
┌──────────────────────┐
│  Rate Controller     │  (mc_rate_control)
│  - PID: angular rate → torque + thrust
└──────────┬───────────┘
           │ vehicle_thrust_setpoint
           │ vehicle_torque_setpoint
           ▼
┌──────────────────────┐
│  Control Allocator   │  (control_allocator)
│  - Phân bổ lực/moment → tốc độ từng động cơ
└──────────────────────┘
```

---

## 2. Điều Khiển Bằng Gia Tốc (Acceleration-Based Control)

### 2.1 Chuỗi xử lý gia tốc trong PositionControl

**File:** `src/modules/mc_pos_control/PositionControl/PositionControl.cpp`

Lớp `PositionControl` thực hiện điều khiển theo thứ tự:

#### Bước 1: Điều khiển vị trí (P controller)
```cpp
// P-position controller: vị trí error → velocity setpoint
Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
```
- Hệ số: `MPC_XY_P = 0.95` (ngang), `MPC_Z_P = 1.0` (dọc)
- Đơn vị: m/s corrective velocity per m position error

#### Bước 2: Điều khiển vận tốc (PID controller)
```cpp
// PID velocity control: velocity error → acceleration setpoint
Vector3f vel_error = _vel_sp - _vel;
Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p)
                         + _vel_int                         // I term
                         - _vel_dot.emult(_gain_vel_d);    // D term
```
- `MPC_XY_VEL_P_ACC = 1.8` m/s² per m/s
- `MPC_XY_VEL_I_ACC = 0.4` m/s² per m·s
- `MPC_XY_VEL_D_ACC = 0.2` m/s² per m/s²
- `MPC_Z_VEL_P_ACC = 4.0`, `MPC_Z_VEL_I_ACC = 2.0`, `MPC_Z_VEL_D_ACC = 0.0`

#### Bước 3: Chuyển đổi gia tốc → Thrust vector (_accelerationControl)
```cpp
// Tạo hướng thrust từ gia tốc mong muốn
Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);

// Chuyển gia tốc dọc thành lực đẩy
const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
_thr_sp = body_z * collective_thrust;
```

**Công thức chuyển đổi gia tốc → lực đẩy:**
```
T = a_sp * (Th_hover / g) - Th_hover
```
Trong đó:
- `T` = lực đẩy chuẩn hóa [-1, 0]
- `a_sp` = gia tốc setpoint (m/s²)
- `Th_hover` = hover thrust (MPC_THR_HOVER)
- `g` = 9.81 m/s²

#### Bước 4: Chuyển thrust vector → Attitude setpoint
```cpp
// thrustToAttitude: thrust vector + yaw → quaternion setpoint
void ControlMath::thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp,
                                    vehicle_attitude_setpoint_s &att_sp)
{
    bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
    att_sp.thrust_body[2] = -thr_sp.length();
}
```
Hướng body-z (trục đẩy) được xác định bởi gia tốc yêu cầu → tạo ma trận quay R_sp → quaternion.

### 2.2 Attitude Controller (Quaternion-based)

**File:** `src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`

Dựa trên nghiên cứu: *"Nonlinear Quadrocopter Attitude Control"* - ETH Zurich (2013)

```cpp
// Tách lỗi tilt và lỗi yaw, ưu tiên roll/pitch hơn yaw
const Vector3f eq = 2.f * qe.canonical().imag();  // sin(alpha/2) scaled rotation axis
Vector3f rate_setpoint = eq.emult(_proportional_gain);
```
- Đầu vào: quaternion hiện tại `q`, quaternion mục tiêu `qd`
- Đầu ra: angular rate setpoint [rad/s] theo body frame

### 2.3 FlightTask: Manual Acceleration Mode

**File:** `src/modules/flight_mode_manager/tasks/ManualAcceleration/FlightTaskManualAcceleration.cpp`

Chế độ **Position Mode** của pilot sử dụng `StickAccelerationXY`:

```cpp
// Stick input → acceleration setpoint
_acceleration_setpoint = stick_xy.emult(acceleration_scale);

// Áp dụng giới hạn jerk (đạo hàm của gia tốc)
applyJerkLimit(dt);
applyTiltLimit(_acceleration_setpoint);

// Tích phân gia tốc → velocity setpoint
_velocity_setpoint += _acceleration_setpoint * dt;
```

**File:** `src/modules/flight_mode_manager/tasks/Utility/StickAccelerationXY.cpp`

- Stick → acceleration → tích phân → velocity → position
- Có drag model để giới hạn tốc độ tự nhiên khi nhả stick
- Jerk limit: `MPC_JERK_MAX` để tránh giật cục

---

## 3. Setpoint Hierarchy (Thứ tự ưu tiên)

PX4 hỗ trợ thiết lập setpoint ở bất kỳ cấp nào. `NAN` = không điều khiển cấp đó:

| Cấp setpoint | Topic | Mô tả |
|---|---|---|
| Position | `trajectory_setpoint.position` | Bay đến điểm cố định |
| Velocity | `trajectory_setpoint.velocity` | Tốc độ feedforward |
| **Acceleration** | `trajectory_setpoint.acceleration` | Gia tốc trực tiếp |
| Thrust | Được tính từ gia tốc | Không đặt trực tiếp |

Nếu có cả position và velocity setpoint, velocity được dùng làm feedforward. Nếu chỉ có acceleration, nó được dùng trực tiếp (bỏ qua P/PID theo vị trí/vận tốc).

---

## 4. Hoạt Động Khi Mất GPS (GPS-Denied Flight)

### 4.1 Cơ chế ước tính trạng thái trong EKF2

**File:** `src/modules/ekf2/EKF/control.cpp`

EKF2 quản lý nhiều nguồn dữ liệu theo cơ chế fusion:

```cpp
void Ekf::controlFusionModes(const imuSample &imu_delayed)
{
    controlMagFusion();          // Từ trường → heading
    controlOpticalFlowFusion();  // Optical flow → velocity XY
    controlGpsFusion();          // GPS → position/velocity
    controlExternalVisionFusion(); // VIO/mocap → position/velocity
    controlHeightFusion();       // Baro/range/GPS → altitude
    controlFakePosFusion();      // Khi không có nguồn nào
    _zero_velocity_update.update(); // Khi đứng yên
}
```

### 4.2 Các chế độ hoạt động không GPS

#### A. Optical Flow (Cảm biến dòng quang học)
**File:** `src/modules/ekf2/EKF/aid_sources/optical_flow/optical_flow_control.cpp`

- Tham số: `EKF2_OF_CTRL = 1` để bật
- Cung cấp velocity XY từ camera downward-facing
- Cần kết hợp với rangefinder để đo độ cao
- Hoạt động tốt ở độ cao thấp (<10m), trong nhà

#### B. External Vision / VIO (Visual-Inertial Odometry)
**File:** `src/modules/ekf2/EKF/aid_sources/external_vision/ev_control.cpp`

- Tham số: `EKF2_EV_CTRL` (bit mask)
  - Bit 0: HPOS (vị trí ngang)
  - Bit 1: VPOS (độ cao)
  - Bit 2: VEL (vận tốc)
  - Bit 3: YAW (hướng)
- Nguồn: camera depth, T265, VIO pipeline bên ngoài
- Hỗ trợ frame NED hoặc body FRD

#### C. Fake Position Fusion (Chế độ không có nguồn nào)
**File:** `src/modules/ekf2/EKF/aid_sources/fake_pos_control.cpp`

Khi **không có bất kỳ nguồn horizontal aiding nào**:
```cpp
// Nếu đang bay, dùng noise cao để cho phép drift
obs_var(0) = obs_var(1) = sq(fmaxf(_params.ekf2_noaid_noise, 1.f));

// Giữ vị trí cuối cùng đã biết
const Vector2f innovation = (_gpos - _last_known_gpos).xy();
```
- UAV cố gắng giữ vị trí cuối cùng đã biết
- Vị trí sẽ drift theo thời gian (chỉ IMU dead-reckoning)

#### D. Zero Velocity Update (Khi đứng yên)
**File:** `src/modules/ekf2/EKF/aid_sources/ZeroVelocityUpdate.cpp`

```cpp
// Fuse vận tốc = 0 mỗi 200ms khi xe đứng yên
if (ekf.control_status_flags().vehicle_at_rest) {
    Vector3f vel_obs{0.f, 0.f, 0.f};
    ekf.fuseDirectStateMeasurement(innovation, innov_var, obs_var, State::vel.idx + i);
}
```

### 4.3 Bảng tổng hợp nguồn aiding theo kịch bản

| Kịch bản | Nguồn Horizontal | Nguồn Altitude | Chất lượng |
|---|---|---|---|
| GPS đầy đủ | GNSS | GPS/Baro | Tốt nhất |
| Trong nhà (thấp) | Optical Flow | Rangefinder | Tốt |
| Trong nhà (cao) | VIO/Mocap | Baro/EV | Tốt |
| Không có gì | Fake Pos (drift) | Barometer | Kém, drift |
| Đứng yên | Zero Vel Update | Barometer | Ổn định |

### 4.4 Flight Mode khi mất GPS

Khi GPS mất, hệ thống tự động chuyển chế độ:
- **Position Mode** → yêu cầu `xy_valid` và `v_xy_valid` từ EKF
- Nếu mất: chuyển sang **Altitude Mode** (chỉ giữ độ cao, không giữ vị trí XY)
- Nếu mất cả baro: **Stabilized Mode** (chỉ giữ thái độ)

```cpp
// MulticopterPositionControl.cpp
if (vehicle_local_position.xy_valid && position_xy.isAllFinite()) {
    states.position.xy() = position_xy;
} else {
    states.position(0) = states.position(1) = NAN;  // Bộ điều khiển tự bỏ qua
}
```

---

## 5. Tham Số Quan Trọng

### 5.1 Điều khiển vị trí/vận tốc/gia tốc

| Tham số | Giá trị mặc định | Ý nghĩa |
|---|---|---|
| `MPC_XY_P` | 0.95 | P gain vị trí ngang (m/s per m) |
| `MPC_Z_P` | 1.0 | P gain vị trí dọc |
| `MPC_XY_VEL_P_ACC` | 1.8 | P gain vận tốc ngang (m/s² per m/s) |
| `MPC_XY_VEL_I_ACC` | 0.4 | I gain vận tốc ngang |
| `MPC_XY_VEL_D_ACC` | 0.2 | D gain vận tốc ngang |
| `MPC_Z_VEL_P_ACC` | 4.0 | P gain vận tốc dọc |
| `MPC_ACC_HOR` | 3.0 | Gia tốc ngang tối đa (m/s²) |
| `MPC_ACC_UP_MAX` | 4.0 | Gia tốc lên tối đa |
| `MPC_ACC_DOWN_MAX` | 3.0 | Gia tốc xuống tối đa |
| `MPC_JERK_MAX` | 8.0 | Jerk tối đa (m/s³) |
| `MPC_THR_HOVER` | 0.5 | Hover thrust chuẩn hóa [0,1] |
| `MPC_TILTMAX_AIR` | 45° | Góc nghiêng tối đa khi bay |

### 5.2 Điều khiển thái độ/tốc độ góc

| Tham số | Ý nghĩa |
|---|---|
| `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P` | P gain thái độ |
| `MC_ROLLRATE_P/I/D` | PID gain tốc độ góc roll |
| `MC_PITCHRATE_P/I/D` | PID gain tốc độ góc pitch |
| `MC_YAW_WEIGHT` | Mức độ ưu tiên yaw vs roll/pitch [0,1] |

### 5.3 Nguồn dữ liệu EKF2

| Tham số | Ý nghĩa |
|---|---|
| `EKF2_GPS_CTRL` | GPS control bits (HPOS\|VEL mặc định) |
| `EKF2_OF_CTRL` | Optical flow: 0=tắt, 1=bật |
| `EKF2_EV_CTRL` | External vision control bits |
| `EKF2_HGT_REF` | Nguồn độ cao chính (0=baro, 1=gps, 2=rng, 3=ev) |
| `EKF2_NOAID_NOISE` | Noise khi không có horizontal aiding |

---

## 6. Luồng Dữ Liệu (Data Flow) Chi Tiết

```
[Pilot/Autopilot]
       │
       │ trajectory_setpoint (pos/vel/acc/yaw)
       ▼
[mc_pos_control @ ~50Hz]
  ├── set_vehicle_states() ← vehicle_local_position (từ EKF2)
  │     ├── Lọc velocity: notch filter + low-pass filter
  │     └── Tính vel_dot (acceleration từ diff velocity)
  │
  ├── PositionControl::update()
  │     ├── _positionControl() → vel_sp += P*(pos_sp - pos)
  │     ├── _velocityControl() → acc_sp += PID*(vel_sp - vel)
  │     └── _accelerationControl() → thr_sp = acc → thrust vector
  │
  └── publish: vehicle_attitude_setpoint
               vehicle_local_position_setpoint

[mc_att_control @ ~250Hz]
  ├── AttitudeControl::update(q_current)
  │     ├── Tính quaternion error: qe = q_inv * qd
  │     ├── rate_sp = 2 * qe.imag() * gain
  │     └── Ưu tiên tilt over yaw
  └── publish: vehicle_rates_setpoint

[mc_rate_control @ ~1000Hz]
  ├── RateControl::update(rate_current)
  │     └── PID: rate_error → torque setpoint
  └── publish: vehicle_thrust_setpoint
               vehicle_torque_setpoint

[control_allocator]
  └── Phân bổ → actuator_motors (tốc độ RPM từng động cơ)
```

---

## 7. Kết Luận

### Điểm mấu chốt về điều khiển gia tốc:
1. **Gia tốc là cầu nối** giữa vận tốc và lực đẩy/thái độ - đây là lớp điều khiển trung gian quan trọng nhất
2. **Chuyển đổi gia tốc → thái độ**: vector gia tốc mong muốn xác định hướng thrust, từ đó suy ra góc nghiêng cần thiết
3. **Hover thrust** là hệ số tỉ lệ giữa gia tốc và lực đẩy: `T = a * (Th_hover/g) - Th_hover`
4. **Feed-forward**: setpoint ở cấp cao hơn (vị trí) truyền xuống cấp thấp (gia tốc) dưới dạng feedforward

### Điểm mấu chốt khi mất GPS:
1. EKF2 tự động chuyển sang nguồn thay thế (optical flow, VIO)
2. Không có nguồn nào → fake position fusion, chỉ giữ vị trí cuối cùng (drift tăng dần)
3. Hệ điều khiển tự detect `xy_valid = false` và vô hiệu hóa vòng lặp vị trí XY
4. Optical flow + rangefinder là combo tốt nhất cho bay trong nhà không GPS

---

*Tài liệu tham khảo code:*
- `src/modules/mc_pos_control/PositionControl/PositionControl.cpp`
- `src/modules/mc_pos_control/PositionControl/ControlMath.cpp`
- `src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`
- `src/modules/mc_rate_control/MulticopterRateControl.cpp`
- `src/modules/flight_mode_manager/tasks/Utility/StickAccelerationXY.cpp`
- `src/modules/ekf2/EKF/control.cpp`
- `src/modules/ekf2/EKF/aid_sources/fake_pos_control.cpp`
- `src/modules/ekf2/EKF/aid_sources/optical_flow/optical_flow_control.cpp`
- `src/modules/ekf2/EKF/aid_sources/external_vision/ev_control.cpp`
