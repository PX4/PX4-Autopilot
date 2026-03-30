# Cơ Sở Toán Học Xây Dựng Hàm Ngược: Giá Trị Góc → RC Input Cho PX4

## Mục Tiêu

Xây dựng hàm chuyển đổi ngược từ giá trị vật lý mong muốn (góc nghiêng, vận tốc, tốc độ xoay) sang giá trị RC stick để gửi vào PX4 qua MAVLink `MANUAL_CONTROL` (msg #69), **không sửa firmware**.

Điều kiện tiên quyết: đặt tham số PX4 `COM_RC_IN_MODE = 1` (MavLinkOnly) hoặc `2` (Fallback).

---

## 1. Hàm Thuận Trong PX4 (Forward Functions)

PX4 xử lý stick input qua chuỗi 2 hàm trước khi dùng:

### 1.1. Hàm Deadzone

**Nguồn:** `src/lib/mathlib/math/Functions.hpp:124-133`

Cho stick value `s ∈ [-1, 1]` và deadzone `dz ∈ [0, 0.99)`:

$$
D(s, dz) = \begin{cases}
\dfrac{s - \text{sign}(s) \cdot dz}{1 - dz} & \text{nếu } |s| > dz \\[8pt]
0 & \text{nếu } |s| \leq dz
\end{cases}
$$

**Ý nghĩa:** Loại bỏ vùng chết quanh tâm stick, sau đó co giãn lại cho output vẫn đạt [-1, 1].

### 1.2. Hàm Expo

**Nguồn:** `src/lib/mathlib/math/Functions.hpp:83-89`

Cho giá trị `d ∈ [-1, 1]` và hệ số expo `e ∈ [0, 1]`:

$$
E(d, e) = (1 - e) \cdot d + e \cdot d^3
$$

**Ý nghĩa:** Pha trộn tuyến tính và bậc 3. Khi `e > 0`, vùng giữa ít nhạy hơn → dễ điều khiển tinh. Khi `e = 0`, hoàn toàn tuyến tính.

### 1.3. Hàm hợp Expo-Deadzone

**Nguồn:** `src/lib/mathlib/math/Functions.hpp:136-139`

$$
F(s) = E\Big(D(s,\; dz),\; e\Big) = (1-e) \cdot D(s, dz) + e \cdot D(s, dz)^3
$$

Với giá trị mặc định trong PX4:
- `e = 0.6` — hardcoded trong `src/lib/sticks/Sticks.hpp:67-70`
- `dz = MAN_DEADZONE` — tham số PX4, mặc định `0.0`

---

## 2. Các Chuỗi Biến Đổi Theo Flight Mode

Ký hiệu: `s` = stick value (đầu vào), `Y` = giá trị vật lý (đầu ra PX4 sử dụng).

### 2.1. Position Mode — Pitch/Roll (Stick → Vận tốc)

**Nguồn:** `src/modules/flight_mode_manager/tasks/ManualPosition/FlightTaskManualPosition.cpp:65-97`

Dùng `getPitchRollExpo()` → **CÓ expo**:

$$
Y_{vel} = F(s) \times V_{max}
$$

| Ký hiệu | Tham số PX4 | Mặc định | Ý nghĩa |
|----------|-------------|----------|---------|
| $V_{max}$ | `MPC_VEL_MANUAL` | 10.0 m/s | Vận tốc ngang tối đa |

### 2.2. Altitude Mode — Pitch/Roll (Stick → Góc nghiêng)

**Nguồn:** `src/modules/flight_mode_manager/tasks/Utility/StickTiltXY.cpp:52-68`

Dùng `getPitchRoll()` → **KHÔNG expo**, dùng stick raw:

$$
Y_{accel} = s \times a_{max}
$$

Trong đó:

$$
a_{max} = \text{clamp}\Big(\tan(\theta_{max}),\; 0.02,\; 3.0\Big) \times g
$$

Mà gia tốc ngang liên quan đến góc nghiêng bởi: $a = \tan(\theta) \times g$

Nên: $Y_{accel} = \tan(\theta) \times g = s \times \tan(\theta_{max}) \times g$

| Ký hiệu | Tham số PX4 | Mặc định | Ý nghĩa |
|----------|-------------|----------|---------|
| $\theta_{max}$ | `MPC_MAN_TILT_MAX` | 35° | Góc nghiêng tối đa |
| $g$ | — | 9.81 m/s² | Gia tốc trọng trường |

### 2.3. Throttle — Cả hai mode (Stick → Vận tốc đứng)

**Nguồn:** `src/modules/flight_mode_manager/tasks/ManualAltitude/FlightTaskManualAltitude.cpp:89-96`

Dùng `getThrottleZeroCenteredExpo()` → **CÓ expo**:

$$
Y_{vz} = -F(s) \times V_{z,max}
$$

Dấu trừ vì quy ước NED: Z dương = xuống, nhưng stick dương = lên.

| Ký hiệu | Tham số PX4 | Mặc định | Dùng khi |
|----------|-------------|----------|----------|
| $V_{z,max}$ | `MPC_Z_VEL_MAX_UP` | 3.0 m/s | stick > 0 (lên) |
| $V_{z,max}$ | `MPC_Z_VEL_MAX_DN` | 1.5 m/s | stick < 0 (xuống) |

### 2.4. Yaw — Cả hai mode (Stick → Tốc độ xoay)

**Nguồn:** `src/modules/flight_mode_manager/tasks/ManualAltitude/FlightTaskManualAltitude.cpp:283-287`

Dùng `getYawExpo()` → **CÓ expo**:

$$
Y_{yaw} = F(s) \times \omega_{max}
$$

| Ký hiệu | Tham số PX4 | Mặc định | Ý nghĩa |
|----------|-------------|----------|---------|
| $\omega_{max}$ | `MPC_MAN_Y_MAX` | 150 °/s | Tốc độ xoay yaw tối đa |

---

## 3. Xây Dựng Hàm Ngược (Inverse Functions)

### 3.1. Bài toán tổng quát

Cho giá trị vật lý mong muốn $Y$, tìm stick value $s$ sao cho chuỗi biến đổi thuận cho ra đúng $Y$.

### 3.2. Trường hợp CÓ expo (Position pitch/roll, Throttle, Yaw)

Chuỗi thuận: $Y = F(s) \times K$ với $K$ là hệ số scale ($V_{max}$, $V_{z,max}$, hoặc $\omega_{max}$).

**Bước 1 — Chuẩn hóa:** Tính output đã chuẩn hóa

$$
\hat{Y} = \frac{Y}{K} \quad \in [-1, 1]
$$

**Bước 2 — Giải ngược Expo:** Tìm $d$ từ $\hat{Y} = (1-e) \cdot d + e \cdot d^3$

Viết lại thành phương trình bậc 3:

$$
\boxed{e \cdot d^3 + (1-e) \cdot d - \hat{Y} = 0}
$$

Với `e = 0.6` mặc định:

$$
0.6 \cdot d^3 + 0.4 \cdot d - \hat{Y} = 0
$$

**Phương pháp giải:**

*Cách 1 — Công thức Cardano:*

Phương trình dạng depressed cubic $t^3 + pt + q = 0$ với:

$$
p = \frac{1-e}{e} = \frac{0.4}{0.6} = \frac{2}{3}, \quad q = \frac{-\hat{Y}}{e} = \frac{-\hat{Y}}{0.6}
$$

Discriminant: $\Delta = -4p^3 - 27q^2$

Khi $\Delta > 0$ (3 nghiệm thực, luôn đúng khi $|\hat{Y}| \leq 1$), dùng dạng lượng giác:

$$
d = 2\sqrt{\frac{-p}{3}} \cdot \cos\left(\frac{1}{3}\arccos\left(\frac{3q}{2p}\sqrt{\frac{-3}{p}}\right) - \frac{2\pi k}{3}\right), \quad k = 0, 1, 2
$$

Chọn nghiệm $d \in [-1, 1]$.

*Cách 2 — Newton-Raphson (thực dụng hơn):*

$$
d_{n+1} = d_n - \frac{e \cdot d_n^3 + (1-e) \cdot d_n - \hat{Y}}{3e \cdot d_n^2 + (1-e)}
$$

Khởi tạo: $d_0 = \hat{Y}$. Hội tụ sau 5-10 vòng lặp.

*Cách 3 — numpy.roots:*

```python
roots = numpy.roots([e, 0, (1-e), -Y_hat])
d = nghiệm thực trong [-1, 1]
```

**Bước 3 — Giải ngược Deadzone:** Tìm $s$ từ $d$

$$
\boxed{s = d \cdot (1 - dz) + \text{sign}(d) \cdot dz}
$$

Khi $dz = 0$ (mặc định): $s = d$.

### 3.3. Trường hợp KHÔNG expo (Altitude Mode pitch/roll)

Chuỗi thuận: $\tan(\theta) \times g = s \times \tan(\theta_{max}) \times g$

Rút gọn:

$$
\boxed{s = \frac{\tan(\theta)}{\tan(\theta_{max})}}
$$

Không cần giải phương trình bậc 3. Công thức trực tiếp.

---

## 4. Tổng Hợp Công Thức Ngược

### 4.1. Bảng công thức theo Mode và Trục

| Mode | Trục | Input người dùng | Công thức ngược tìm $s$ |
|------|------|-----------------|------------------------|
| **Position** | Pitch | $v_x$ (m/s) | $s = F^{-1}\!\left(\dfrac{v_x}{V_{max}}\right)$ |
| **Position** | Roll | $v_y$ (m/s) | $s = F^{-1}\!\left(\dfrac{v_y}{V_{max}}\right)$ |
| **Altitude** | Pitch | $\theta_p$ (°) | $s = \dfrac{\tan(\theta_p)}{\tan(\theta_{max})}$ |
| **Altitude** | Roll | $\theta_r$ (°) | $s = \dfrac{\tan(\theta_r)}{\tan(\theta_{max})}$ |
| **Cả hai** | Throttle | $v_z$ (m/s) | $s = F^{-1}\!\left(\dfrac{-v_z}{V_{z,max}}\right)$ |
| **Cả hai** | Yaw | $\omega$ (°/s) | $s = F^{-1}\!\left(\dfrac{\omega}{\omega_{max}}\right)$ |

Trong đó $F^{-1}$ là hàm ngược expo-deadzone (giải bậc 3 + ngược deadzone).

### 4.2. Chuyển Stick → MAVLink

**Nguồn:** `src/modules/mavlink/mavlink_receiver.cpp:2102-2113`

| Trục | Stick $s$ | MAVLink value | Công thức |
|------|-----------|---------------|-----------|
| Pitch | $s \in [-1, 1]$ | $x \in [-1000, 1000]$ | $x = \text{round}(s \times 1000)$ |
| Roll | $s \in [-1, 1]$ | $y \in [-1000, 1000]$ | $y = \text{round}(s \times 1000)$ |
| Throttle | $s \in [-1, 1]$ | $z \in [0, 1000]$ | $z = \text{round}((s + 1) \times 500)$ |
| Yaw | $s \in [-1, 1]$ | $r \in [-1000, 1000]$ | $r = \text{round}(s \times 1000)$ |

> Throttle khác biệt vì MAVLink quy định range [0, 1000] (không âm), còn PX4 nội bộ cần [-1, 1] (zero-centered).

---

## 5. Ví Dụ Tính Toán

### Ví dụ 1: Position Mode — Bay tiến 5 m/s

**Cho:** $v_x = 5$ m/s, $V_{max} = 10$ m/s, $e = 0.6$, $dz = 0$

**Bước 1:** $\hat{Y} = 5 / 10 = 0.5$

**Bước 2:** Giải $0.6 d^3 + 0.4 d - 0.5 = 0$

| $d$ | $0.4d + 0.6d^3$ | So sánh |
|-----|-----------------|---------|
| 0.65 | 0.260 + 0.165 = 0.425 | thấp |
| 0.70 | 0.280 + 0.206 = 0.486 | gần |
| **0.711** | **0.284 + 0.216 = 0.500** | ✅ |

**Bước 3:** $dz = 0$ → $s = d = 0.711$

**MAVLink:** $x = 0.711 \times 1000 = 711$

**Kiểm chứng:** PX4 tính $F(0.711) = 0.4 \times 0.711 + 0.6 \times 0.711^3 = 0.500$ → $v = 0.500 \times 10 = 5.0$ m/s ✅

### Ví dụ 2: Altitude Mode — Nghiêng 15° pitch, hạ 1 m/s, xoay 45°/s, có deadzone 0.05

**Cho:** $\theta_p = 15°$, $\theta_{max} = 35°$, $v_z = +1$ m/s (xuống), $\omega = 45$ °/s, $dz = 0.05$

**Pitch** (không expo):

$$
s_{pitch} = \frac{\tan(15°)}{\tan(35°)} = \frac{0.2679}{0.7002} = 0.383
$$

$x = 383$

**Throttle** (có expo + deadzone):

$\hat{Y} = -v_z / V_{dn} = -1 / 1.5 = -0.667$ (âm vì hạ = stick kéo xuống)

Giải $|0.6 d^3 + 0.4 d| = 0.667$ → $|d| = 0.825$ → $d = -0.825$

Ngược deadzone: $s = -0.825 \times (1 - 0.05) + (-1) \times 0.05 = -0.784 - 0.05 = -0.834$

$z = (-0.834 + 1) \times 500 = 83$

**Yaw** (có expo + deadzone):

$\hat{Y} = 45 / 150 = 0.3$

Giải $0.6 d^3 + 0.4 d = 0.3$ → $d = 0.529$

Ngược deadzone: $s = 0.529 \times 0.95 + 0.05 = 0.503 + 0.05 = 0.553$

$r = 553$

**Kết quả:**

```python
manual_control_send(x=383, y=0, z=83, r=553)
```

---

## 6. Tham Số PX4 Cần Biết

Toàn bộ hằng số trong công thức đều lấy từ code PX4. Phải đọc đúng giá trị trên drone trước khi tính.

| Tham số | Mặc định | Ảnh hưởng | File nguồn |
|---------|----------|-----------|------------|
| `e` (expo) | 0.6 | Đường cong expo | `Sticks.hpp:67` — hardcoded |
| `MAN_DEADZONE` | 0.0 | Vùng chết stick | Tham số PX4 |
| `MPC_VEL_MANUAL` | 10.0 m/s | $V_{max}$ Position Mode | `FlightTaskManualPosition.cpp:84` |
| `MPC_MAN_TILT_MAX` | 35° | $\theta_{max}$ Altitude Mode | `StickTiltXY.cpp:57` |
| `MPC_Z_VEL_MAX_UP` | 3.0 m/s | $V_{z,max}$ khi lên | `FlightTaskManualAltitude.cpp:92` |
| `MPC_Z_VEL_MAX_DN` | 1.5 m/s | $V_{z,max}$ khi xuống | `FlightTaskManualAltitude.cpp:93` |
| `MPC_MAN_Y_MAX` | 150 °/s | $\omega_{max}$ yaw | `FlightTaskManualAltitude.cpp:286` |

Đọc param từ PX4 bằng MAVLink:

```python
master.mav.param_request_read_send(1, 1, b'MPC_VEL_MANUAL', -1)
msg = master.recv_match(type='PARAM_VALUE', blocking=True)
V_max = msg.param_value
```

---

## 7. Trục Nào Dùng Công Thức Nào Và Tại Sao

Sự khác biệt do PX4 gọi hàm khác nhau trong `FlightTaskManualAltitude.cpp`:

```
_updateXYSetpoint()  → _sticks.getPitchRoll()     → RAW (không expo)
_updateYawSetpoint() → _sticks.getYawExpo()        → CÓ expo
_scaleSticks()       → _sticks.getThrottleExpo()    → CÓ expo
```

Còn `FlightTaskManualPosition.cpp` override:

```
_scaleSticks()       → _sticks.getPitchRollExpo()   → CÓ expo (khác Altitude!)
```

| Trục | Altitude Mode | Position Mode | Lý do |
|------|--------------|---------------|-------|
| **Pitch/Roll** | Raw — `getPitchRoll()` | Expo — `getPitchRollExpo()` | Altitude cần ánh xạ **tuyến tính** ra góc nghiêng vật lý. Position cần **mượt** vì ánh xạ ra vận tốc |
| **Throttle** | Expo | Expo | Cần hover ổn định quanh tâm |
| **Yaw** | Expo | Expo | Cần xoay chậm mượt quanh tâm |

→ **Hệ quả:** Altitude pitch/roll dùng công thức $\tan$ trực tiếp, còn Position pitch/roll và throttle/yaw phải giải phương trình bậc 3.

---

## 8. Tham Khảo Mã Nguồn PX4

| Công thức | File |
|-----------|------|
| `expo()`, `deadzone()`, `expo_deadzone()` | `src/lib/mathlib/math/Functions.hpp:83-139` |
| Sticks class, expo = 0.6 | `src/lib/sticks/Sticks.hpp:67-70` |
| MAVLink MANUAL_CONTROL → stick | `src/modules/mavlink/mavlink_receiver.cpp:2102-2113` |
| COM_RC_IN_MODE selector | `src/modules/manual_control/ManualControlSelector.cpp:63-115` |
| Position: stick → velocity | `src/modules/flight_mode_manager/tasks/ManualPosition/FlightTaskManualPosition.cpp:65-97` |
| Altitude: stick → acceleration | `src/modules/flight_mode_manager/tasks/Utility/StickTiltXY.cpp:52-68` |
| Throttle: stick → vz | `src/modules/flight_mode_manager/tasks/ManualAltitude/FlightTaskManualAltitude.cpp:89-96` |
| Yaw: stick → yaw rate | `src/modules/flight_mode_manager/tasks/ManualAltitude/FlightTaskManualAltitude.cpp:283-287` |

---

*Tài liệu dựa trên phân tích mã nguồn PX4 Autopilot — Không sửa đổi firmware*
