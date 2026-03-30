# Thử Nghiệm `px4_inverse_rc` Trên PX4 SITL

## Tổng Quan

Thử nghiệm thư viện `px4_inverse_rc.py` trên mô phỏng PX4 SITL để kiểm chứng rằng các hàm ngược (giá trị vật lý → RC stick → MAVLink) hoạt động đúng trong thực tế — drone phản hồi đúng vận tốc/góc nghiêng mong muốn.

**Script:** `Tools/test_sitl_inverse_rc.py`  
**Thư viện:** `Tools/px4_inverse_rc.py`  
**Toán học:** `docs/px4_co_so_toan_hoc_ham_nguoc_rc.md`

---

## 1. Điều Kiện Thử Nghiệm

### Phần mềm

| Thành phần | Yêu cầu |
|------------|----------|
| PX4 SITL | Đang chạy (`make px4_sitl gazebo-classic`) |
| Python | ≥ 3.8 |
| pymavlink | `pip install pymavlink` |
| numpy | `pip install numpy` |

### Tham số PX4

Script tự động set `COM_RC_IN_MODE = 1`. Các tham số khác dùng giá trị mặc định:

| Tham số | Giá trị | Ý nghĩa |
|---------|---------|---------|
| `COM_RC_IN_MODE` | 1 | MavLink only — bắt buộc |
| `MPC_VEL_MANUAL` | 10.0 m/s | Vận tốc ngang tối đa |
| `MPC_MAN_TILT_MAX` | 35° | Góc nghiêng tối đa |
| `MPC_Z_VEL_MAX_UP` | 3.0 m/s | Vận tốc lên tối đa |
| `MPC_Z_VEL_MAX_DN` | 1.5 m/s | Vận tốc xuống tối đa |
| `MPC_MAN_Y_MAX` | 150 °/s | Tốc độ xoay tối đa |
| `MAN_DEADZONE` | 0.0 | Vùng chết stick |

### Kết nối

- Mặc định: `udp:127.0.0.1:14540` (PX4 SITL standard)
- Tần số gửi: 10 Hz
- Drone takeoff lên 10m trước khi bắt đầu test

---

## 2. Kịch Bản Thử Nghiệm

### Quy trình chung mỗi test case

```
1. Tính MAVLink (x,y,z,r) từ giá trị vật lý bằng px4_inverse_rc
2. Gửi MANUAL_CONTROL liên tục 10Hz
3. Chờ ổn định (2s)
4. Đo LOCAL_POSITION_NED + ATTITUDE trong 3s
5. Tính trung bình, so sánh với giá trị mong đợi
6. Hover reset 3s giữa các test
```

### Bảng test case

| TC | Mode | Input | MAVLink | Kết quả mong đợi | Dung sai |
|----|------|-------|---------|-------------------|----------|
| **TC1** | Position | Hover (0,0,0,0) | (0, 0, 500, 0) | \|v\| < 0.5 m/s | 0.5 m/s |
| **TC2** | Position | vx=3 m/s | (≈553, 0, 500, 0) | vx ≈ 3 m/s | ±1.0 m/s |
| **TC3** | Position | vy=2 m/s | (0, ≈411, 500, 0) | vy ≈ 2 m/s | ±1.0 m/s |
| **TC4** | Position | vz=-1 m/s (lên) | (0, 0, ≈606, 0) | vz ≈ -1 m/s | ±1.0 m/s |
| **TC5** | Position | vz=+0.5 m/s (xuống) | (0, 0, ≈394, 0) | vz ≈ +0.5 m/s | ±1.0 m/s |
| **TC6** | Position | yaw=30°/s | (0, 0, 500, ≈363) | yaw_rate ≈ 30°/s | ±15°/s |
| **TC7** | Position | vx=2 vy=1 vz=-0.5 yr=15 | (≈411, ≈189, ≈553, ≈189) | \|vh\|≈2.24 vz≈-0.5 | ±1.0 m/s |
| **TC8** | Altitude | pitch=10° | (≈251, 0, 500, 0) | vx > 0.5 m/s | drone tiến |

### Chi tiết từng test case

#### TC1: Hover — Kiểm tra ổn định
- **Mục đích:** Stick trung tâm → drone giữ vị trí
- **Tiêu chí pass:** Tổng vận tốc < 0.5 m/s

#### TC2: Bay tiến 3 m/s
- **Mục đích:** Kiểm tra `velocity_to_stick()` trục pitch
- **Tính toán:** ŷ = 3/10 = 0.3 → giải expo → s ≈ 0.553 → x = 553

#### TC3: Bay ngang 2 m/s
- **Mục đích:** Kiểm tra `velocity_to_stick()` trục roll
- **Tính toán:** ŷ = 2/10 = 0.2 → giải expo → s ≈ 0.411 → y = 411

#### TC4: Lên cao 1 m/s
- **Mục đích:** Kiểm tra `vz_to_throttle_stick()` hướng lên
- **Tính toán:** vz=-1 (NED) → stick_dir=+1 → ŷ = 1/3 → z > 500

#### TC5: Hạ xuống 0.5 m/s
- **Mục đích:** Kiểm tra `vz_to_throttle_stick()` hướng xuống
- **Tính toán:** vz=+0.5 (NED) → stick_dir=-0.5 → ŷ = -0.5/1.5 → z < 500

#### TC6: Xoay yaw 30°/s
- **Mục đích:** Kiểm tra `yaw_rate_to_stick()`
- **Tính toán:** ŷ = 30/150 = 0.2 → giải expo → s ≈ 0.363 → r = 363

#### TC7: Bay kết hợp 4 trục
- **Mục đích:** Kiểm tra hoạt động đồng thời tất cả trục
- **Tiêu chí:** Biên độ vận tốc ngang và vận tốc đứng đúng

#### TC8: Altitude Mode — Nghiêng pitch 10°
- **Mục đích:** Kiểm tra `tilt_to_stick()` (không có expo)
- **Tính toán:** s = tan(10°)/tan(35°) ≈ 0.251 → x = 251
- **Tiêu chí:** Drone di chuyển tiến (vx > 0.5)

---

## 3. Cách Chạy

### Bước 1: Khởi động PX4 SITL

```bash
# Terminal 1
cd PX4-Autopilot
make px4_sitl gazebo-classic
```

### Bước 2: Chạy test

```bash
# Terminal 2
python3 Tools/test_sitl_inverse_rc.py

# Hoặc với chuỗi kết nối khác
python3 Tools/test_sitl_inverse_rc.py --connection udp:localhost:14550

# Nếu drone đã bay, bỏ qua setup
python3 Tools/test_sitl_inverse_rc.py --skip-setup
```

### Bước 3: Đọc kết quả

Script in bảng kết quả cuối cùng, ví dụ:

```
==============================================================================
KẾT QUẢ THỬ NGHIỆM PX4 SITL — px4_inverse_rc
==============================================================================

✅ TC1: Hover
   Mong đợi : |v| < 0.5 m/s
   Thực tế  : |v| = 0.082 m/s (vx=0.03 vy=0.05 vz=0.04)
   Sai số   : 0.082 m/s

✅ TC2: Tiến vx=3 m/s
   Mong đợi : vx ≈ 3.0 m/s
   Thực tế  : vx = 2.87 m/s
   Sai số   : 0.13 m/s

...

TỔNG KẾT: 8/8 PASS ✅ TẤT CẢ OK
==============================================================================
```

---

## 4. Dung Sai & Nguồn Sai Số

| Nguồn | Độ lớn | Giải thích |
|-------|--------|------------|
| Lượng tử hóa MAVLink | ~0.01 m/s | Stick [-1,1] → int [-1000,1000] |
| Bộ điều khiển PX4 | ~0.5 m/s | PID tracking không hoàn hảo |
| Trễ hệ thống | ~0.2s | Trễ từ lệnh → phản hồi drone |
| Mô phỏng Gazebo | biến thiên | Gió, nhiễu vật lý mô phỏng |

**Dung sai chọn:**
- Vận tốc: ±1.0 m/s — đủ rộng cho sai số bộ điều khiển + mô phỏng
- Yaw rate: ±15°/s — yaw controller có tracking error lớn hơn
- Hover: |v| < 0.5 m/s — cho phép dao động nhỏ

---

## 5. Xử Lý Sự Cố

| Vấn đề | Nguyên nhân | Giải pháp |
|--------|-------------|-----------|
| Không kết nối được | SITL chưa chạy | Chạy `make px4_sitl gazebo-classic` trước |
| Arm thất bại | Pre-arm check fail | Kiểm tra GPS lock trong SITL |
| Drone không phản hồi | `COM_RC_IN_MODE ≠ 1` | Script tự set, nhưng kiểm tra lại |
| Sai số lớn | Tham số PX4 khác default | Kiểm tra `MPC_VEL_MANUAL` etc. |
| Timeout | Kết nối sai port | Thử `udp:localhost:14550` |

---

## 6. Tóm Tắt

- **8 test case** bao phủ: hover, 4 trục riêng lẻ, kết hợp, 2 flight mode
- **Kiểm chứng end-to-end:** giá trị vật lý → `px4_inverse_rc` → MAVLink → PX4 SITL → đo LOCAL_POSITION_NED
- **Tự động hóa:** 1 lệnh chạy, tự set param + arm + takeoff + test + báo cáo
- **Dung sai thực tế:** tính đến sai số bộ điều khiển và mô phỏng
