# THROUGH Mode — 實作前驗證（M0）

- 基準：PX4-Autopilot `v1.17.0`（`git describe` = `v1.17.0`），Ubuntu 22.04，gz sim 8.15.0（Harmonic）
- 驗證日期：2026-10-27
- 方法：原始碼查證 + `px4_sitl_default` 實際編譯 + headless `gz_x500_gimbal` runtime 量測（標示 **[實測]**）
- 檔案路徑皆相對於 `PX4-Autopilot/`

---

## 0. 結論摘要

| 編號 | 結論 | 對規格的影響 |
|---|---|---|
| V1 | 外部模式註冊流程與 `mc_nn_control` 相同；`health_component_index = 0`、`num_events = 0` 確認。註冊 / 註銷**只在 disarmed 時生效**。 | 小：註銷只在地面有意義；arming check 回覆有 50 ms 期限 |
| V2 | `VEHICLE_CMD_SET_NAV_STATE`（100001）存在，param1 = nav_state。 | 無 |
| V3 | QGC 透過 `AVAILABLE_MODES` 顯示名稱；RC 用 `COM_FLTMODEx = 100..107`；槽位由 `COM_MODEx_HASH` 固定。**SITL 沒有 RC，mode slot 只吃 `SOURCE_RC`**。 | 中：「從 RC 切入」在 SITL 需要搖桿或模擬 RC（見 V3） |
| V4 | 旗標組合下只有 THROUGH 發布 `vehicle_attitude_setpoint`；`thrust_body = [0,0,-T]`；`q_d` 為 FRD→NED Hamilton。 | 無 |
| V5 | **gimbal 回授只有 5 Hz [實測]**；gz gimbal 不自我穩定，角度是相對機身的關節角；pitch 負值 = 往下 [實測]。 | **大：第 7 節 Step 1 的「優先使用 gimbal 回授」不可行，需處置（D1）** |
| V6 | 用 `DO_GIMBAL_MANAGER_CONFIGURE` 取得控制權，再發 `gimbal_manager_set_attitude`；**`angular_velocity` 必須填 NaN**，否則 gz 不動 [實測]。 | 中：需在模組內自行換算成機體座標 |
| V7 | PX4 SITL 時鐘 = gz sim time；預設 `UXRCE_DDS_SYNCT=1` 會把 sim-time 戳記毀掉。 | 中：建議 `UXRCE_DDS_SYNCT=0`（D2） |
| V8 | 新 msg 放 `msg/`（非版本化）即可，topic 無 `_vN` 後綴。 | 無 |
| V9 | `module.yaml` 參數、`rc.mc_apps` 啟動、airframe 需登記在 CMakeLists。 | 無 |
| V10 | `hover_thrust_estimate.valid` 可用；x500 的 `MPC_THR_HOVER = 0.60`。**M2 更正：x500_gimbal / x500_through 實際 hover thrust ≈ 0.81 [實測]**，見第 12 節。 | **大：`THRU_THR_BASE = 0.70` 低於懸停推力，M5 前需處置（D7）**；鎖存 T_h（D4） |

**決策**：D1–D6 已確認採用建議方案（D1 = 方案 A），見第 11 節；D7 = A（12.3 節）；D8 = A（環與起點上移 5 m，15.4 節）；D9 = C（17.4 節）。

---

## V1 外部模式註冊

### 查證檔案
- `msg/versioned/RegisterExtComponentRequest.msg`、`RegisterExtComponentReply.msg`、`UnregisterExtComponent.msg`、`ArmingCheckRequest.msg`、`ArmingCheckReply.msg`、`VehicleControlMode.msg`
- `src/modules/mc_nn_control/mc_nn_control.cpp:132-210, 435-480`（範本）
- `src/modules/commander/ModeManagement.{hpp,cpp}`
- `src/modules/commander/HealthAndArmingChecks/checks/externalChecks.{hpp,cpp}`
- `src/modules/commander/Commander.cpp:2600`（`updateControlMode`）、`UserModeIntention.cpp`

### 欄位與流程
1. **Request**（`register_ext_component_request`，queue 2）：`name` char[25]（`"THROUGH"`）、`request_id` uint64（自選）、`px4_ros2_api_version = 1`（`LATEST_PX4_ROS2_API_VERSION`）、`register_arming_check = true`、`register_mode = true`（註冊 mode 必須同時註冊 arming check）、`enable_replace_internal_mode = false`。
2. **Reply**（`register_ext_component_reply`，queue 2）：比對 `request_id` 與 `success`，取得 `arming_check_id`（0..7）與 `mode_id`（nav_state 23..30 = EXTERNAL1..8）。範本最多讀 `ORB_QUEUE_LENGTH` 筆。
3. **只在 disarmed 時處理**：`ModeManagement::update()` 在 armed（且 `allowUpdateWhileArmed()` 為 false）時，會直接以 `success=false` 拒絕註冊；**註銷也只在 disarmed 分支處理**。
   → 影響：模組結束時送 `unregister_ext_component`（`mode_id` + `arming_check_id` + `name`），只在地面有效。若註銷時模式正在使用中，commander 會切到 Hold。
4. **槽位固定**：`Modes::addExternalMode()` 以名稱 hash 存進 `COM_MODE{0..7}_HASH`，同名模式重開機後拿到同一個 EXTERNALn（RC 綁定可以穩定使用）。
5. **`config_control_setpoints`**：commander 以 `source_id == mode_id` 找到模式後**整個 struct 複製**，再覆寫 `flag_armed`，並計算 `flag_multicopter_position_control_enabled = alt || climb || pos || vel || accel`。註冊後、尚未收到 config 前，預設值是**全部控制旗標 = true**（`Mode::setControlModeDefaults`），所以註冊成功後必須馬上發布。
   THROUGH 要發布的值：
   - `attitude = rates = allocation = true`
   - `manual = auto = offboard = position = velocity = altitude = climb_rate = acceleration = false`
   - `termination = false`（`mc_nn_control` 設成 true，但 src/ 內沒有任何讀者；在 TERMINATION 模式的語意是「關閉所有控制器」，所以不照抄）
   - `flag_armed`、`flag_multicopter_position_control_enabled` 由 commander 覆寫，填什麼都沒差
6. **Arming check 時序**（`externalChecks.hpp:69-75`）：commander 每 **300 ms** 發一次 `arming_check_request`。回覆必須在 **50 ms** 內送達，`request_id` 要相符，且 `reply.timestamp` 與現在相差 < 300 ms。連續 3 次沒回（剛註冊時是 10 次）會標成 unresponsive，設定 `mode_req_other`；如果模式正在使用中就會觸發 failsafe（不會移除使用中的模式）。
   → 影響：收到 request 後要在同一個週期立即回覆。100 Hz 主迴圈足夠，但 request 要每個週期都檢查。
7. **`can_arm_and_run = false`** → 該 nav_state 的 `mode_req_other` 被設定，並清除該模式的 arming bits → `canRun()` 為 false → **armed 時** `UserModeIntention::change()` 拒絕切換。**disarmed 時一律允許選擇模式**（再由 `mode_req_prevent_arming` 擋下解鎖）。
   → 條件一的狀態最多會有約 300 ms 延遲才反映到 commander（請求週期）。
8. **`health_component_index = 0`（`HEALTH_COMPONENT_INDEX_NONE`）、`num_events = 0`**：與範本一致，確認。
9. **模式需求旗標**：`mode_req_angular_velocity`、`mode_req_attitude`、`mode_req_local_alt`、`mode_req_local_position` = true（與範本相同），`mode_req_prevent_arming = true`（規格要求），其餘 false。
10. **能否附帶原因文字**：可以，但需要 metadata。reply 最多可帶 5 個 `event_s`，commander 會用 `Report::addExternalEvent()` 轉發給 GCS（並把 modes 寫進 arguments 開頭）。QGC 能顯示文字的前提是事件 ID 存在於 `events/all_events.json`，而這份 json 由 `Tools/px4events/srcparser.py` 掃描原始碼產生。parser 支援 `reporter.armingCheckFailureExt(events::ID("..."), events::Log::Error, "訊息")` 這種呼叫形式（原本是給 ROS 2 用的）。
    → 做法：在 `ModeRegistration` 裡寫一個同名、同簽章的 helper 來填 `event_s`，讓 parser 擷取到文字。實際顯示效果在 M2 驗證。
    → 備援：QGC（compid 190）送 `DO_SET_MODE` 被拒時，commander 本來就會送出 `"Switching to THROUGH is currently not available"`（`Commander::printRejectMode`）。

---

## V2 模組內切出到 Hold

- `msg/versioned/VehicleCommand.msg:114`：`VEHICLE_CMD_SET_NAV_STATE = 100001`，param1 = nav_state。
- `Commander.cpp:936`：收到後呼叫 `_user_mode_intention.change(param1, source)`，會檢查 `canRun()`，ack 結果為 `ACCEPTED` 或 `TEMPORARILY_REJECTED`。
- 指令過濾（`Commander.cpp:746`）：`target_system` 必須是自己的 sysid 或 0，`target_component` 是自己的 compid 或 0。
- `source_component < COMPONENT_MODE_EXECUTOR_START (1000)` → 視為 `ModeChangeSource::User`。
- **參數**：Hold = `NAVIGATION_STATE_AUTO_LOITER = 4`，Position = `NAVIGATION_STATE_POSCTL = 2`。
- 發布到 uORB `vehicle_command`，`from_external = false`，`source_system = MAV_SYS_ID`，`source_component = MAV_COMP_ID`，`target_system = MAV_SYS_ID`，`target_component = 0`。再從 `vehicle_command_ack` 確認結果（`command == 100001`）。
- 也可以用 `DO_SET_MODE`（`base_mode` CUSTOM、main AUTO、sub LOITER），但 `SET_NAV_STATE` 比較直接，**採用 `SET_NAV_STATE`**。

---

## V3 QGC 動態模式列表與 RC

- `src/modules/mavlink/streams/AVAILABLE_MODES.hpp`：名稱來自 `register_ext_component_reply` 的訂閱。uORB 晚訂閱者只會拿到最後一筆（`uORBDeviceNode.cpp:454`）。rcS 中 mavlink（第 360 行）比 `rc.mc_apps`（第 289 行）晚啟動，但只有 THROUGH 一個外部模式時，最後一筆就是 THROUGH 的 reply，所以能拿到名稱。模式未註冊時顯示 `"(Mode not available)"`。串流頻率 0.3 Hz，另外有 dynamic update 機制。
- QGC 切換：`DO_SET_MODE`，custom main mode AUTO、sub mode `PX4_CUSTOM_SUB_MODE_EXTERNAL1..8` → nav_state 23..30（`Commander.cpp:863`）。
- RC：`ManualControl::navStateFromParam()` 中 `COM_FLTMODEx = 100..107` → EXTERNAL1..8。用 `commander status` 可看到 `External Mode N`。
- **SITL 限制**：`ManualControl.cpp:172` 只有在 `data_source == SOURCE_RC` 時才處理 mode slot。SITL 沒有 RC，QGC 虛擬搖桿 / MANUAL_CONTROL 不算 RC，所以 `COM_FLTMODE*` 在 SITL 無法觸發。替代方案是 QGC 的搖桿按鈕動作（可直接指定模式，文件 `docs/en/ros2/px4_ros2_control_interface.md:707` 有提到）。→ D6。
- QGC 版本：文件建議用 **QGC Daily**（支援動態模式列表）。本機 `~/qgc/QGroundControl.AppImage` 是 2024-06 版，M2 時實測；不行就換 Daily 或 v5 穩定版。
- **結論**：機制可行；QGC 顯示與切換在 M2 實測。

---

## V4 `mc_att_control` 與 `vehicle_attitude_setpoint`

- `mc_att_control_main.cpp:290-320`：`flag_control_attitude_enabled` 且為 rotary wing 時才執行。只接受 `timestamp > _last_attitude_setpoint` 的新 setpoint；**沒有新 setpoint 時沿用上一筆**（EXIT 階段停發時會維持最後姿態）。
- 只有在 `manual && !alt && !vel && !pos` 時才會由搖桿產生 setpoint → THROUGH 設 `manual = false` 就不會發生。
- 其他發布者：
  - `mc_pos_control`（`MulticopterPositionControl.cpp:452, 612`）只在 `flag_multicopter_position_control_enabled` 時發布；這個旗標由 commander 依 alt/climb/pos/vel/accel 計算 → 全關即不發布。
  - `mavlink_receiver`（SET_ATTITUDE_TARGET）只在 `nav_state == OFFBOARD` 時發布。
  - `flight_mode_manager` 只訂閱，不發布。
  - VTOL / FW / UUV / spacecraft 在 x500 上不會啟動。
  → **只有 THROUGH 發布。**
- `VehicleAttitudeSetpoint.msg`：`thrust_body` 為 FRD 正規化 [-1, 1]，多旋翼 `[0, 0, -T]`，T ∈ [0, 1]。與第 6 節一致。
- `q_d`：與 `vehicle_attitude.q` 相同約定（Hamilton，FRD body → NED）。
- `ControlMath::bodyzToAttitude(Vector3f body_z, float yaw_sp, vehicle_attitude_setpoint_s &)`：`body_z` 是世界座標下期望的機體 z 軸方向（`mc_pos_control` 傳入 `-thr_sp`）→ 規格的 `body_z = -f/|f|` 正確。函式位於 `src/modules/mc_pos_control/PositionControl/ControlMath.cpp`，CMake library 名稱為 `PositionControl`。
- 推力單位：`hover_thrust_estimate` 與 `MPC_THR_HOVER` 和 `thrust_body` 用同一種正規化單位（`mc_hover_thrust_estimator` 直接用 `vehicle_thrust_setpoint`）。
- **land detector 副作用**（`MulticopterLandDetector.cpp:159-290`）：climb_rate 關閉時，`maybe_landed` 需要 `thrust ≤ MPC_MANTHR_MIN + 0.01 = 0.09` 且無旋轉、無垂直/水平移動。`THRU_THR_MIN = 0.30` 遠高於此門檻 → 空中不會誤判著陸。（`ground_contact` 的 low-throttle 門檻 ≈ 0.12 + (0.6 − 0.12)·0.6 ≈ 0.41，0.30 會低於它，但還需要「無移動」條件，俯衝中不會成立。）

---

## V5 `gz_x500_gimbal`

### 查證檔案
`Tools/simulation/gz/models/{x500_gimbal,gimbal}/model.sdf`、`ROMFS/px4fmu_common/init.d-posix/airframes/4019_gz_x500_gimbal`、`src/modules/simulation/gz_bridge/GZGimbal.{hpp,cpp}`

### 相機
| 項目 | 值 |
|---|---|
| gz topic | `/world/<world>/model/x500_gimbal_0/link/camera_link/sensor/camera/image`、`.../camera_info` [實測] |
| 解析度 / 格式 | 1280×720 R8G8B8 [實測] |
| HFOV | 2.0 rad（114.6°）；`fx = fy = 410.94`、`cx = 640`、`cy = 360`、無畸變 [實測] |
| update rate | SDF 30 Hz；實測約 29.7 Hz（sim time 157 s 時 seq 4662）[實測] |
| header.stamp | sim time，`frame_id = camera_link` [實測] |
| headless render | 本機 WSL2 可正常產生影像 [實測] |
| 模型實例名 | `<model>_<instance>`，例如 `x500_gimbal_0`（`px4-rc.gzsim:110`） |

### Gimbal 機構
- joint（gimbal 模型繞 z 旋轉 180° 掛載）：yaw 無限制、roll ±45°、pitch `[-2.356, +0.785]` rad。
- 每軸由 `JointPositionController`（PID）驅動，`cmd_max = 0.3` N·m。
- **不會自我穩定**：`GZGimbal::pollSetpoint()` 把 `gimbal_device_set_attitude.q` 轉成 Euler 後**直接當作相對機身的關節角**送出，LOCK flags 沒有處理（程式碼註解 `//TODO handle device flags`）。PX4 `gimbal` 模組在 v2 輸出模式下也不做姿態補償（`OutputMavlinkV2::update` 不呼叫 `_calculate_angle_output`）。→ **pitch 指令是機體座標。**
- **pitch 正負號**：負值 = 鏡頭往下（PX4 FRD 約定）[實測：−45 → −45.0、+30 → +30.0、−90 → −89.9]。**−120° 不可用**：Euler 分解後 roll/yaw 翻轉 180°，最後停在 −37.9° [實測]。
  → 可用行程約 +45°（上）到 −90°（下），**涵蓋規格要求的 +30° 到 −90°，不需修改 joint limit**。模組內部「正值 = 往下」，介面處取反。
- **響應** [實測，`camera_imu` 250 Hz]：0 → −60° 步階，63% 0.22 s、90% 0.40 s、98% 0.61 s，無過衝。再加上 `GZGimbal` 5 Hz 排程，最多 200 ms 的指令延遲。

### 姿態回授
- `gimbal_device_attitude_status` 只在 `MNT_MODE_OUT = 2` 時由 `GZGimbal` 發布。內容來自 gz `camera_imu` 的 orientation，經 ENU/FLU → NED/FRD 轉換，**實際上是地球座標的完整相機姿態**（包含 yaw）。但 `device_flags` 只填了 `YAW_IN_VEHICLE_FRAME (32)`，這個旗標與內容不一致，不能依旗標判讀。
- **頻率：uORB 5 Hz、MAVLink 1 Hz [實測]**。原因是 `GZGimbal` 用 `ScheduleOnInterval(200_ms)`，發布 `timestamp = hrt_absolute_time()`，沒有 `timestamp_sample`。
- gz 端的 `camera_imu` 本身是 250 Hz [實測]。

### 對規格的影響（重大）
第 7 節 Step 1 要「優先由 gimbal 姿態回授取得、並對應影像擷取時刻」。5 Hz 而且沒有取樣時間戳的回授做不到。M4 要求 `los_ned` 誤差 < 1.5°，但 gimbal 移動時 200 ms 的回授間隔可能造成數度誤差。處置方案見 **D1**。

---

## V6 Gimbal 控制權與 pitch 指令

### 查證檔案
`src/modules/gimbal/{input_mavlink,output,output_mavlink,gimbal}.cpp`、`gimbal_params.c`、`src/modules/flight_mode_manager/tasks/Utility/Gimbal.cpp`（範本）

- airframe 4019 的設定：`MNT_MODE_IN = 4`（MAVLink gimbal v2）、`MNT_MODE_OUT = 2`（v2）、`MNT_RANGE_PITCH = 180`。`x500_through` 沿用即可，**不需要新增 `MNT_*` 參數**。
- **取得控制權**：發布 `vehicle_command`，`DO_GIMBAL_MANAGER_CONFIGURE`，`param1 = MAV_SYS_ID`、`param2 = MAV_COMP_ID`、`param3 = param4 = -1`，`source_system/component` 與 param 相同。範本是 `Gimbal::acquireGimbalControlIfNeeded()`，會讀 `gimbal_manager_status` 判斷是否已持有。
- **指令**：`gimbal_manager_set_attitude`，`origin_sysid/compid` 必須等於 primary control。`flags = 0`（不設 LOCK，等同機體座標）、`q = Euler(0, pitch_body, 0)`（yaw 0 = 跟隨機身）。
  - **`angular_velocity_x/y/z` 必須是 NaN**：若為 0，`GZGimbal::computeJointSetpoint()` 會回傳上一個角度，gimbal 不會動 [實測：`gimbal test pitch` 在角度模式下留下 rate = 0，所以完全不動；改用 PITCHYAW 並給 NaN rate 後正常]。
- **釋放**：`DO_GIMBAL_MANAGER_CONFIGURE` 帶 `param1 = param2 = -3`（範本 `releaseGimbalControlIfNeeded()`）。
- gimbal 模組主迴圈約 50 Hz（`gimbal_device_set_attitude` 實測 50 Hz），但 gz 端只有 5 Hz（V5）。
- **座標換算**（因為 gz gimbal 不穩定）：期望的相機光軸方向（機體座標）`d_b = R_nb^T · u`。在 roll 關節為 0、yaw 跟隨的情況下，光軸只能在機體 x–z 平面轉動，所以 `pitch_body_down = atan2(d_b.z, d_b.x)`（內部正值 = 往下），送出時用 `pitch_iface = -pitch_body_down`。第 7 節的 `el_des = asin(u[2])` 用於地球座標的狀態記錄；送出的指令改用上式（屬於規格允許的「若只接受機體座標則轉換」）。
- **風險**：QGC 隨時可以用 CONFIGURE 搶走控制權 → THROUGH 執行中要監看 `gimbal_manager_status.primary_control_*`，被搶時重新取得（或視為異常）。啟動後 primary control 實測為 1/1。

---

## V7 時間同步與 QoS

### 查證檔案
`src/modules/simulation/gz_bridge/GZBridge.cpp:331`、`src/modules/uxrce_dds_client/{uxrce_dds_client.cpp,utilities.hpp,dds_topics.h.em,module.yaml}`、`Tools/msg/templates/ucdr/msg.h.em:156-170`

- **PX4 SITL 時鐘 = gz sim time**：`GZBridge::clockCallback` 每收到一個 clock 訊息就用 `px4_clock_settime(CLOCK_MONOTONIC)` 設定，所以 `hrt_absolute_time()` 就是 sim time（µs）。
- **進來的時間戳轉換**：名稱為 `timestamp` / `timestamp_sample` 的 uint64 欄位，0 → `hrt_absolute_time()`；否則 → `min(t − time_offset, now)`。`time_offset` 是 agent OS 時鐘與 PX4 時鐘的差。
- `UXRCE_DDS_SYNCT` 預設 **1**（`module.yaml:88`）：offset ≈ 牆鐘 epoch − sim time。如果 ROS 節點用 `use_sim_time`，影像 header 是 sim time，減掉一個很大的 offset 後會溢位，被 clamp 成 now → **`timestamp_sample` 變得沒有意義**。
- **建議（D2）**：
  - `x500_through` airframe 設 `UXRCE_DDS_SYNCT = 0`（offset 固定為 0）。
  - 所有 ROS 2 節點 `use_sim_time = true`，並以 `ros_gz_bridge` 橋接 `/clock`。
  - `timestamp_sample = header.stamp`（sim time，µs），`timestamp = node.get_clock().now()`（µs）。兩者都直接等於 PX4 時鐘。
  - 副作用：`/fmu/out/*` 的時間戳也不再加 offset，直接是 sim time，與 ROS sim time 一致，反而一致。
- **QoS**（`utilities.hpp:135`）：PX4 端 DataReader 是 `BEST_EFFORT`、`VOLATILE`、`KEEP_LAST`，depth = 2 × ORB queue。ROS 端 publisher 建議 `BEST_EFFORT / VOLATILE / KEEP_LAST depth 1`（`qos_profile_sensor_data`）；RELIABLE writer 也相容，但沒必要。
- lockstep：實測 RTF ≈ 1.0007。視覺處理延遲由 `timestamp_sample` 補償。
- **未實測**：端到端時間戳在 M1 用 `listener vision_target` 驗證（ROS 2 與 MicroXRCEAgent 已於 2026-10-27 安裝完成）。

---

## V8 新增 uORB msg 與 DDS

- `docs/en/middleware/uorb.md:19-22`：需要跨版本相容的才放 `msg/versioned/`（必須有 `MESSAGE_VERSION` 並配合 translation node）。本專案放 **`msg/VisionTarget.msg`、`msg/ThroughStatus.msg`（非版本化）**，並加入 `msg/CMakeLists.txt` 的 `msg_files`。topic 名稱由檔名推導：`vision_target`、`through_status`。
- `dds_topics.yaml` 的 `subscriptions:` 加上：
  ```yaml
    - topic: /fmu/in/vision_target
      type: px4_msgs::msg::VisionTarget
  ```
  非版本化 msg 的 topic 不會加 `_vN`（`utilities.hpp:35`，只有 `MESSAGE_VERSION > 0` 才加），ROS 端就是 `/fmu/in/vision_target`。
- 規格 5.1 的 msg 沒有宣告 `ORB_QUEUE_LENGTH`（預設 1）→ reader depth 2，沒問題。
- **px4_msgs**：`Tools/copy_to_ros_ws.sh <ros_ws>`。若 ws 內沒有 px4_msgs 會先 clone 並清空，再複製 `msg/*.msg`、`msg/versioned/*.msg`、`srv/*.srv`。注意這個腳本也會把 `translation_node`、`px4_msgs_old` 複製進 ws/src；不需要的話改成手動 `cp`（`docs/en/middleware/uxrce_dds.md:388`）。
- logger：`src/modules/logger/logged_topics.cpp` 的 `add_default_topics()` 加 `add_topic("vision_target")`、`add_topic("through_status")`。

---

## V9 Build 與自動啟動

- **Kconfig**：`menuconfig MODULES_THROUGH_CONTROL`，`bool`，`default n`（照 `mc_nn_control` 的寫法）。
- **CMake**：`px4_add_module(MODULE modules__through_control MAIN through_control SRCS ... MODULE_CONFIG module.yaml DEPENDS PositionControl mathlib px4_work_queue)`。`module.yaml` 是現行支援的寫法（例如 `battery_status`、`commander`）→ **採用 `module.yaml`**。
- **board**：`boards/px4/sitl/default.px4board` 加 `CONFIG_MODULES_THROUGH_CONTROL=y`。
- **啟動**：`ROMFS/px4fmu_common/init.d/rc.mc_apps`（SITL 也用這份；由 `rcS:289` 的 `rc.vehicle_setup` 載入），比照 `MC_NN_EN`：
  ```sh
  if param compare -s THRU_EN 1
  then
  	through_control start
  fi
  ```
  啟動順序：`rc.mc_apps`（289）→ `uxrce_dds_client`（324）→ mavlink（360）。註冊在 mavlink 之前完成沒有問題（V3）。
- **airframe**：`ROMFS/px4fmu_common/init.d-posix/airframes/<ID>_gz_x500_through`，並登記到同目錄的 `CMakeLists.txt`。`PX4_SIM_MODEL=gz_x500_through` 會依檔名找到 autostart（`rcS:56`）。
  - **ID 選 4090**：posix 與 NuttX 的 airframes 都沒有用到（4050、4052 等已被 NuttX 機型佔用）。→ D3
- **模型 / 世界**：`Tools/simulation/gz/models/x500_through/`、`Tools/simulation/gz/worlds/through_ring.sdf`，用 `PX4_GZ_WORLD=through_ring` 選擇。`gz_env.sh` 把 `PX4_GZ_MODELS`、`PX4_GZ_WORLDS` 指向原始碼樹，修改模型不需要重新編譯。世界的 system plugin（Sensors、Imu 等）來自 `server.config`，世界 SDF 裡不必另外宣告。

---

## V10 Hover thrust

- `msg/HoverThrustEstimate.msg`：`timestamp`、`timestamp_sample`、`hover_thrust`（[0.1, 0.9]）、`hover_thrust_var`、innovation 相關欄位、**`valid`**。
- `MulticopterHoverThrustEstimator.cpp`：只要 armed 且 in_air（`dist_bottom > 1 m`），**任何模式下都會更新**，使用已分配推力的垂直分量。`valid` 的條件是 `var < 0.001`（加上 test ratio），有 2 s 遲滯。|vz|、|vxy| 越大，量測雜訊放得越大，所以俯衝中更新變慢，但仍保持有效。
- 備援：`MPC_THR_HOVER` 預設 0.5，**x500 airframe 設 0.60**（`4001_gz_x500:50`）；`MPC_USE_HTE = 1`。
- **推力餘裕**（T_h = 0.60）：`f0 = 0.70/0.60·g = 1.17 g`、`f_max = 1.50 g`、`f_min = 0.50 g`。Step 5a 中垂直 LOS 的需求（含重力分量）上限是 1.5 g，俯衝時 `a_perp` 的可用空間有限。調參時要留意，但不改設計。
- **建議（D4）**：在 ALIGN 進入時鎖存 T_h（valid 用 estimate，否則用 `MPC_THR_HOVER`），整趟使用同一個值，避免俯衝中估計值漂移造成推力基準浮動。這仍符合 Step 5「T_h = hover thrust」的定義。

---

## 9. 規格版本表補完（第 2 節）

| 項目 | 查證結果 |
|---|---|
| gz sim | Harmonic（gz-sim 8.x）；PX4 文件指定 Ubuntu 22.04 用 Harmonic（`docs/en/sim_gazebo_gz/index.md:21`）。本機 8.15.0 ✔ |
| ROS 2 | Humble（`docs/en/ros2/user_guide.md:53`）。已安裝 `ros-humble-desktop` 0.10.0 ✔ |
| ros_gz_bridge | apt 的 `ros-humble-ros-gz` 是對應 Fortress 的。接 Harmonic 要用 OSRF 套件庫的 `ros-humble-ros-gzharmonic`，或從原始碼編譯（`GZ_VERSION=harmonic`）。已安裝 `ros-humble-ros-gzharmonic` 0.244.12（`ros_gz_bridge`、`ros_gz_image`）✔ |
| MicroXRCEAgent | v2.4.3（PX4 client 為 v2.x，不可用 v3.x），已安裝於 `/usr/local/bin/MicroXRCEAgent` ✔ |
| OpenCV / cv_bridge | `python3-opencv` 4.5.4、`ros-humble-cv-bridge` 3.2.1 ✔。注意：apt 的 cv2 是用 NumPy 1.x 編譯的，`~/.local` 不可放 NumPy 2.x（已降為 1.26.4） |
| QGC | 本機有 2024-06 的 AppImage；文件建議 Daily（動態模式列表） |
| pymavlink / pyulog | 已安裝（2.4.50 / 1.2.4） |

## 10. 其他發現（規格內部一致性）

1. 第 8.2 節的 `uncage == false → USER_UNLOCK` 只列了 RAMP/GUIDE，ALIGN 被解鎖時會持續懸停，直到逾時或跟丟。→ D5
2. `MIN_ALT` 只列了 RAMP/GUIDE，ALIGN 沒有。→ D5
3. `ThroughStatus.los_elevation` 是「正值 = 地平線以上」，gimbal 內部與 `el_des` 是「正值 = 往下」。兩者都有明確定義，實作時照規格，但 log 分析工具要標示清楚。
4. 規格 13.1 要求的相機（640×480、HFOV 90°、60 Hz）與原始模型（1280×720、114.6°、30 Hz）不同，在 `x500_through` 自訂模型中修改即可。

## 11. 決策（2026-10-27 確認：D1–D6 全部採用建議方案）

| 代號 | 問題 | 決定 |
|---|---|---|
| **D1** | gimbal 回授只有 5 Hz、沒有取樣時間戳（V5） | **A（建議）**：修改 `GZGimbal`（只動模擬橋接，不動 commander），讓指令在 `gimbal_device_set_attitude` 更新時就送出，並在每次 `camera_imu` callback（或 ≥ 100 Hz）發布 `gimbal_device_attitude_status`，`timestamp` 用 IMU 的 sim time。最貼近 Step 1 的原意。<br>B：改用 `R_nb · R_bc(cmd)` 備援，但 gimbal 移動時誤差可能達數度。<br>C：把 `camera_imu` 橋接到 ROS，由視覺節點附帶相機四元數；這會改動 `VisionTarget` 介面。 |
| **D2** | DDS 時間同步（V7） | airframe 設 `UXRCE_DDS_SYNCT=0`，ROS 節點全部 `use_sim_time` |
| **D3** | airframe ID | `4090_gz_x500_through` |
| **D4** | T_h 取值時機（V10） | ALIGN 進入時鎖存 |
| **D5** | ALIGN 階段的解鎖 / 低高度（第 10 節 1、2） | 建議 ALIGN 也套用 `USER_UNLOCK` 與 `MIN_ALT` 退出 |
| **D6** | SITL 沒有 RC（V3） | M2 的「RC 切入」改用 QGC 搖桿按鈕動作驗證；真正的 `COM_FLTMODE` 綁定需要實體 RC 或搖桿模擬 RC，列為選測 |

## 12. M2 驗收結果（2026-10-27）

### 12.1 自動驗收（`tools/through/m2_acceptance.py`）

環境：headless `px4 -d`（`PX4_SYS_AUTOSTART=4090`）、`MicroXRCEAgent udp4 -p 8888`、`ros_gz_bridge` `/clock`、`fake_vision_target`（`use_sim_time`）。腳本走 MAVLink 14540，用和 QGC 相同的指令（`AVAILABLE_MODES`、`DO_SET_MODE`），並以 1 Hz 送出 GCS heartbeat（沒有 heartbeat 時 `NAV_DLL_ACT` 會擋下解鎖）。

| 項目 | 結果 |
|---|---|
| 單元測試 `unit-ThroughPnGuidance` | PASS |
| 註冊 | `mode_id (nav_state) = 23`（EXTERNAL1）、`arming_check_id = 0` |
| M1：`listener vision_target` | 有資料，`timestamp` / `timestamp_sample` 為 sim time（差 20 ms = fake latency）✔ |
| 1 模式清單出現 THROUGH | PASS（`AVAILABLE_MODES` index 19，custom_mode `0x0b040000`） |
| 2 uncage=false 切換被拒 | PASS（commander 最終 ack = `TEMPORARILY_REJECTED`，模式維持 Hold） |
| 2b 拒絕原因 | ULog `event` topic 確認 [實測]：達到 `THRU_MIN_ALT` 後送出 `through_target_not_locked`，切換時送出 `commander_modeswitch_not_avail`，QGC 會把最後一筆 health report 附加在後面。MAVLink 端無法自動驗證（本機 pymavlink 2.4.50 沒有 `EVENT` 定義）；**QGC 實際顯示需人工確認** |
| 3 uncage=true 切入 | PASS（`THROUGH: IDLE -> ALIGN`） |
| 4 懸停 30 s 水平漂移 < 1 m | PASS（最大 0.08–0.12 m；高度下沉約 2.9 m，見 12.3） |
| 5 從外部切走立即停止 | PASS（`THROUGH stopped: USER_MODE_SWITCH`，約 1.3–1.5 s 內 nav_state 改變，含 heartbeat 週期） |
| RC 切入（D6） | 未測：需要 QGC 搖桿按鈕，人工確認 |

### 12.2 實作時的發現

1. **`DO_SET_MODE` 有兩個 ack**：`mavlink_receiver.cpp:553-554, 616` 轉送指令時會先回 `ACCEPTED`，接著 commander 再回真正結果（被拒時是 `TEMPORARILY_REJECTED`，`Commander.cpp:915-931`）。這是 PX4 原本的行為，判斷結果要看最後一個 ack。
2. **拒絕原因不是在切換當下送出**：health / arming report 只在結果改變時送出，且至少間隔 2 s（`HealthAndArmingChecks/Common.cpp:230-267`）。commander 的 `"Switching to External 1 is currently not available"` 文字訊息使用 nav_state 名稱（不是 `THROUGH`），可讀的原因要靠 QGC 的 events 顯示。
3. `px4` 用 `make px4_sitl` 在背景執行、stdin 為 `/dev/null` 時，pxh 會無限輸出提示字元（log 在 2 分鐘內長到 700 MB）。無人值守時改用 `bin/px4 -d`。

### 12.3 Hover thrust 更正（影響 M5，D7 = A，2026-10-27 確認）

- `x500_through` 目前與 `x500_gimbal` 相同（gimbal 4 個 link 各 0.1 kg）。實測 `hover_thrust_estimate` 一直停在 **0.800**，這是 HTE 的上限 `MPC_THR_HOVER + HTE_THR_RANGE = 0.6 + 0.2`（`MulticopterHoverThrustEstimator.cpp:92`）。ALIGN 穩態：thrust 0.812 時 `vz = +0.097 m/s`，速度阻尼的穩態誤差 `g·(T_true/T_h − 1)/K_VDAMP` ≈ 0.1 m/s，對應真實 hover thrust ≈ **0.81**。
- 因此 V10 的 0.60 只適用於純 x500。以 T_h = 0.81 代入規格第 7 節：`f0 = 0.70/0.81·g = 0.86 g`（基準推力低於懸停），`f_max = 1.11 g`，GUIDE 幾乎沒有餘裕。
- ALIGN 的高度下沉是「只做速度阻尼」設計（規格 7、17 節）加上 T_h 鎖存值偏低 1.5% 的結果，符合規格允許的漂移；修正 T_h 之後會變小。

| 代號 | 問題 | 選項 |
|---|---|---|
| **D7** | 懸停推力 0.81，規格推力參數（0.30 / 0.70 / 0.90）是以約 0.6 設計的 | **A（建議）**：M3 的 `x500_through` 自訂模型本來就要替換 gimbal/相機，順便把 gimbal 質量降到合理值或提高馬達常數，讓 hover ≈ 0.6；規格參數不用改。<br>B：保留質量，airframe 4090 設 `MPC_THR_HOVER 0.80`，並重新調整 `THRU_THR_BASE/MIN/MAX`（例如 0.90 / 0.45 / 1.0），最大推力只剩約 1.23 g。<br>C：只在 4090 設 `MPC_THR_HOVER 0.80`，讓 HTE 不被夾住，推力參數維持不變（GUIDE 會一直推力飽和，不建議）。 |

**D7 決定：A**。原因分析與做法：

- PX4 SITL 把正規化推力 T **線性**對應到馬達轉速：`ω = EC_MIN + (EC_MAX − EC_MIN)·T = 150 + 850·T` rad/s（`4001_gz_x500` 的 `SIM_GZ_EC_MIN/MAX`，`THR_MDL_FAC = 0`）。gz 的推力是 `F = k·ω²`（`motorConstant k = 8.54858e-6`）。
  - 純 x500（2.06 kg）：`ω_h = √(m·g/4k) ≈ 769` → T_h ≈ 0.73；加上 gimbal（2.46 kg）：`ω_h ≈ 840` → T_h ≈ **0.81**（與實測一致）。所以 0.60 只是參數預設值，不是任何一種模型的實際值。
  - 只減 gimbal 質量無法達到 0.6（需要總質量 1.52 kg，比機身 2.0 kg 還輕），**因此改用馬達常數**。
- 規格第 7 節假設推力與正規化值成正比（`thrust = |f|·T_h/g`、`f_max = THR_MAX/T_h·g`）。實際的 `F ∝ (0.15 + 0.85·u)²` 不是線性。設 `THR_MDL_FAC = 0.72`，讓 PX4 先做二次補償：`(0.15+0.85u)² = 0.0225 + 0.255u + 0.7225u²`，與 PX4 模型 `0.7225u² + 0.2775u` 只差 `0.0225·(1−u)`（< 2.3% F_max），因此 T 與推力近似成正比。
- `x500_through` 自帶 4 個 `MulticopterMotorModel`，`motorConstant = 1.007e-5`：`F_max = 4k·1000² = 40.3 N`，`T_h = m·g/F_max = 2.46·9.81/40.3 ≈ 0.60`，`T = 0.9` → 1.5 g，與規格第 10 節的推力參數一致。
- airframe 4090：`THR_MDL_FAC 0.72`、`MPC_THR_HOVER 0.60`。
- 結果在 M3 實測（HTE 收斂值、ALIGN 高度漂移）。

## 13. M3 結果（2026-10-27）

### 13.1 產出

| 項目 | 位置 |
|---|---|
| 機型 | `Tools/simulation/gz/models/x500_through/`（x500_base + 自帶馬達，`motorConstant 1.007e-5`）、`models/through_gimbal/`（由 `gimbal` 衍生：640×480、HFOV 90°、60 Hz；pitch PID `p 3.0 / i 0.05 / d 0.06`、`cmd_max ±1.5`；pitch 行程不變，見 V5） |
| airframe | `4090_gz_x500_through`：`THR_MDL_FAC 0.72`、`MPC_THR_HOVER 0.60`（D7） |
| 世界 | `Tools/simulation/gz/worlds/through_ring.sdf`，由 `tools/through/gen_through_ring_world.py` 產生（可調整中心、瞄準點、內徑、管徑、段數）。環為 24 段 cylinder，紅色 emissive，有碰撞；單色天空（`<background>`、無 `<sky>`）、無陰影 |
| 視覺節點 | `ros2_ws/src/through_vision/through_vision/through_vision_node.py`（`ros2 run through_vision through_vision`） |
| 穿環判定 | `tools/through/ring_judge.py`（gz transport `pose/info`，輸出 `crossings.csv`、`los_truth.csv`） |
| 啟動 | `tools/through/run_through_sim.sh [--headless] [--fake-vision] [--no-window] [--no-judge]`，bridge 設定在 `tools/through/config/through_bridge.yaml` |
| 檢查腳本 | `tools/through/m3_check.py`；`m2_acceptance.py --lock-service /through_vision/set_lock` 可接真實視覺節點 |

### 13.2 驗收

| 項目 | 結果 |
|---|---|
| 一鍵啟動 | PASS（headless）：PX4 + gz、agent、bridge、vision、judge 都啟動，THROUGH 註冊，環被偵測。GUI 模式（gz GUI + OpenCV 視窗）**需人工確認** |
| 相機 | 640×480，`fx = fy = 320`（HFOV 90°），影像實測 59.4 fps |
| 除錯視窗正確框出環 | PASS（以 `~/debug_image` 截圖確認：十字、輪廓、外接圓、中心點、pan/tilt/size、鎖定狀態、FPS） |
| pan/tilt 正負號（第 6 節） | PASS：往北（左）3 m → pan 增加；右轉 10° → pan 減少 9.1°；下降 3 m → tilt 減少 5.3°；懸停 20 m 時 tilt = +31.2°（幾何值 31.0°） |
| `set_lock` 服務 | PASS（偵測到目標時接受鎖定，`uncage` 跟著改變；沒偵測到時拒絕） |
| `l` / `u` 鍵 | **需人工確認**（需要顯示器） |
| `ring_judge.py` | 離線合成資料：0.3 m → PASS、0.9 m → HIT、2.0 m → MISS、20 m → 忽略。實飛（定高 5 m 水平穿越）：環心 PASS（r = 0.15 m，與法向量夾 30.9°，等於環的傾角），偏北 2 m → MISS（r = 2.26 m） |
| D7 | HTE 收斂到 **0.594**。M2 回歸（真實視覺節點）5/5 PASS，ALIGN 30 s 高度變化從 2.9 m 降到 **0.22 m** |

### 13.3 實作時的發現與偏離

1. **形態學 open 預設關閉（偏離規格 12.2 step 3）**：0.15 m 管徑在 25–30 m 只有約 2 px 寬，任何 ≥ 2 px 的 open 都會把環整個刪掉（實測 5 px：0 個像素留下）。模擬影像沒有雜訊，所以 `open_kernel_px` 預設 1（不做），`close_kernel_px` 預設 5，小雜點交給 `min_area_px` 過濾。真實雜訊場景可以把 open 打開。
2. 目標無效時 `pan_error` / `tilt_error` 填 **NaN**（規格沒規定）。PX4 端（M4 LosEstimator）必須先看 `target_valid`。
3. **EKF 航向誤差**：地面靜止時 EKF yaw 92.8°，gz 真值 89.05°（NED），差 3.7°。視覺量測本身正確（pan 實測 +0.90°，真值 +1.16°）。M4 的 `los_ned` 會直接吃到這個航向誤差（要求 < 1.5°），驗收時要確認飛行中 EKF 航向是否收斂，否則 LOS 方位角的誤差會由航向主導。
4. PX4 `MAV_CMD_DO_REPOSITION` 的 `param4`（yaw）用**弧度**（`navigator_main.cpp:317`），MAVLink 規格是度。
5. `ring_judge` 只判定「從接近側穿到另一側」的方向，反向穿越記成 `REVERSE`、不計入統計（`--both-directions` 可改）。
6. 曾經發生一次：測試腳本結束後 PX4 onboard MAVLink（14580→14540）停止送出，px4.log 出現 `Error: Interrupted system call`。當時有舊的 MicroXRCEAgent 佔住 8888、vision 節點也被手動重啟過；整個 stack 重開後沒有再出現。`run_through_sim.sh` 已加入檢查，有殘留的 px4 / gz / agent 時拒絕啟動。

## 14. M4 結果（2026-10-27）

### 14.1 EKF 航向誤差的原因：gz 磁偏角正負號相反 [實測]

- 在 PX4 預設地點（lat 47.398、lon 8.546），飛行中 EKF yaw 與 gz 真值固定差約 **+7.0°**，不會收斂（roll/pitch 誤差 < 0.05°）。
- 機體朝向真東時，機體座標的 mag 讀值換算出的磁偏角為 **−3.56°**，PX4 的 WMM 表在同地點是 **+3.41°**，兩者相差 7.0°。傾角、強度都相符，只有偏角的正負號相反（gz 的磁場模型問題；`GZBridge::magnetometerCallback` 的註解也提到 gz 磁力計座標異常，gz-sim PR #2460）。世界座標的誤差無法在 bridge 內修正（需要真實姿態）。
- 影響：LOS 用的相機姿態來自 gz（真值座標），控制則在 EKF 座標。ALIGN 時 yaw 對準後 pan 仍會剩約 7°，永遠進不了 RAMP；`los_ned` 的誤差也會超過 1.5°。
- **試過但無效**：airframe 設 `EKF2_DECL_TYPE 0` + `EKF2_MAG_DECL -3.56`。`EKF2_MAG_DECL` 只有在 `EKF2_DECL_TYPE` 設了 SAVE bit（2）時才會被使用（`mag_control.cpp:248, 615`），設 0 時 EKF 融合的是 0°（剩約 4–5° 誤差）；而設了 SAVE bit，有 GPS 後 PX4 又會把 WMM 值寫回參數（`EKF2.cpp:2751`）。所以只靠參數無法指定偏角，已移除這兩行。
- **採用的處置**：把世界原點移到磁偏角約 0 的地方，這樣正負號相反也沒有影響。PX4 WMM 表在 lat 47.398 時，偏角為 0 的位置在 lon −3.40（梯度 0.334°/°）；實測航向誤差 0.85°（gz 的表比較舊），推算後改用 **lon −4.67**，實測 **0.3–0.6°**。由 `gen_through_ring_world.py --lat/--lon` 產生（預設值即此點）。

### 14.2 實作內容

- **D1**：`GZGimbal` 的 `gimbal_device_set_attitude` 改用 `SubscriptionCallbackWorkItem`，有新設定點就立刻送出（實測 50 Hz，等於 gimbal 模組的頻率）；`gimbal_device_attitude_status` 改在 camera IMU callback 中發布，實測 **125 Hz**（原本 5 Hz），`timestamp` 用 IMU header 的 sim time。
- **`LosEstimator`**（純計算，屬於 `through_guidance` library）：第 7 節 Step 1–3。相機姿態緩衝區在 `timestamp_sample` 前後兩筆樣本之間做**內插**（nlerp），超出緩衝區範圍時取最近的一筆（容許 20 ms 內）。優先用 gimbal 回授，沒有時改用 `R_nb·R_bc(pitch_cmd)`。`u_dot` 用一階 LPF，並移除平行分量；目標無效時暫停，恢復後重新初始化前一筆值。
- **`GimbalTracker`**：`DO_GIMBAL_MANAGER_CONFIGURE` 取得 / 釋放控制權（每秒檢查，被搶走時重新取得）；機體座標 pitch 指令 `cmd = meas + K·(des − meas)`，限制在 −30°～+89°（正值 = 往下），送出時 rate 填 NaN。
- **`ThroughControl`**：ALIGN 時 yaw 以 `THRU_YAW_RMAX` 限速轉向 LOS 方位角、gimbal 追蹤；`|pan|, |tilt|, |yaw 誤差| < THRU_ALN_ERR` 持續 `THRU_ALN_T_OK` 後進入 RAMP。RAMP 在 M4 只維持懸停與追蹤（M5 實作）。ALIGN/RAMP 共用 USER_UNLOCK / TARGET_LOST / MIN_ALT 退出條件；EXIT 與 IDLE 時釋放 gimbal。`through_status` 填入 `los_ned`、`los_rate`、仰角/方位角、`vc`、`gimbal_pitch_cmd/meas`。
- 單元測試 `test/ThroughLosEstimatorTest.cpp`：相機射線正負號、相機姿態 → u 方向、姿態內插、fallback、無效量測保留 u、靜止時 ω ≈ 0、等速直線運動與解析解 `ω = r×ṙ/|r|²` 比較（誤差 < 3%）。`unit-ThroughPnGuidance`、`unit-ThroughLosEstimator` 全部 PASS。

### 14.3 驗收（`tools/through/m4_check.py`）

流程：起飛到 20 m，機頭先偏離環 30°，鎖定後切入 THROUGH；ALIGN 轉回 LOS 方位並由 gimbal 追蹤，進入 RAMP 後停 5 s，再切回 Hold、降落。之後分析 ULog 的 `through_status`，並與 `ring_judge` 的 `los_truth.csv` 比較（兩者都是 sim time，直接對齊）。

| 項目 | 標準 | 結果 |
|---|---|---|
| yaw 對準後進入 RAMP | 進入 | PASS：偏 30° 起算，3.36 s 後進入 RAMP |
| ALIGN 穩態 `|tilt_error|` | < 1° | PASS：平均 0.016°、最大 0.063° |
| `los_ned` 與真實 LOS 夾角 | < 1.5° | PASS：ALIGN+RAMP 全程平均 0.48°、最大 **0.71°**；穩態最大 0.51°（主要是剩下的 EKF 航向誤差） |
| gimbal | — | 指令 31.11°、量測 31.12°、LOS 仰角 −31.09°（幾何 31°） |

- 改成內插之前，全程最大誤差是 3.2°：進入模式後約 60 ms，gimbal 以每秒數百度從 0° 轉到 31°，取最近一筆樣本（模組 100 Hz 讀取）時差了幾 ms，就造成數度誤差。內插後降到 0.71°。轉機頭（最大 53°/s）期間誤差維持在 0.5°，表示影像與姿態的時間戳一致。
- 懸停時的 LOS 角速度雜訊：平均 0.41°/s、p95 0.89°/s。以 N = 3.5、Vc = 10 m/s 估算，PN 的雜訊約 0.5 m/s²（p95），遠小於 `THRU_A_MAX`。

### 14.4 其他發現

1. **本機 Python 環境**：2026-10-27 13:50 有人在 `~/.local` 用 pip 安裝了 `opencv-python 5.0.0.93` + `numpy 2.2.6`。ROS Humble 的 `cv_bridge` 是用 NumPy 1.x 編譯的，載入時出現 `_ARRAY_API not found` 後 segfault，視覺節點收到第一張影像就當掉。`run_through_sim.sh` 改成以 `PYTHONNOUSERSITE=1` 啟動視覺節點（改用系統的 numpy 1.21 / OpenCV 4.5.4），沒有改動使用者的套件。
2. 在 shell 內用 `pkill -f` / `pgrep -f` 找啟動腳本時，如果同一行指令也含有腳本名稱，就會連自己的 shell 一起砍掉（exit 144）。停止和啟動要分成兩個指令執行。

## 15. M5 結果（2026-10-27）

### 15.1 實作內容

- **`PnGuidance::guide()`**（第 7 節 Step 3–5，純計算）：`vc = max(v·u, THRU_VC_MIN)`；`a_perp = N·vc·ω×u`，超過 `THRU_A_MAX` 等比例縮小；5a 在 `|w_perp| > f_max` 時保留重力補償、只縮 `a_perp`（解 `|k·a_perp − g_perp| = f_max` 取 k ∈ [0,1] 的最大值）；5b 固定推力解 / `disc < 0` 時的最小推力解；5c 速度上限 `s_vlim`。
- **RAMP**：`α` 在 `THRU_RAMP_T` 內 0 → 1；`a = (1−α)·a_hover + α·a_guide`，`f0` 的推力基準 `T_h + α·(THRU_THR_BASE − T_h)`；`α = 1` 時進入 GUIDE。
- **GUIDE**：Step 4–6 + yaw/gimbal 追蹤；轉換與退出依第 8.2 節：USER_UNLOCK、TARGET_LOST（最後有效尺寸 > `TERM_SZ·TERM_NEAR` 時改進 TERMINAL）、MIN_ALT、`target_size > THRU_TERM_SZ` → TERMINAL、TILT_SAT（`tilt_limited ∨ thrust_saturated` 連續 > `THRU_SAT_T`）、OPENING（`vc_raw < 0` 連續 0.5 s）；MODE_TIMEOUT 全程適用。
- **TERMINAL**（提前於 M5 實作，因為 GUIDE 的轉換表需要它）：凍結進入當下的 `q_d` 與推力、gimbal 不再下指令，`THRU_TERM_T` 後以 COMPLETED 退出。M6 專注在穿環成功率。
- 同一個週期內發生的階段轉換，會在該週期就執行新階段的 setpoint（EXIT / TERMINAL），setpoint 串流不中斷。
- `through_status` 補上 `a_perp`、`s_along`、`ramp_alpha`、`a_perp_limited`（只在 RAMP/GUIDE 有值，其他階段為 NaN / false）。
- 單元測試新增 7 項（水平 LOS 固定推力解的垂直平衡、沿 LOS 俯衝 `|f| = f0`、`disc < 0` 推力上升、速度上限使 `s` 變小或為負、PN 方向與 `vc_min`、`a_perp` 限幅、5a 垂直推力限制）。`unit-ThroughPnGuidance` 15/15、`unit-ThroughLosEstimator` 7/7 PASS。

### 15.2 實作時的發現：gimbal 釋放後停在最後的指令角度 [實測]

- 第一次驗收時，第一個情境之後所有鎖定都被視覺節點拒絕（目標不在畫面內）。原因：`DO_GIMBAL_MANAGER_CONFIGURE` 釋放控制權後，gimbal manager **保留最後一個 setpoint**（實測 MAVLink pitch +8.4°，鏡頭往上），直到其他模式（Land）重設為止。在起點，環在地平線下 31°、鏡頭上仰約 8° 時已超出垂直半視角 36.9°。
- **處置**：`GimbalTracker::release()` 先送一次 pitch 0（中立）再釋放。`gimbal/input_mavlink.cpp` 在同一次 poll 中先處理 `gimbal_manager_set_attitude` 再處理 `vehicle_command`，所以順序有保證。規格 8.1 只寫「釋放」，回中立是補充行為，不改變設計方向。

### 15.3 驗收（`tools/through/m5_check.py`）

流程：起飛到 20 m（面向環），每個情境先回起點、鎖定、切入 THROUGH，再觸發一種退出；退出原因同時以狀態文字（即時）和 ULog `through_status.exit_reason` 確認，並確認之後模式為 Hold。鎖定 / 解鎖用 rclpy 直接呼叫 `/through_vision/set_lock`（`ros2 service call` 每次約 1 s，在 GUIDE 中太慢）。

| 情境 | 觸發方式 | 階段 | exit_reason | Hold | 退出後最低高度 |
|---|---|---|---|---|---|
| unlock | GUIDE 0.5 s 後 `set_lock(false)` | ALIGN→RAMP→GUIDE→EXIT | USER_UNLOCK ✔ | ✔ | 14.3 m |
| mode_switch | GUIDE 0.5 s 後切到 Hold | ALIGN→RAMP→GUIDE | USER_MODE_SWITCH ✔ | ✔ | 14.2 m |
| min_alt | `THRU_MIN_ALT = 16` | ALIGN→RAMP→GUIDE→EXIT | MIN_ALT ✔ | ✔ | 11.6 m |
| align_to | `THRU_ALN_T_OK = 20`、`THRU_ALN_T_MAX = 3` | ALIGN→EXIT | ALIGN_TIMEOUT ✔ | ✔ | 18.6 m |
| mode_to | `THRU_T_MAX = 4` | ALIGN→RAMP→GUIDE→EXIT | MODE_TIMEOUT ✔ | ✔ | 4.8 m |
| dive（預設參數） | — | ALIGN→RAMP→GUIDE→TERMINAL→EXIT | COMPLETED ✔ | ✔ | **−0.44 m（觸地）** |

- 俯衝接近：ALIGN 約 1 s、RAMP 1 s、GUIDE 3.1 s。`vc` 最大 9.7 m/s（`THRU_V_MAX` 10），推力 0.30–0.70，`|a_perp|` 最大 4.8 m/s²，傾角受限 6% 的時間，推力飽和 0%。目標尺寸最大 0.53（< `TERM_SZ` 0.6），所以 TERMINAL 是由「遺失且已接近」進入的（環在 1.5–2 m 內時會超出畫面）。
- `ring_judge`：dive 情境 **PASS**，r = 0.095 m、速度 9.4 m/s、與環法向量夾 1.1°。（另一筆 112.6 s 的 PASS 是 mode_to 在 5.9 m 退出後，Hold 煞車時以 0.39 m/s 慢速飄過環，不是導引結果；115.6 s 的 REVERSE 是回起點時反向穿過。）

### 15.4 穿環後觸地（D8）

- TERMINAL 進入時高度 5.25 m、`v = (−0.2, 8.1, 5.0)` m/s（向下 5 m/s）。0.5 s 滑行後剩約 2.2–2.75 m，再由 Hold 煞車；以推力上限 0.9（淨向上約 0.5 g）計算，停住 5 m/s 的下降需要 ≥ 2.5 m 加上反應時間，因此在退出後 0.4 s 觸地。根本原因是幾何：環心只有 5 m 高（規格 13.2），俯衝角 31°、`THRU_V_MAX` 10 m/s。
- M6 要求連續 10 次，每次都觸地不可接受。

| 代號 | 問題 | 選項 |
|---|---|---|
| **D8** | 穿環後高度不足，Hold 來不及停住下降 | **A（建議）**：環與起點整體上移，環心改 `(25, 0, 10)`、起點懸停 25 m，俯角與距離不變（31°），穿環後多 5 m 餘裕。只改 `gen_through_ring_world.py` 預設值和測試腳本的 `--alt`，導引參數不動。<br>B：維持世界，降低 `THRU_V_MAX`（例如 6 m/s，下降約 3 m/s）。會改變 PN 表現與規格第 10 節的參數。<br>C：維持世界，縮短 `THRU_TERM_T`（例如 0.2 s）並讓 EXIT 改送「優先垂直煞車」的 setpoint。仍然只有約 3 m 餘裕，邊際。 |

**D8 決定：A（2026-10-27）**。`gen_through_ring_world.py` 預設改為環心 `(25, 0, 10)`、法向量指向 `(0, 0, 25)`（世界已重新產生，環仍傾斜 31°）；`m3_check.py` / `m4_check.py` / `m5_check.py` 的 `--alt` 預設改為 25 m。規格 13.2 的 `(25, 0, 5)` 與 20 m 懸停以此為準。

重跑 `m5_check.py`（25 m 懸停）：6/6 PASS，退出原因與階段同上表。dive 情境 `ring_judge` **PASS**（r = 0.089 m、9.3 m/s、與法向量夾 1.9°），退出後最低高度 **3.81 m**（原本 −0.44 m）。其他情境退出後最低高度：unlock 19.4 m、mode_switch 19.4 m、min_alt（`THRU_MIN_ALT = 21`）16.6 m、align_to 24.6 m、mode_to 9.8 m。

## 16. M6 結果（2026-10-27）

### 16.1 內容

- TERMINAL 已在 M5 實作（15.1 節），M6 沒有改動韌體。
- `tools/through/m6_check.py`：起飛到 25 m（D8）面向環，連續 `--runs` 次（預設 10）從起點鎖定、切入 THROUGH、等待退出；每次之間先**原地爬升**到 25 m 再飛回起點，避免回程穿過環（M5 時回程曾在距環心 1.7 m 處反向穿越，環有碰撞）。之後以 sim time 把 `ring_judge` 的 `crossings.csv` 對到 ULog 中每一次模式執行。
- **只採計模式執行期間（退出後 0.5 s 內）的正向穿越**：M5 的 mode_to 情境中，Hold 在退出後慢速飄過環也被 `ring_judge` 記成 PASS，那不是導引的結果。

### 16.2 驗收：標準場景 10 次

| 次 | ring_judge | r [m] | 速度 [m/s] | 與法向量夾角 | TERMINAL 進入 | 退出後最低高度 |
|---|---|---|---|---|---|---|
| 1 | PASS | 0.102 | 9.3 | 1.3° | 尺寸 0.66 | 3.4 m |
| 2 | PASS | 0.118 | 9.4 | 1.2° | 尺寸 0.64 | 3.3 m |
| 3 | PASS | 0.111 | 9.3 | 1.8° | 尺寸 0.63 | 3.5 m |
| 4 | PASS | 0.123 | 9.4 | 1.9° | 尺寸 0.64 | 3.0 m |
| 5 | PASS | 0.110 | 9.3 | 3.0° | 尺寸 0.78 | 3.4 m |
| 6 | PASS | 0.121 | 9.4 | 1.5° | 尺寸 0.64 | 3.3 m |
| 7 | PASS | 0.112 | 9.4 | 1.8° | 尺寸 0.63 | 3.2 m |
| 8 | PASS | 0.124 | 9.4 | 2.2° | 尺寸 0.64 | 3.0 m |
| 9 | PASS | 0.119 | 9.3 | 1.9° | 尺寸 0.64 | 3.4 m |
| 10 | PASS | 0.117 | 9.3 | 2.2° | 尺寸 0.64 | 3.3 m |

- **10/10 PASS**（標準 ≥ 8）。每次都是 ALIGN → RAMP → GUIDE → TERMINAL → EXIT、`exit_reason = COMPLETED`，退出後進入 Hold；`crossings.csv` 沒有任何 REVERSE 或 HIT（原地爬升有效）。
- 穿越點離環心 0.10–0.12 m（PASS 門檻 0.65 m），偏差很一致，推測是系統性的（相機與機體中心的距離、TERMINAL 滑行時的重力下沉），M7 可以分析。TERMINAL 都是由尺寸門檻進入（目標尺寸 > 0.6，此時距環約 1.7 m）。
- 模擬是無雜訊、無風的確定性場景，10 次結果幾乎相同；這個驗收不代表穩健性。規格第 15 節的其他情境（橫向偏移、30 m 起點、側風、中途遮蔽）留到 M7 調參時一起跑。
- 退出後最低高度 3.0–3.5 m（D8 前為 −0.44 m）。

## 17. M7 結果（2026-10-27）

### 17.1 工具

- `tools/through/analyze_through_log.py LOG.ulg [--run N] [--crossings crossings.csv]`：把 ULog 中每次模式執行切出來，以進入模式的時間為 0、各階段塗色，畫 8 張時間序列（phase / 目標有效、LOS 仰角 / 方位角、LOS 角速度、`a_perp` 與 `s`、推力、傾角設定值 / 實際值、pan / tilt 與目標尺寸、`vc` / 速度 / 高度 / gimbal），並印出調參用的指標（ALIGN 靜止時的 LOS 角速度雜訊、GUIDE 的 `|ω|` p95、`|a_perp|`、各種限制 / 飽和的比例、TERMINAL 進入方式、穿環結果）。
- `tools/through/m7_sweep.py`：一次起飛、每個 case 飛一次俯衝並比較。case = PX4 參數 + 視覺節點參數 + 起點偏移（`std` 標準、`lat` 北偏 5 m（規格 15.2）、`latS` 南偏 5 m、`alt` 高 10 m（規格 15.3））。另外用 `ring_judge` 的真實 LOS 算出 2–5 m 距離的 LOS 偏差。高起點時中立 gimbal 看不到環，腳本會先以 GCS 身分取得 gimbal、壓到幾何俯角再鎖定（`-2` 在 PX4 gimbal 代表自動駕駛自己，要給明確的 sysid/compid）。

### 17.2 掃描結果（`logs/m7_sweep1/`，每格一次俯衝；預設 N 3.5、LPF 8 Hz、TERM_SZ 0.6）

| case | 結果 | r [m] | GUIDE `\|ω\|` p95 [°/s] | `\|a_perp\|` 平均 / 最大 [m/s²] | 推力飽和 |
|---|---|---|---|---|---|
| std（M6 的 10 次） | 10/10 PASS | 0.10–0.12 | 2.0–2.8 | 0.6–0.7 / 2.3–3.2 | 0% |
| std N 2.5 | PASS | 0.101 | 4.4 | 0.68 / 3.8 | 0% |
| std N 5 | PASS | 0.092 | 3.9 | 1.03 / 7.0 | 3% |
| std LPF 4 | PASS | 0.111 | 3.1 | 0.71 / 3.1 | 0% |
| std LPF 15 | PASS | 0.101 | 4.9 | 1.02 / 8.2 | 4% |
| std TERM_SZ 0.45 | PASS | 0.111 | 3.4 | 0.81 / 5.0 | 1% |
| std TERM_SZ 0.75 | PASS | 0.121 | 4.0 | 0.82 / 4.9 | 0% |
| std `center_method=moments` | PASS | 0.141 | 4.0 | 0.84 / 4.6 | 0% |
| lat 基準 | PASS | 0.108 | 7.0 | 1.22 / **9.8** | 0% |
| lat N 2.5 | PASS | 0.111 | 7.0 | 0.93 / 6.4 | 0% |
| lat N 5 | PASS | 0.164 | 6.1 | 1.42 / **9.8** | 3% |
| lat LPF 4 | PASS | **0.063** | 4.5 | 0.87 / **9.8** | 0% |
| lat LPF 15 | PASS | 0.098 | 10.8 | 1.40 / **9.8** | 4% |
| latS 基準 | PASS | 0.057–0.060 | 5.4–8.8 | 1.1–1.2 / **9.8** | 0–1% |
| alt N 2.5 | PASS | 0.079 | 7.6 | 0.88 / 6.3 | 0% |
| alt N 5 | PASS | **0.246** | 6.4 | 1.46 / 9.1 | 5% |
| std 基準、alt 基準（各一次） | **ALIGN_TIMEOUT** | — | — | — | — |

進入 GUIDE 的俯衝全部 PASS（27/27：M6 10 次、掃描 13 次、重跑 4 次）。兩次 ALIGN_TIMEOUT 見 17.4。

**`|a_perp|` 大值出現的位置**：
1. **GUIDE 開始的 1 s 內**（RAMP 末段機體俯仰到約 60°、gimbal 快速擺動時，LOS 角速度有雜訊）。N 5、LPF 15 時明顯（最大 7–8 m/s²），**LPF 4 Hz 時完全消失**。
2. **只有側向起點**：TERMINAL 前最後 0.3 s（目標尺寸 0.3–0.5，距環約 2–3 m），每次都打到 `THRU_A_MAX`。斜看環時，LOS 量測偏差隨距離快速變化，PN 把它當成 LOS 轉動。

### 17.3 調參建議

| 參數 | 目前 | 建議 | 依據 |
|---|---|---|---|
| `THRU_LOS_LPF` | 8 Hz | **4–5 Hz** | GUIDE 開始的雜訊尖峰消失；側向 `\|ω\|` p95 從 7.0 降到 4.5 °/s；側向偏差最小（0.063 m）。15 Hz 雜訊加倍並出現推力飽和。 |
| `THRU_PN_N` | 3.5 | **維持 3.5（可降到 3）** | N 5 在每種起點都讓 `a_perp` 與飽和變大、偏差變大（alt 0.246 m 最差）；N 2.5 也都 PASS 且 `a_perp` 較小。目標靜止、起點幾乎正對環，不需要高 N。 |
| `THRU_TERM_SZ` | 0.6 | **0.4** | LOS 誤差隨尺寸增加（尺寸 0.3–0.4：約 3°；0.6–0.7：約 6°；> 0.7：> 20°），側向起點最後 2–3 m 的 `a_perp` 都是假訊號。0.4 約在環前 2.6 m 凍結（約 0.28 s 滑行），標準起點的 0.45 / 0.75 都不影響結果。 |

以上是單次俯衝的比較（模擬是確定性的）。**決定（2026-10-27）：建議只記錄在本文件，`module.yaml` 預設值維持規格第 10 節。** 要採用時，先以 `m7_sweep.py` 用新值跑標準 + 側向 ± 5 m + 高起點確認。

### 17.4 發現：EKF 航向誤差使 ALIGN 無法完成（D9）

- 兩次 ALIGN_TIMEOUT 都是 pan 卡在約 3–6° 以上，tilt 正常。gimbal yaw 沒問題（實測相機與機體航向差 0.2–0.3°）。
- 原因是**座標框不一致**：LOS 用的相機姿態是 gz 真值（D1），機體 yaw 是 EKF 估測。ALIGN 把 EKF yaw 轉到 LOS 方位角，EKF 航向誤差就以 pan 的形式留著；超過 `THRU_ALN_ERR`（3°）就永遠無法對準。
- 實測（`vehicle_attitude` 與 `vehicle_attitude_groundtruth`）：起飛時 EKF 航向誤差約 5.7°，第一次飛行約 60 s 後降到 1° 以下；但另一次飛行在起飛後 93 s（中間爬升了 10 m）仍有 5.9°。M4 的 0.3–0.6° 是穩態值。所以「起飛後等 60 s」（`m7_sweep.py --settle`）不可靠。
- 真機上相機姿態會跟機體姿態同一個座標框，不會出現這種不一致；這是模擬用真值相機姿態造成的。

| 代號 | 問題 | 選項 |
|---|---|---|
| **D9** | 相機姿態（gz 真值）與機體姿態（EKF）座標框不一致，EKF 航向誤差 > 3° 時 ALIGN 必定逾時 | **A（建議）**：`GZGimbal` 發布的相機姿態改為「EKF 機體姿態 × 真實的機體→相機相對姿態（關節角）」，與真機的 gimbal 一致。ALIGN 不再受 EKF 航向誤差影響；LOS 會帶有 EKF 航向誤差（真機也是），M4 的 LOS 精度要重新量測。只改模擬橋接。<br>B：保留模擬，ALIGN 的對準條件只看 tilt 與 yaw 誤差、不看 pan。偏離規格 8.2。<br>C：維持現狀，只在測試流程中等 EKF 收斂。不可靠（見上）。 |

**D9 決定：C（2026-10-27）**。模擬與 ALIGN 都不改；測試流程在起飛後先懸停讓 EKF 航向收斂（`m7_sweep.py --settle`，預設 60 s），並接受偶發的 ALIGN_TIMEOUT（屬於正確的退出路徑，不影響安全）。M6/M7 的統計只計入有進入 GUIDE 的俯衝，ALIGN_TIMEOUT 另外記錄。

### 17.5 其他觀察

- **LOS 偏差約 0.15 m 加 0.2°**（估測的 LOS 比真實的往上）：距離 21 m 時 0.6°，1.75 m 時 5.0°，符合「固定 0.145 m 偏移 + 0.2° 角度偏差」。`moments` 與 `circle` 結果相同，所以不是偵測方法；環模型的中心正確。M6 穿越點穩定偏 0.11 m 應該就是這個偏差（相機瞄準點偏上、機體原點在相機下方約 0.1 m）。來源未查明，候選為相機光學中心與 `camera_link` 的差、或 gz 相機內參。目前在 PASS 門檻（0.65 m）內。
- 側向起點的方位偏差（約 ±2°，正負號跟著偏移方向）是斜看環的透視效應，屬預期。
- TERMINAL 中 LOS 估測會跳到數百 °/s（環離開畫面時的殘缺影像），因為 TERMINAL 不使用視覺，不影響飛行，但分析時要排除。
- 規格第 15 節的側風（4）與中途遮蔽目標（6）尚未測：側風需要在世界加 wind 外掛並讓模型 `enable_wind`；遮蔽需要在視覺節點加一個模擬遮蔽的參數。

## 附錄：runtime 探測腳本

位於 session scratchpad（不屬於專案）：`probe_gimbal.py`（MAVLink gimbal v2 正負號與狀態頻率）、`step2.py`（以 gz `camera_imu` 量 pitch 步階響應）。若要保留，可移到 `tools/through/`。
