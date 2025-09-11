# 飞行数据反演气动力（可落地流程）

本文给出一套可执行的“灰盒（物理+神经网络）”建模与飞行数据反演气动力流程，适用于扑翼/多旋翼/固定翼等平台，用于得到机体系气动力/力矩并用于后续建模与控制前馈/补偿。

---

## 一、所需数据（建议话题与频率）
- 姿态/角速度/线速度/位置
  - `vehicle_attitude`（四元数 Rwb，≥200 Hz）
  - `vehicle_angular_velocity`（ωb，≥200 Hz）
  - `vehicle_local_position`（vw, pw，≥50 Hz；若有 aw 更好）
  - `vehicle_gps_position`（RTK fix/float、速度；用于真值/对齐）
- IMU/气压/空速（可选但强烈推荐）
  - `vehicle_imu`（加速度计 fb、陀螺，≥400–1000 Hz）
  - `vehicle_air_data`（p、T→ρ）；`airspeed`（若有）
- 执行器与电功（建模推进/舵面）
  - `actuator_outputs` / `actuator_motors` / `actuator_servos`（≥200 Hz）
  - `battery_status`（V、I，用于电机推力模型修正）
- 质量与惯量（离线标定）
  - 总质量 m、转动惯量 J（机体坐标系）；重心位置

---

## 二、预处理（对齐/降噪/机体系）
- 时间同步与重采样：将所有话题插值到 IMU 时间轴（噪声最低），统一频率（如 400–500 Hz）。
- 去噪：对加速度与角速度用低通/SG 滤波（例如 30–60 Hz），对导数信号用二次以上平滑。
- 参考系约定：
  - Rwb：世界→机体旋转（由 `vehicle_attitude` 获取）；Rbw = Rwb^T
  - ρ 由气压/温度估算；g = 9.80665 m/s^2
- 质量/惯量：采用地面标定值；随载重变化需更新。

---

## 三、动力学方程（推荐：世界系平动，机体系转动）
- 平动（世界系）：
  - m·v̇w = Fw_tot + m·g·ez
  - 实践：Fw_tot = m·v̇w − m·g·ez，再转到机体系 Fb_tot = Rbw·Fw_tot
- 平动（机体系，比力法）：
  - 加速度计比力 fb = ab − Rbw·g·ez + ba + n
  - 若已校正偏差与滤波，则 Fb_tot ≈ m·fb
- 转动（机体系）：
  - τb_tot = J·ω̇b + ωb × (J·ωb)
- 分解：
  - Fb_aero = Fb_tot − Fb_prop − Fb_other
  - τb_aero = τb_tot − τb_prop − τb_other

---

## 四、反演步骤（两条路线）
- 路线A（IMU比力法，推荐）：
  1) 计算 fb 并低通 → Fb_tot = m·fb
  2) 计算 ωb、ω̇b 并低通 → τb_tot = J·ω̇b + ωb × (J·ωb)
  3) 建模/估算 Fb_prop、τb_prop（见第五节）
  4) 得 Fb_aero、τb_aero
- 路线B（位置二次微分法）：
  1) 用 `vehicle_local_position` 的 vw 平滑微分得 v̇w
  2) 得 Fw_tot 并转到机体系 → Fb_tot
  3) 后续同上 2)–4)
- 建议：A 为主、B 交叉校核。

---

## 五、推进力/力矩建模（Fb_prop、τb_prop）
- 多旋翼/电机桨：
  - 推力：T = kT·ω² 或 T = a·u² + b·u + c（u为归一化油门/DSHOT；可加电压补偿）
  - 推力方向：电机轴向（通常机体 z 轴），方向对号入座（上推为 −z）
  - 反扭矩 Q = kQ·ω²，臂长产生 τ = r × F
- 固定翼舵面：通常并入气动项（不单独当 prop 扣除）。
- 扑翼（拍振）：
  - 若有相位/频率/扭矩测量：可做“平均推力+周期力矩”简模；否则仅计平均推力，周期项由气动残差承接。

---

## 六、风与空速
- 有空速管：使用 Va 作为特征；用于后续灰盒/NN。
- 无空速管：使用 EKF2 风估计；或将风当未建模扰动（反演得到的是“净气动”）。

---

## 七、数值与信号技巧
- 低通：加速度/角速度 30–60 Hz；ω̇ 用 Savitzky–Golay（窗口与阶次匹配采样）。
- 延迟：执行器→机体响应 10–50 ms，建议互相关估计并校正。
- 坐标/重心：IMU 与机体系一致；IMU 离重心较远时考虑平动/转动耦合修正。
- 采样：扑翼高频，建议 ≥500 Hz 采样，避免把周期效应滤没。

---

## 八、验证与质量控制
- 动力学一致性：用 Fb_aero、τb_aero 前推预测 ab/ω̇b 与测量对比（MSE/相关系数）。
- 方向性检查：⟨vb, Fb_aero⟩ ≤ 0 多数情况下成立（阻力非负），异常段定位问题或模型错误。
- 周期特性：扑翼工况对单拍做平均/谐波分解，检查能量/力矩平衡。

---

## 九、简明伪代码（IMU比力法）
```python
# 输入：f_b(t), omega_b(t), Rwb(t), actuator, m, J, g
# 预处理：滤波、对齐到 IMU 频率

F_tot_b = m * f_b                      # 比力法得到总力（机体系）
omega_dot_b = smooth_diff(omega_b)     # SG/卡尔曼微分器

# 刚体转动总力矩（机体系）
tau_tot_b = J @ omega_dot_b + cross(omega_b, J @ omega_b)

# 推进贡献（电机/拍振）
F_prop_b, tau_prop_b = propulsion_model(actuator, V_batt=None, rho=None)

# 气动反演结果
F_aero_b = F_tot_b - F_prop_b
tau_aero_b = tau_tot_b - tau_prop_b
```

---

## 十、常见坑与对策
- 加速度计零偏/量程：做静态/在线校准；bias 进入 fb 扣除。
- 位置二次微分噪声（路线B）：优先用 RTK 速度或平滑样条。
- 质量/惯量误差：对反演线性影响；优先地面三摆/摆动估计，必要时最小二乘修正。
- 推力曲线不准：先做台架标定；或把平均推进力当未知，由网络残差吸收。
- 高风与阵风：分数据集评估；在模型中显式加入风/空速特征。

---

> 提示：若你提供包含上述话题的 ULog，我可以进一步给出 Python 脚本样板（ULog→DataFrame→反演→验证图表），并按你的扑翼频率/传感条件调整滤波与延迟补偿细节。

