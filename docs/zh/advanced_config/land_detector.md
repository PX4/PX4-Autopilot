# 着陆探测器配置

着陆探测器是一个动态飞行器模型，代表了从接触地面到着陆的关键飞行器状态。本节主题说明您可能希望调整的主要参数，以改善飞行器着陆行为。

## 自动上锁

当飞行器着陆时着陆探测器会为飞行器自动加锁。

您可以设置 [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND) 来指定着陆后系统上锁前的秒数（或通过将参数设置为 -1 关闭自动上锁）。

## 多旋翼配置

相关着陆探测器参数的完整组列于参数引用前缀 [LNDMC](../advanced_config/parameter_reference.md#land-detector) (这些参数可以通过 [参数编辑器](../advanced_config/parameters.md)在QGroundControl中进行编辑。

:::tip
有关这些参数如何影响着陆的信息，请参见下文的[着陆检测器状态](#mc-land-detector-states)。
:::

为了改善特定机架上的着陆，您可能需要调整的其他关键参数包括:

- [MPC_THR_HOVER](../advanced_config/parameter_reference.md#MPC_THR_HOVER) - 系统的悬停油门(默认值为50%)。正确设置这一点是很重要的, 因为它使高度控制更加准确, 并确保正确的地面检测。没有安装有效载荷的穿越机或大型相机的无人机可能需要更低的悬停油门 (例如 35%)。

  ::: 信息
  如果错误地设置了 `MPC_THR_HOVER`，可能会导致飞行器仍在空中时（特别是当在 [位置模式](../flight_modes_mc/position.md) 或 [高度模式](../flight_modes_mc/altitude.md) 下下降时）触发触地或疑似着陆检测。这将导致载具 "抽搐" （关闭电机，然后立即将其重新打开）。

:::

- [MPC_THR_MIN](../advanced_config/parameter_reference.md#MPC_THR_MIN) - 系统的总最小油门。应将此设置为“启用”以实现受控下降。

- [MPC_LAND_CRWL](../advanced_config/parameter_reference.md#MPC_LAND_CRWL) —— 若系统配备距离传感器且该传感器存在并正常工作，则在自主着陆的最后阶段应用的垂直速度。必须设置大于 LNDMC_Z_VEL_MAX。

### 着陆探测器状态

为了探测着陆，多旋翼首先必须经历三个不同的状态，其中每个状态都包含来自先前状态的条件以及更严格的约束。如果由于缺少传感器而无法达到条件，则默认情况下认为该条件为真。例如，在 [特技模式](../flight_modes_mc/acro.md) 下，如果除了陀螺仪传感器外没有其他传感器处于活动状态，那么检测将完全依赖于推力输出和时间。

为了进入下一个状态，每个条件必须在某个预定义的时间内为真 300ms。如果机体配备了距离传感器，但当前无法测量与地面的距离（通常是因为距离过大），则触发时间将增加3倍。

如果一个条件失败，则陆地探测器立即退出当前状态。

#### 地面接触

此状态的条件：

- 无垂直移动([LNDMC_Z_VEL_MAX](../advanced_config/parameter_reference.md#LNDMC_Z_VEL_MAX))
- 无水平移动([LNDMC_XY_VEL_MAX](../advanced_config/parameter_reference.md#LNDMC_XY_VEL_MAX))
- 推力低于 [MPC_THR_MIN](../advanced_config/parameter_reference.md#MPC_THR_MIN) + (悬停油门值 - [MPC_THR_MIN](../advanced_config/parameter_reference.md#MPC_THR_MIN)) × (0.3，除非有悬停推力估计值，否则为 0.6)，
- 额外检查：机体当前是否处于高度速率控制飞行模式：飞行器必须具有下降意图（垂直速度设定值高于 LNDMC_Z_VEL_MAX）。
- 额外检查配有距离传感器的机体：目前距离地面的距离不到1米。

如果机体处于位置控制或速度控制模式，且检测到与地面的接触，
位置控制器将把沿机身x-y轴方向的推力矢量设为零。

#### 可能着陆

此状态的条件：

- [着陆](#ground-contact) 状态的所有条件均为真
- 没有在旋转([LNDMC_ROT_MAX](../advanced_config/parameter_reference.md#LNDMC_ROT_MAX))
- 具有低推力 MPC_THR_MIN + (MPC_THR_HOVER - MPC_THR_MIN) \* 0.1\`
- 未检测到自由落体事件

如果飞行器只知道推力和角速度，为了进入下一个状态，机体必须具有较低的油门和非旋转状态达到 8.0 秒。

如果机体处于位置或速度控制模式，且检测到可能已着陆，
位置控制器将把推力矢量设为零。

#### 降落完成

此状态的条件：

- [着陆](#maybe-landed) 状态的所有条件均为真

## 固定翼配置

调节固定翼着陆探测参数：

- [LNDFW_AIRSPD_MAX](../advanced_config/parameter_reference.md#LNDFW_AIRSPD_MAX) - 允许系统仍被视为着陆的最大空速。必须在空速检测精度与触发速度之间权衡取舍。越好的空速传感器允许此参数的值越低。
- [LNDFW_VEL_XY_MAX ](../advanced_config/parameter_reference.md#LNDFW_VEL_XY_MAX) —— 系统仍被视为已着陆时的最大水平速度。
- [LNDFW_VEL_Z_MAX](../advanced_config/parameter_reference.md#LNDFW_VEL_XY_MAX) —— 系统仍被视为已着陆时的最大垂直速度。
- [LNDFW_XYACC_MAX](../advanced_config/parameter_reference.md#LNDFW_XYACC_MAX) ——系统仍被视为已着陆时的最大水平加速度。
- [LNDFW_TRIG_TIME](../advanced_config/parameter_reference.md#LNDFW_TRIG_TIME) - 触发时间，在此期间必须满足上述条件，才能判定为着陆。

:::info
当启用FW起飞检测功能时（[FW_LAUN_DETCN_ON](../advanced_config/parameter_reference.md#FW_LAUN_DETCN_ON)），机体将保持在“着陆”状态，直到检测到起飞（该检测仅基于加速度，而非速度）。
:::

## VTOL 着陆探测器

如果系统处于悬停模式，VTOL着陆探测器为1:1，与MC着陆探测器相同。在FW模式下，着陆探测功能被禁用。
