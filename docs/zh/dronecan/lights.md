# DroneCAN 灯光

PX4 可以使用标准 DroneCAN [LightsCommand](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#lightscommand) 消息，控制已连接的 DroneCAN 外设上的外部 LED。

最多支持 2 个灯。这些灯可以显示[系统状态颜色](../getting_started/led_meanings.md#ui-led)、固定颜色（用于指示飞行器朝向），或根据解锁状态在两者之间切换。

## 支持的 RTK 设备

任何实现标准 `LightsCommand` 消息类型的 DroneCAN 外设都应能正常工作。

以下设备已经过测试：

- **Vertiq 电调 LED 扩展模块**：每个电调提供两个灯光 ID，分别对应一个 RGB 灯（用于显示状态）和一个白灯。每个灯的 `light_id` 按 `esc_index × 3 + BASE_ID` 计算，其中 RGB 灯的 `BASE_ID` 为 1，白灯为 2。其他电调设置详情请参阅 [Vertiq](../peripherals/vertiq.md)。

## PX4 配置

1. 按 [DroneCAN](index.md) 中的说明设置 DroneCAN（`UAVCAN_ENABLE` ≥ 2）。
2. 将 [UAVCAN_LGT_NUM](../advanced_config/parameter_reference.md#UAVCAN_LGT_NUM) 设为灯的数量（1 或 2）。然后重启并重新打开地面站，使新增实例的参数可见。
3. 设置每个灯的 `light_id` 和[灯光功能](#light_functions)：
   - [UAVCAN_LGT_ID0](../advanced_config/parameter_reference.md#UAVCAN_LGT_ID0) / [UAVCAN_LGT_ID1](../advanced_config/parameter_reference.md#UAVCAN_LGT_ID1)：设为 `light_id` 的值（由具体产品定义）。
   - [UAVCAN_LGT_FN0](../advanced_config/parameter_reference.md#UAVCAN_LGT_FN0) / [UAVCAN_LGT_FN1](../advanced_config/parameter_reference.md#UAVCAN_LGT_FN1)：选择所需的[灯光功能](#light_functions)。
4. 设置 [UAVCAN_LGT_MODE](#UAVCAN_LGT_MODE)，以控制固定的“方向指示”颜色何时启用。
5. 重启使更改生效。

### 灯光功能 {#light_functions}

已启用的灯分别通过 [UAVCAN_LGT_FN0](../advanced_config/parameter_reference.md#UAVCAN_LGT_FN0) 和 [UAVCAN_LGT_FN1](../advanced_config/parameter_reference.md#UAVCAN_LGT_FN1) 配置功能。每种功能由一个数值表示，该数值定义两种行为：激活模式**未激活**时的行为，以及**已激活**时的行为。

| 值 | 参数名   | 模式未激活时 | 模式已激活时 |
| - | ----- | ------ | ------ |
| 0 | 状态/状态 | 系统状态颜色 | 系统状态颜色 |
| 1 | 熄灭/白色 | 熄灭     | White  |
| 2 | 熄灭/红色 | 熄灭     | Red    |
| 3 | 熄灭/绿色 | 熄灭     | 绿色     |
| 4 | 状态/白色 | 系统状态颜色 | White  |
| 5 | 状态/红色 | 系统状态颜色 | Red    |
| 6 | 状态/绿色 | 系统状态颜色 | 绿色     |
| 7 | 状态/熄灭 | 系统状态颜色 | 熄灭     |

Notes:

- [系统状态颜色](../getting_started/led_meanings.md#ui-led)与飞行控制器板载状态 LED 使用相同的指示模式（例如，未解锁时为红色，已解锁且就绪时为绿色）。
- 固定颜色通常用于指示飞行器朝向。例如，常见的惯例是在左侧使用红灯、右侧使用绿灯，或在后方使用白灯。这些颜色不会随飞行控制器状态变化。
- 对于 `Status/Red` 等 _混合_ 功能，激活模式未激活时，灯显示状态颜色；模式激活后，则切换为“固定”的灯光颜色。

### 激活模式（`UAVCAN_LGT_MODE`）{#UAVCAN_LGT_MODE}

激活模式参数（[UAVCAN_LGT_MODE](#UAVCAN_LGT_MODE)）控制每个灯何时从 _未激活_ 行为切换为 _已激活_ 行为（通过[灯光功能](#light_functions)配置）：

| 值 | 含义                      |
| - | ----------------------- |
| 0 | 始终未激活（灯始终按“模式未激活时”一栏显示） |
| 1 | 解锁时激活（默认）               |
| 2 | 预解锁或解锁时激活               |
| 3 | 始终激活（灯始终按“模式已激活时”一栏显示）  |
