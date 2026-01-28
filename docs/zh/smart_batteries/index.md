# 智能电池

智能电池提供的电池状态信息比自动驾驶仪估计的“哑”电池更准确(通常也更详细)。
这使得能够更可靠地通知故障条件。
这些信息可能包括:剩余电量、空电时间(估计)、电池电压(额定最大/最小电压、电流电压等)、温度、电流、故障信息、电池供应商、化学成分等。

PX4（至少）支持以下智能电池：

- [Rotoye 电池监测器](../smart_batteries/rotoye_batmon.md)

### 更多信息

- [Mavlink 电池协议](https://mavlink.io/en/services/battery.html)
- [batt_smbus](../modules/modules_driver.md) - PX4 系统管理总线（SMBus）电池驱动文档
- [安全 > 低电量故障保护](../config/safety.md#battery-level-failsafe)。
