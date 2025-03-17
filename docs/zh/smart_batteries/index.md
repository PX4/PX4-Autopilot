# 智能电池

智能电池提供的电池状态信息比自动驾驶仪估计的“哑”电池更准确(通常也更详细)。
This allows for more more reliable flight planning notification of failure conditions.
这些信息可能包括:剩余电量、空电时间(估计)、电池电压(额定最大/最小电压、电流电压等)、温度、电流、故障信息、电池供应商、化学成分等。

PX4至少支持以下智能电池:

- [Rotoye Batmon](../smart_batteries/rotoye_batmon.md)

### 更多信息

- [Mavlink Battery Protocol](https://mavlink.io/en/services/battery.html)
- [batt_smbus](../modules/modules_driver.md) - PX4 SMBus Battery Driver docs
- [Safety > Low Battery Failsafe](../config/safety.md#battery-level-failsafe).
