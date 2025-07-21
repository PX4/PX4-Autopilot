# Smart Batteries

Smart Batteries provide more accurate (and often more detailed) information about the state of a battery than an autopilot can estimate for "dumb" batteries.
This allows for more more reliable flight planning notification of failure conditions.
The information may include some of: remaining charge, time-to-empty (estimated), cell voltages (rated max/min, current voltage, etc.), temperature, currents, fault information, battery vendor, chemistry, etc.

PX4 supports (at least) following smart batteries:
* [Rotoye Batmon](../smart_batteries/rotoye_batmon.md)

### Further Information

- [Mavlink Battery Protocol](https://mavlink.io/en/services/battery.html)
- [batt_smbus](../modules/modules_driver.md) - PX4 SMBus Battery Driver docs
- [Safety > Low Battery Failsafe](../config/safety.md#battery-level-failsafe).
