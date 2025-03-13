# Power Modules & Power Distribution Boards

_Power Modules_ provide a regulated power supply for the flight controller (FC), along with information about battery voltage and current.
The voltage/current information is used to determine the consumed power, and to hence to estimate remaining battery capacity.
This in turn allows the flight controller to provide failsafe warnings and other actions in the event of low power: [Safety > Low Battery Failsafe](../config/safety.md#battery-level-failsafe).

_Power Distribution Boards (PDB)_ are circuit boards that may be used to simplify splitting the output of the battery to various parts of the drone, and in particular motors.
PDBs will sometimes include a power module for the FC, ESCs for motors, and a battery elimination circuit (BEC) for powering servos.

The PX4 battery/power module configuration (via the ADC interface) is covered in: [Battery Estimation Tuning](../config/battery.md).

::: tip
For easiest assembly use a power module or PDB recommended by your FC manufacturer, and sized for your power requirements.

The Pixhawk connector standard requires that the VCC line must provide at least 2.5A continuous current and default to 5.3V.
In in practice flight controllers may have different recommendations or preferences, so if you don't (or can't) use a recommended module, check that the module matches your FC's requirements.
:::

This section provides information about a number of power modules and power distribution boards (see FC manufacturer docs for more options):

- Analog voltage and current power modules:
  - [CUAV HV PM](../power_module/cuav_hv_pm.md)
  - [Holybro PM02](../power_module/holybro_pm02.md)
  - [Holybro PM07](../power_module/holybro_pm07_pixhawk4_power_module.md)
  - [Holybro PM06 V2](../power_module/holybro_pm06_pixhawk4mini_power_module.md)
  - [Sky-Drones SmartAP PDB](../power_module/sky-drones_smartap-pdb.md)
- Digital (I2C) voltage and current power modules (for Pixhawk FMUv6X and FMUv5X derived controllers):
  - [ARK PAB Power Module](../power_module/ark_pab_power_module.md)
  - [ARK 12S PAB Power Module](../power_module/ark_12s_pab_power_module.md)
  - [Holybro PM02D](../power_module/holybro_pm02d.md)
  - [Holybro PM03D](../power_module/holybro_pm03d.md)
- [DroneCAN](../dronecan/index.md) power modules
  - [CUAV CAN PMU](../dronecan/cuav_can_pmu.md)
  - [Pomegranate Systems Power Module](../dronecan/pomegranate_systems_pm.md)
  - [RaccoonLab Power Connectors and PMU](../dronecan/raccoonlab_power.md)
