# Power Systems

UAVs require a regulated power supply for the flight controller, along with separate power for the motors, servos, and any other peripherals.
The power is usually supplied from a battery (or batteries), though generators and other systems can be used.

Power modules are commonly used to "split off" a regulated power supply for the flight controller and also measure the battery voltage and total current consumed by the vehicle.
PX4 can use this information to infer the remaining battery capacity and provide low-power warnings and other failsafe behaviour.

A Power Distribution Board (PDB) may be used to simplify the wiring for splitting the output of the battery to the flight controller, motors, and other peripherals.
PDBs will sometimes include a power module, ESCs for motors, and a battery elimination circuit (BEC) for powering servos.

PX4 can also receive more comprehensive battery/power-supply information as MAVLink telemetry instead of using a power module.
Batteries that can _supply_ MAVLink information are sometimes referred to as "Smart Batteries" (this definition is open for debate).

- [Power Modules/PDB](../power_module/index.md)
- [Smart/MAVLink Batteries](../smart_batteries/index.md)
