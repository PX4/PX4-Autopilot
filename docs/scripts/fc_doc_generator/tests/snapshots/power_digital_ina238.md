## Power {#power}

The flight controller can be powered from a power module connected to the **TODO: POWER port label** port.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `TODO: POWER port label`: TODO: connector type

:::warning
The PWM output ports are not powered by the POWER port.
The output rail must be [separately powered](../assembly/servo_power.md) if it needs to power servos or other hardware.
This is generally true for VTOL and fixed-wing vehicles, but not necessarily true for MC vehicles.
:::

For battery and power module configuration see [Battery and Power Module Setup](../config/battery.md).

![Power Connection](../../assets/flight_controller/fixture/power_connection.png)

<!-- checklist
- [ ] Confirm power port label(s) as printed on board
- [ ] Confirm connector type(s)
- [ ] Confirm voltage/current ratings of power module used
- [ ] Add a connection diagram image showing power port location
-->

<!-- power-source-data
{
  "board": "test/fixture",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "has_dual_battery_monitoring": false,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ina238"
  },
  "power_ports_wizard": null
}
-->