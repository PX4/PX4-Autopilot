## Power {#power}

The flight controller can be powered from a power module connected to the **POWER 1** or **POWER 2** port, providing redundant power inputs.
If one power module fails the controller automatically switches to the other.

The power module must supply a regulated **5V** at a minimum of **3A continuous**.

Power ports:

- `POWER 1`: 6-pin Molex CLIK-Mate
- `POWER 2`: 6-pin Molex CLIK-Mate

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
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "has_dual_battery_monitoring": true,
    "has_dronecan_power_input": true,
    "power_monitor_type": "ltc44xx"
  },
  "power_ports_wizard": [
    {
      "label": "POWER 1",
      "connector_type": "6-pin Molex CLIK-Mate"
    },
    {
      "label": "POWER 2",
      "connector_type": "6-pin Molex CLIK-Mate"
    }
  ]
}
-->