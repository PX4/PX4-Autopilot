## Voltage Ratings {#voltage_ratings}

_Pixhawk 5X_ can be triple-redundant on the power supply if 3 power sources are supplied.
The 3 power rails are: **POWER1**, **POWER2** and **USB**.

**Normal Operation Maximum Ratings**

Under these conditions all power sources will be used in this order to power the system:

1. **POWER1** input (4.9V to 5.5V)
1. **POWER2** input (4.9V to 5.5V)
1. **USB** input (4.75V to 5.25V)

**Absolute Maximum Ratings**

Under these conditions the system will not draw any power (will not be operational), but will remain intact.

1. **POWER1** input (operational range 4.9V to 5.5V, 0V to 10V undamaged)
1. **POWER2** input (operational range 4.9V to 5.5V, 0V to 10V undamaged)
1. **USB** input (operational range 4.75V to 5.25V, 0V to 6V undamaged)
1. **Servo input:** `VDD_SERVO` pin of **FMU PWM OUT** (0V to 42V undamaged)

**Voltage monitoring**

Digital I2C battery monitoring is enabled by default.

<!-- voltage-ratings-source-data
{
  "board": "test/fixture",
  "source": {
    "num_power_inputs": 2,
    "has_redundant_power": true,
    "power_monitor_type": "ina226"
  },
  "overview_wizard": {
    "min_voltage": null,
    "max_voltage": null,
    "usb_pwr_min_v": "4.75",
    "usb_pwr_max_v": "5.25",
    "has_servo_rail": true,
    "servo_rail_absolute_max_v": "42"
  },
  "power_ports_wizard": [
    {
      "label": "POWER1",
      "normal_min_v": "4.9",
      "normal_max_v": "5.5",
      "absolute_max_v": "10"
    },
    {
      "label": "POWER2",
      "normal_min_v": "4.9",
      "normal_max_v": "5.5",
      "absolute_max_v": "10"
    }
  ]
}
-->