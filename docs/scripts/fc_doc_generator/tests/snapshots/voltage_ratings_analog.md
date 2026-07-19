## Voltage Ratings {#voltage_ratings}

_Durandal_ is 1-way-redundant on the power supply if 1 power sources are supplied.
The 1 power rail is: **POWER**.

**Normal Operation Maximum Ratings**

Under these conditions all power sources will be used in this order to power the system:

1. **POWER** input (4.9V to 5.5V)

**Absolute Maximum Ratings**

Under these conditions the system will not draw any power (will not be operational), but will remain intact.

1. **POWER** input (operational range 4.9V to 5.5V, 0V to 6V undamaged)
1. **Servo input:** `VDD_SERVO` pin of **FMU PWM OUT** (0V to TODO undamaged)

**Voltage monitoring**

Analog battery monitoring via ADC is enabled by default.

<!-- voltage-ratings-source-data
{
  "board": "test/fixture",
  "source": {
    "num_power_inputs": 1,
    "has_redundant_power": false,
    "power_monitor_type": "analog"
  },
  "overview_wizard": {
    "min_voltage": null,
    "max_voltage": null,
    "usb_pwr_min_v": null,
    "usb_pwr_max_v": null,
    "has_servo_rail": true,
    "servo_rail_absolute_max_v": null
  },
  "power_ports_wizard": [
    {
      "label": "POWER",
      "normal_min_v": "4.9",
      "normal_max_v": "5.5",
      "absolute_max_v": "6"
    }
  ]
}
-->