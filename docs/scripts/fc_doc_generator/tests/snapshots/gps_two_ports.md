### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)
- `GPS2` (FMU): TODO: connector type — GPS, compass (I2C)

The GPS module's integrated safety switch is enabled _by default_ (when enabled, PX4 will not let you arm the vehicle).
To disable the safety switch press and hold it for 1 second.
You can press the safety switch again to enable safety and disarm the vehicle.

![GNSS Connection](../../assets/flight_controller/test_fixture/gnss_connection.png)

<!-- checklist
- [ ] Confirm physical label(s) of GPS port(s) as printed on board
- [ ] Confirm connector type (e.g. 10-pin JST GH Pixhawk full or 6-pin basic)
- [ ] Confirm whether GPS1 includes safety switch, buzzer and LED pins
- [ ] Note whether an external compass is required or integrated in GPS module
- [ ] Add a connection diagram image showing GPS port location
-->

<!-- gps-source-data
{
  "board": "test/fixture",
  "source": {
    "gps_ports": [
      {
        "label": "GPS1",
        "device": "/dev/ttyS0",
        "uart": "USART1"
      },
      {
        "label": "GPS2",
        "device": "/dev/ttyS3",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": true,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->