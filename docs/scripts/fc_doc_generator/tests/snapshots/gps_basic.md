### GPS & Compass {#gps_compass}

PX4 supports GPS modules connected to the GPS port(s) listed below.
The module should be [mounted on the frame](../assembly/mount_gps_compass.md) as far away from other electronics as possible, with the direction marker pointing towards the front of the vehicle.

The GPS ports are:

- `GPS1` (FMU): TODO: connector type — GPS, compass (I2C)

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
        "device": "/dev/ttyS3",
        "uart": "UART4"
      }
    ],
    "has_pps_capture": false,
    "has_safety_switch": false,
    "has_safety_led": false,
    "has_buzzer": false
  },
  "gps_ports_wizard": null
}
-->