### Radio Control {#radio_control}

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

The ports and supported protocols are:

- `PX4IO/RC` (IO): SBUS, DSM/DSMX, CRSF, and GHST receivers

For PPM and S.Bus receivers, a single signal wire carries all channels. If your receiver outputs individual PWM signals (one wire per channel) it must be connected via a [PPM encoder](../getting_started/rc_transmitter_receiver.md#pwm-receivers).

![RC Connection Diagram](../../assets/flight_controller/test_fixture/rc_connection_diagram.png)

<!-- checklist
- [ ] List all RC port names exactly as labeled on the board
- [ ] State which protocols each port supports
- [ ] State the connector type used for each RC port
- [ ] Confirm which protocols have been tested with this board
- [ ] Add a connection diagram image showing where to plug in the RC receiver
-->

<!-- rc-source-data
{
  "board": "test/fixture",
  "modules": {
    "rc_input": false,
    "common_rc": true,
    "px4io": true,
    "has_ppm_pin": true,
    "ppm_shared_with_serial": false
  },
  "rc_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO/RC",
    "side": "IO"
  },
  "io_serial": {
    "device": "/dev/ttyS5",
    "uart": "USART6",
    "label": "PX4IO/RC"
  },
  "rc_ports_wizard": null
}
-->