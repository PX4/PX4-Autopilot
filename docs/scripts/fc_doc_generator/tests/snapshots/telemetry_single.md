## Telemetry Radios (Optional) {#telemetry}

[Telemetry radios](../telemetry/index.md) may be used to communicate and control a vehicle in flight from a ground station (for example, you can direct the UAV to a particular position, or upload a new mission).

The vehicle-based radio should be connected to the **TELEM1** port (if connected to this port, no further configuration is required).
The other radio is connected to your ground station computer or mobile device (usually by USB).

![Telemetry Radio](../../assets/flight_controller/fixture/telemetry_radio.jpg)

<!-- checklist
- [ ] Confirm TELEM port label(s) as printed on board
- [ ] Add a wiring photo showing telemetry radio connected to TELEM1
-->

<!-- telemetry-source-data
{
  "board": "test/fixture",
  "source": {
    "telem_ports": [
      "TELEM1"
    ]
  }
}
-->