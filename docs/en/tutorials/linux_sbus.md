# Connecting an RC Receiver to a PX4 Linux-based Autopilot

This topic shows how to setup a PX4 Linux-based autopilot to connect and use a [supported RC receiver](../getting_started/rc_transmitter_receiver.md) on any serial port.

For RC types other than S.Bus, you can just connect the receiver directly to the serial ports, or to USB via a USB to TTY serial cable (e.g. like PL2302 USB to Serial TTL converter).

::: info
For an S.Bus receiver (or encoder - e.g. from Futaba, RadioLink, etc.) you will usually need to connect the receiver and device via a [signal inverter circuit](#signal_inverter_circuit), but otherwise the setup is the same.
:::

Then [Start the PX4 RC Driver](#start_driver) on the device, as shown below.

<a id="start_driver"></a>

## Starting the Driver

To start the RC driver on a particular UART (e.g. in this case `/dev/ttyS2`):

```sh
rc_input start -d /dev/ttyS2
```

For other driver usage information see: [rc_input](../modules/modules_driver.md#rc-input).

<a id="signal_inverter_circuit"></a>

## Signal Inverter Circuit (S.Bus only)

S.Bus is an _inverted_ UART communication signal.

While some serial ports/flight controllers can read an inverted UART signal, most require a signal inverter circuit between the receiver and serial port to un-invert the signal.

:::tip
This circuit is also required to read S.Bus remote control signals through the serial port or USB-to-TTY serial converter.
:::

This section shows how to create an appropriate circuit.

### Required Components

- 1x NPN transistor (e.g. NPN S9014 TO92)
- 1x 10K resistor
- 1x 1K resistor

::: info
Any type/model of transistor can be used because the current drain is very low.
:::

### Circuit Diagram/Connections

Connect the components as described below (and shown in the circuit diagram):

- S.Bus signal &rarr; 1K resistor &rarr; NPN transistor base
- NPN transistor emit &rarr; GND
- 3.3VCC &rarr; 10K resistor &rarr; NPN transistor collection &rarr; USB-to-TTY rxd
- 5.0VCC &rarr; S.Bus VCC
- GND &rarr; S.Bus GND

![Signal inverter circuit diagram](../../assets/sbus/driver_sbus_signal_inverter_circuit_diagram.png)

The image below shows the connections on a breadboard.

![Signal inverter breadboard](../../assets/sbus/driver_sbus_signal_inverter_breadboard.png)
