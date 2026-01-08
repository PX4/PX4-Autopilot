# ESCs & Motors

Many PX4 drones use brushless motors that are driven by the flight controller via an Electronic Speed Controller (ESC).
The ESC takes a signal from the flight controller and uses it to set control the level of power delivered to the motor.

PX4 supports a number of [common protocols](../esc/esc_protocols.md) for sending the signals to ESCs: [PWM ESCs](../peripherals/pwm_escs_and_servo.md), [OneShot ESCs](../peripherals/oneshot.md), [DShot ESCs](../peripherals/dshot.md), [DroneCAN ESCs](../dronecan/escs.md), PCA9685 ESC (via I2C), and some UART ESCs (from Yuneec).

## Supported ESC

The following list is non-exhaustive.

| ESC Device                     | Protocols                            | Firmwares                | Notes                                                 |
| ------------------------------ | ------------------------------------ | ------------------------ | ----------------------------------------------------- |
| [ARK 4IN1 ESC]                 | [Dshot], [PWM]                       | [AM32]                   | Has versions with/without connnectors                 |
| [Holybro Kotleta 20]           | [DroneCAN], [PWM]                    | [PX4 Sapog ESC Firmware] |                                                       |
| [Vertiq Motor & ESC modules]   | [Dshot], [OneShot], Multishot, [PWM] | Vertiq firmware          | Larger modules support DroneCAN, ESC and Motor in one |
| [RaccoonLab CAN PWM ESC nodes] | [DroneCAN], Cyphal                   |                          | Cyphal and DroneCAN notes for PWM ESC                 |
| [VESC ESCs]                    | [DroneCAN], [PWM]                    | VESC project firmware    |                                                       |
| [Zubax Telega]                 | [DroneCAN], [PWM]                    | Telega-based             | ESC and Motor in one                                  |

<!-- Links for table above -->

[ARK 4IN1 ESC]: ../esc/ark_4in1_esc.md
[AM32]: https://am32.ca/
[PX4 Sapog ESC Firmware]: ../dronecan/sapog.md
[VESC ESCs]: ../peripherals/vesc.md
[DroneCAN]: ../dronecan/escs.md
[Dshot]: ../peripherals/dshot.md
[OneShot]: ../peripherals/oneshot.md
[PWM]: ../peripherals/pwm_escs_and_servo.md
[Holybro Kotleta 20]: ../dronecan/holybro_kotleta.md
[Vertiq Motor & ESC modules]: ../peripherals/vertiq.md
[RaccoonLab CAN PWM ESC nodes]: ../dronecan/raccoonlab_nodes.md
[Zubax Telega]: ../dronecan/zubax_telega.md

## See Also

For more information see:

- [ESC Protocols](../esc/esc_protocols.md) — overview of main ESC/Servo protocols supported by PX4
- [PWM ESCs and Servos](../peripherals/pwm_escs_and_servo.md)
- [OneShot ESCs and Servos](../peripherals/oneshot.md)
- [DShot](../peripherals/dshot.md)
- [DroneCAN ESCs](../dronecan/escs.md)
- [ESC Calibration](../advanced_config/esc_calibration.md)
- [ESC Firmware and Protocols Overview](https://oscarliang.com/esc-firmware-protocols/) (oscarliang.com)
