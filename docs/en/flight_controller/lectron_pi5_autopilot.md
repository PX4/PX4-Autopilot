# Lectron FMU-V6X Flight Controller

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://lectrontech.com/contact-us/) for hardware support or compliance issues.
:::

The Raspberry CM5 Board is designed as an integrated flight control and computing platform for autonomous systems and advanced embedded applications. The hardware architecture consolidates real-time flight control and high-level computing into a single unified board.

This approach simplifies system integration, reduces cabling complexity, and improves overall system reliability.

For more detailed technical specifications and information, please visit the [technical documentation page](https://lectronuser.github.io/Lectron-Doc-Center/md/raspberry/).


![Lectron pi5 Main Photo](../../assets/flight_controller/lectron/lectron_pi5_autopilot.png)


## Sensors

- Dual ICM-42670-P IMUs
- Bosch BMI270 IMUs
- Dual Bosch BMP390 Barometers
- Bosch BMM350 Magnetometer

## Microprocessor

- STM32H753IIK6TR
	- 480MHz
	- 2MB Flash
	- 1MB Flash

## Power Requirements

- **Input Voltage:** 12V – 25V (3S-6S LiPo)

## Additional Information



## Building Firmware

```sh
make lectron_pi5-autopilot_default
```

## See Also

- [Lectron Pi5 Autopilot](https://lectronuser.github.io/Lectron-Doc-Center/md/raspberry/) (Lectron Pi5 Autopilot Docs)
