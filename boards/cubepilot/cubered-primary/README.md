# PX4 on CubePilot CubeRed

CubePilot CubeRed is a dual-MCU autopilot built around two STM32H743 MCUs:

- **Primary** (`cubepilot_cubered-primary`) runs the main PX4 flight stack
  and the `cubered_bridge_primary` driver
  (`src/drivers/cubered_bridge/primary`).
- **Secondary** (`cubepilot_cubered-secondary`) runs the
  `cubered_bridge_secondary` driver
  (`src/drivers/cubered_bridge/secondary`), which speaks the PX4IO register
  protocol so the primary can drive PWM/RC over the UART7 link between the
  two MCUs.

References:
- [Hardware overview](https://docs.cubepilot.org/user-guides/autopilot/cube-red)
- [Carrier board pinout](https://docs.cubepilot.org/user-guides/carrier-boards/cube-red-standard-carrier-board-pinout)

## Bootloader

PX4 currently uses the **ArduPilot bootloader** on this board. A native PX4
bootloader is not yet available, so the upload protocol is driven through
ArduPilot's bootloader and the secondary is exposed as the second device of
the primary's composite USB bootloader.

## Building and flashing

Both MCUs need to be flashed. Build and upload the secondary first, then
the primary:

```sh
make cubepilot_cubered-secondary_default upload && \
make cubepilot_cubered-primary_default upload
```

## Console

- Primary console is on the carrier's GPS2 connector (UART4) at 57600 baud.
- Secondary console is on the carrier's CONS connector (UART8) at 57600 baud.
