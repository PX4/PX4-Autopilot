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

The primary has a native PX4 bootloader
(`make cubepilot_cubered-primary_bootloader`). It is wire-compatible with the
ArduPilot bootloader's protocol and additionally programs the external QSPI
flash: the read-only part of the firmware (romfs and the mavlink module) is
linked into a `.extflash` section at the QSPI memory-mapped address
(`0x90000000`). On upload the bootloader receives that section via the
`EXTF_*` protocol commands (erase / program / CRC), writes it to the W25Q256
QSPI flash, and switches the QSPI controller to memory-mapped mode before
jumping to the app so it can execute and read from it directly. The QSPI
driver lives in `src/qspi.c` / `src/extflash.c` and is only built for the
bootloader.

The **secondary** still relies on the ArduPilot bootloader for now: it has no
USB of its own and is flashed over the UART7 link through the primary's
composite USB bootloader. A native PX4 secondary bootloader plus a dual-CDC
passthrough on the primary is planned as a follow-up.

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
