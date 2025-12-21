# MCU-Link Debug Probe

The [MCU-Link Debug Probe](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/mcu-link-debug-probe:MCU-LINK) is a cheap, fast and highly capable debug probe that can serve as a stand-alone debug and console communicator whn working with Pixhawk boards.

Key features:

- Just one single USB-C connection for Reset, SWD, SWO, and serial in a very small package!
- Up to 9.6MBit/s SWO connection.
  Up to 5 MBaud serial. 1.2V to 5V target voltage.
  USB2 high-speed 480 Mbps connection.
- Driven by NXP LinkServer or pyOCD software with wide device support.
- Much cheaper (<15€) than a Pixhawk Debug Adapter (~20€) with a JLink EDU mini (~55€) or JLink BASE (~400€) while having better hardware specs.

The [Pixhawk Debug Adapter](https://holybro.com/products/pixhawk-debug-adapter) provides an easy way to connect a Pixhawk to an MCU-Link (the probe does not come with an adapter for working with Pixhawk flight controllers).

::: info
These instructions have been tested on: FMUv6X-RT, FMUv6X, FMUv6c, FMUv5X.
:::

## Debugging Configuration using NXP LinkServer

The MCU-Link provides for NXP (FMUv6X-RT) chips the [LinkServer](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/linkserver-for-microcontrollers:LINKERSERVER) GDB server:

[Download](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/linkserver-for-microcontrollers:LINKERSERVER#downloads) the Linkserver for your operating system and follow the installation instructions.

On Windows LinkServer gets installed to `C:\NXP\LinkServer_x.x.x`
On Linux LinkServer gets installed `/usr/local/LinkServer/LinkServer`

To flash you can use the `LinkServer flash` command with target `MIMXRT1176xxxxx:MIMXRT1170-EVK-CM7-ONLY` for the FMUv6X-RT

```sh
/usr/local/LinkServer/LinkServer flash MIMXRT1176xxxxx:MIMXRT1170-EVK-CM7-ONLY load build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.elf
```

You can launch the GDB server in a new terminal shell:

```sh
/usr/local/LinkServer/LinkServer gdbserver MIMXRT1176xxxxx:MIMXRT1170-EVK-CM7-ONLY
```

Then connect to port 3333 via GDB:

```sh
arm-none-eabi-gdb build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.elf -ex "target extended-remote :3333"
```

Use GDB to load the binary into the Pixhawk:

```sh
(gdb) load
```

## Debugging Configuration using pyOCD

The MCU-Link provides the [GDB server via pyOCD](https://pyocd.io/):

```sh
python3 -m pip install -U pyocd
```

You can launch the GDB server in a new terminal shell:

```sh
pyocd gdb -t mimxrt1170_cm7
```

The target needs to be one of:

- FMUv6X-RT: `mimxrt1170_cm7`
- FMUv6X: `stm32h743xx`
- FMUv6C: `stm32h743xx`
- FMUv5X: `stm32f767zi`

You can then connect to port 3333 via GDB:

```sh
arm-none-eabi-gdb build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.elf -ex "target extended-remote :3333"
```

Use GDB to load the binary into the Pixhawk:

```sh
(gdb) load
```
