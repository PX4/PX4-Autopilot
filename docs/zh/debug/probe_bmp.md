# Black Magic Probe (and Dronecode Probe)

The [Black Magic Probe](https://black-magic.org) is an easy to use, mostly plug-and-play, JTAG/SWD debugger for embedded microcontrollers.
Since the Black Magic Probe is a generic debug probe, you will need an adapter to connect to Pixhawk flight controllers, which can be purchased here:

- [Drone Code Debug Adapter](https://1bitsquared.com/products/drone-code-debug-adapter) (1 BIT SQUARED).

## Dronecode Probe

The [Dronecode Probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation) is a specialization of the Black Magic Probe for debugging PX4 autopilots.

The probe's USB interface exposes two separate virtual serial port interfaces: one for connecting to the [System Console](system_console.md) (UART) and the other for an embedded GDB server (SWD interface).

The probe provides a DCD-M connector cable for attaching to the [Pixhawk Debug Mini](swd_debug.md#pixhawk-debug-mini).

:::info
The _6-pos DF13_ connector that comes with the probe cannot be used for SWD debugging (it is for using the System Console).
:::

## Using the Probe

:::info
To debug STM32F7 or later (FMUv5 and newer) the Dronecode probe / Blackmagic probe likely requires a firmware update.
You can find how to update the [blackmagic probe here](https://github.com/blacksphere/blackmagic/wiki/Upgrading-Firmware).
:::

To use a Dronecode probe with GDB, start GDB with the exact ELF file that is currently flashed on the autopilot:

```sh
arm-none-eabi-gdb build/px4_fmu-v5_default/px4_fmu-v5_default.elf
```

Then, you have to select the Dronecode probe interface, on Linux this is e.g.:

```sh
target ext /dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_f9414d5_7DB85DAC-if00
```

Then you scan for the target:

```sh
monitor swdp_scan
```

And you should see something like:

```sh
Target voltage: 3.3V
Available Targets:
No. Att Driver
 1      STM32F76x M7
```

Note that for some autopilots it shows 0.0V but the subsequent steps work nevertheless.

You can now attach to that target:

```sh
attach 1
```

And now you should be connected.
