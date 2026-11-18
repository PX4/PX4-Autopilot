# AIFoundry Erbium

<Badge type="tip" text="main (PX4 v2.0)" />
<LinkedBadge type="warning" text="Experimental" url="../flight_controller/autopilot_experimental.md"/>

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://github.com/aifoundry-org) for hardware support or compliance issues.
:::

[Erbium](https://erbium.readthedocs.io/en/latest/) is an AIFoundry RISC-V chip whose design is [open source](https://github.com/aifoundry-org/erbium).
Its CPU subsystem has eight dual-threaded 64-bit [ET-Minion](https://erbium.readthedocs.io/en/latest/minion/) cores (RV64IMFC), derived from the Esperanto ET-SoC-1.
The PX4 target `aifoundry_erbium_default` runs NuttX on a single hart.

## Development Status

Erbium silicon is not yet available.
The target only runs on the `erbium_emu` system emulator from [ET-platform](https://github.com/aifoundry-org/et-platform), and has not been tested on hardware.

The target has no sensor or actuator drivers.
It boots the [SIH](../sim_sih/index.md) quadcopter (airframe `1100`) and keeps parameters and dataman in RAM, so they are lost on reboot.
In the emulator it has been tested to pass the preflight checks, arm, and take off.

The target is not built in PX4 CI, because the CI image does not include a RISC-V toolchain.

## Building Firmware {#building_firmware}

The build uses the `riscv-none-elf` toolchain ([xPack GNU RISC-V Embedded GCC](https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack), tested with 14.3.0-1).
Add its `bin` directory to `PATH`, then [build PX4](../dev_setup/building_px4.md) for this target:

```sh
make aifoundry_erbium_default
```

## Running in the Emulator

Build `erbium_emu` as described in the [NuttX Erbium Minion documentation](https://nuttx.apache.org/docs/latest/platforms/risc-v/erbium/boards/minion/index.html), then start PX4 with:

```sh
erbium_emu -elf_load build/aifoundry_erbium_default/aifoundry_erbium_default.elf \
  -reset_pc 0x40000200 -single_thread -max_cycles 100000000000
```

The NuttX shell runs on the emulator console.
