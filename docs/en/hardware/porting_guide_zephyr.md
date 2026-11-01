# Zephyr Board Porting Guide

<Badge type="warning" text="Experimental" />

::: warning
The Zephyr port is experimental.
PX4 only flies in simulation on Zephyr, as there are no sensor drivers, actuator outputs or RC input yet (see [Limitations](#limitations)).
:::

PX4 can run on the [Zephyr RTOS](https://zephyrproject.org/), on top of its POSIX API.

PX4 is built with Zephyr's tools: [west](https://docs.zephyrproject.org/latest/develop/west/index.html) builds the firmware and [twister](https://docs.zephyrproject.org/latest/develop/test/twister.html) runs the tests.

## Architecture

The PX4-Autopilot repository is a [Zephyr module](https://docs.zephyrproject.org/latest/develop/modules.html) ([zephyr/module.yml](https://github.com/PX4/PX4-Autopilot/blob/main/zephyr/module.yml)).
A Zephyr application that sets `CONFIG_PX4=y` gets PX4 built into it:

- The PX4 modules are selected by a [PX4 board configuration](porting_guide_config.md) (`.px4board` file), as on the other platforms.
  The configuration in `boards/zephyr/<board>/` is used if it exists for the Zephyr board, and [boards/zephyr/generic](https://github.com/PX4/PX4-Autopilot/tree/main/boards/zephyr/generic) otherwise.
- The operating system is configured with Zephyr Kconfig (`prj.conf`) and devicetree, as for any Zephyr application.
- The platform layer in [platforms/zephyr](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr) runs PX4 on Zephyr's POSIX API.
  PX4 tasks and work queues are preemptible Zephyr threads, and the high-resolution timer counts the cycles of the system timer, with its callouts on the kernel tick (100 kHz by default).
  PX4 is built as on NuttX, with `-Os`, and its estimators and controllers with `-O2`.
  A board with flash to spare can build it for speed with `CONFIG_SPEED_OPTIMIZATIONS`.
- The application calls `px4_zephyr_start()`, which initialises PX4 and runs the commands of the board's `init/rc.px4` startup script, one per line.
  A board configuration with a [label](porting_guide_config.md#px4-kconfig-label-inheritance) runs `init/rc.<label>.px4` instead, if it exists.
- The `px4` command of the Zephyr shell runs PX4 commands, for example `px4 uorb status`.

The PX4 options of the Zephyr configuration are:

| Option                      | Description                                                                                         |
| --------------------------- | --------------------------------------------------------------------------------------------------- |
| `CONFIG_PX4`                | Build PX4 into the application.                                                                     |
| `CONFIG_PX4_BOARD_CONFIG`   | PX4 board configuration to build instead of the default one, for example `zephyr_generic_sih`.      |
| `CONFIG_PX4_SHELL`          | Add the `px4` shell command (enabled with the shell).                                               |
| `CONFIG_PX4_STACK_OVERHEAD` | Bytes added to the stack of each PX4 task and work queue.                                           |

## Setting Up a Workspace

PX4-Autopilot is the manifest repository of the west workspace, which fetches the version of Zephyr that PX4 is tested with:

```sh
mkdir px4-zephyr && cd px4-zephyr
git clone https://github.com/PX4/PX4-Autopilot.git
python3 -m venv .venv
source .venv/bin/activate
pip install west
west init -l --mf platforms/zephyr/west.yml PX4-Autopilot
west update --narrow
west packages pip --install
west sdk install
```

The PX4 build only fetches the Git submodules it uses.

## Building and Running in QEMU

The reference application is in [platforms/zephyr/app](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/app).
Build it and run it in QEMU, emulating an Arm Cortex-M4F board (`mps2/an386`):

```sh
west build -b mps2/an386 PX4-Autopilot/platforms/zephyr/app -t run
```

or, from the **PX4-Autopilot** directory:

```sh
make zephyr_qemu
```

PX4 starts and the Zephyr shell runs PX4 commands:

```sh
uart:~$ px4 uorb status
uart:~$ px4 work_queue status
```

Exit QEMU with **Ctrl+A**, **X**.

## Flying in Simulation (SIH)

PX4 flies a quadcopter in QEMU with the [SIH](../sim_sih/index.md) simulator, which runs in PX4 itself, as on a flight controller (airframe 1100):

```sh
west build -b mps2/an386 PX4-Autopilot/platforms/zephyr/app -t run -- -DFILE_SUFFIX=sih
```

or `make zephyr_sih`.

The `sih` [file suffix](https://docs.zephyrproject.org/latest/develop/application/index.html#application-file-suffixes) selects `prj_sih.conf`, which builds the `zephyr_generic_sih` PX4 board configuration ([sih.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/zephyr/generic/sih.px4board) and [rc.sih.px4](https://github.com/PX4/PX4-Autopilot/blob/main/boards/zephyr/generic/init/rc.sih.px4)), and `boards/mps2_an386_sih.conf`.
The emulated CPU runs at 250 MHz, the speed of a flight controller.

Fly from the Zephyr shell:

```sh
uart:~$ px4 commander takeoff
uart:~$ px4 listener vehicle_local_position
uart:~$ px4 commander land
```

or with _QGroundControl_ on the host, which connects automatically: MAVLink is sent over UDP to port 14550 of the host, with [QEMU user networking](https://docs.zephyrproject.org/latest/connectivity/networking/qemu_user_setup.html).

## Building for Other Boards

To build the application for another Zephyr board:

```sh
make zephyr ZEPHYR_BOARD=<board>
```

## Running the Tests

The tests run in QEMU with twister:

```sh
west twister -T PX4-Autopilot/platforms/zephyr -p mps2/an386
```

or `make zephyr_tests`.

- [tests/platform](https://github.com/PX4/PX4-Autopilot/tree/main/platforms/zephyr/tests/platform) tests the platform layer: time, callouts from interrupts, tasks, work queues, uORB and parameters.
- The application tests check that PX4 starts, and fly the SIH quadcopter: take off, hover and land.

## Adding a Board

A board supported by Zephyr runs PX4 without any PX4 board files.

A board that needs its own PX4 modules or startup commands gets a PX4 board configuration in `boards/zephyr/<board>/`, named after the Zephyr board without its qualifiers (for example `vmu_rt1170`).
Copy [boards/zephyr/generic](https://github.com/PX4/PX4-Autopilot/tree/main/boards/zephyr/generic) as a starting point.

## Limitations

- PX4 only flies in simulation: there are no sensor drivers, actuator outputs, RC input or logging.
- MAVLink is only sent over UDP, not over serial ports.
- Parameters are kept in RAM only.
- Dynamic modules (`DYNAMIC` in `px4_add_module()`) are not supported.
- `native_sim` is not supported, as its POSIX API uses the host C library.
- Only the QEMU board is tested.

## Roadmap

- Sensors with the Zephyr sensor API.
- Actuator outputs with the Zephyr PWM API.
- MAVLink over serial ports.
- Parameter and log storage on the Zephyr file systems or ZMS.
- Dynamic modules as Zephyr loadable extensions (below).

### Loadable Modules

On Linux, PX4 builds `DYNAMIC` modules as shared libraries, which the `dyn` command loads.
On Zephyr, the plan is to build them as [LLEXT](https://docs.zephyrproject.org/latest/services/llext/index.html) extensions:

- `px4_add_module(... DYNAMIC)` creates the extension with `add_llext_target()`.
- `dyn` loads the extension from a file system with `llext_load()`, runs its constructors with `llext_bringup()`, finds its `px4_module_main` entry point with `llext_find_sym()`, and runs it as a PX4 task.
- The firmware exports the PX4 API that modules use (uORB, parameters, work queues, performance counters and logging), from a list generated at build time.
- The extensions can be tested in QEMU, as Zephyr supports LLEXT on Arm Cortex-M.
