# PX4 소프트웨어 빌드

PX4 firmware can be built from source code on the console or in an IDE, for both simulated and hardware targets.

You need to build PX4 in order to use [simulators](../simulation/index.md), or if you want to modify PX4 and create a custom build.
If you just want to try out PX4 on real hardware then [load the prebuilt binaries](../config/firmware.md) using QGroundControl (there is no need to follow these instructions).

:::info
Before following these instructions you must first install the [Developer Toolchain](../dev_setup/dev_env.md) for your host operating system and target hardware.
If you have any problems after following these steps see the [Troubleshooting](#troubleshooting) section below.
:::

## PX4 소스 코드 다운로드

The PX4 source code is stored on Github in the [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) repository.

To get the _very latest_ (`main` branch) version onto your computer, enter the following command into a terminal:

```sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

Note that you may already have done this when installing the [Developer Toolchain](../dev_setup/dev_env.md)

:::info
This is all you need to do in order to get the latest code.
If needed you can also [get the source code specific to a particular release](../contribute/git_examples.md#get-a-specific-release).
[GIT Examples](../contribute/git_examples.md) provides a lot more information working with releases and contributing to PX4.
:::

## First Build (Using a Simulator)

먼저 콘솔 환경에서 시뮬레이션 대상을 빌드합니다.
이를 통하여 실제 하드웨어와 IDE로 사용전에 시스템 설정을 검증할 수 있습니다.

Navigate into the **PX4-Autopilot** directory and start [Gazebo SITL](../sim_gazebo_gz/index.md) using the following command:

```sh
make px4_sitl gz_x500
```

:::details
If you installed Gazebo Classic
Start  [Gazebo Classic SITL](../sim_gazebo_classic/index.md) using the following command:

```sh
make px4_sitl gazebo-classic
```

:::

This will bring up the PX4 console:

![PX4 Console](../../assets/toolchain/console_gazebo.png)

:::info
You may need to start _QGroundControl_ before proceeding, as the default PX4 configuration requires a ground control connection before takeoff.
This can be [downloaded from here](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html).
:::

The drone can be flown by typing the following command (as shown in the console above):

```sh
pxh> commander takeoff
```

The vehicle will take off and you'll see this in the Gazebo simulator UI:

![Gazebo UI with vehicle taking off](../../assets/toolchain/gazebo_takeoff.png)

The drone can be landed by typing `commander land` and the whole simulation can be stopped by doing **CTRL+C** (or by entering `shutdown`).

Flying the simulation with the ground control station is closer to the real operation of the vehicle.
Click on a location in the map while the vehicle is flying (takeoff flight mode) and enable the slider.
This will reposition the vehicle.

![QGroundControl GoTo](../../assets/toolchain/qgc_goto.jpg)

## NuttX/Pixhawk 기반 보드

### NuttX용 빌드

To build for NuttX- or Pixhawk- based boards, navigate into the **PX4-Autopilot** directory and then call `make` with the build target for your board.

For example, to build for [Pixhawk 4](../flight_controller/pixhawk4.md) hardware you could use the following command:

```sh
cd PX4-Autopilot
make px4_fmu-v4_default
```

A successful run will end with similar output to:

```sh
-- 빌드 파일은 /home/youruser/src/PX4-Autopilot/build/px4_fmu-v4_default에 작성되었습니다.
[954/954] Creating /home/youruser/src/PX4-Autopilot/build/px4_fmu-v4_default/px4_fmu-v4_default.px4
```

The first part of the build target `px4_fmu-v4` indicates the target flight controller hardware for the firmware.
The suffix, in this case `_default`, indicates a firmware _configuration_, such as supporting or omitting particular features.

:::info
The `_default` suffix is optional.
For example, `make px4_fmu-v5` and `px4_fmu-v5_default` result in the same firmware.
:::

The following list shows the build commands for the [Pixhawk standard](../flight_controller/autopilot_pixhawk_standard.md) boards:

- [Holybro Pixhawk 6X-RT (FMUv6X)](../flight_controller/pixhawk6x-rt.md): `make px4_fmu-v6xrt_default`

- [Holybro Pixhawk 6X (FMUv6X)](../flight_controller/pixhawk6x.md): `make px4_fmu-v6x_default`

- [Holybro Pixhawk 6C (FMUv6C)](../flight_controller/pixhawk6c.md): `make px4_fmu-v6c_default`

- [Holybro Pixhawk 6C Mini (FMUv6C)](../flight_controller/pixhawk6c_mini.md): `make px4_fmu-v6c_default`

- [Holybro Pix32 v6 (FMUv6C)](../flight_controller/holybro_pix32_v6.md): `make px4_fmu-v6c_default`

- [Holybro Pixhawk 5X (FMUv5X)](../flight_controller/pixhawk5x.md): `make px4_fmu-v5x_default`

- [Pixhawk 4 (FMUv5)](../flight_controller/pixhawk4.md): `make px4_fmu-v5_default`

- [Pixhawk 4 Mini (FMUv5)](../flight_controller/pixhawk4_mini.md): `make px4_fmu-v5_default`

- [CUAV V5+ (FMUv5)](../flight_controller/cuav_v5_plus.md): `make px4_fmu-v5_default`

- [CUAV V5 nano (FMUv5)](../flight_controller/cuav_v5_nano.md): `make px4_fmu-v5_default`

- [Pixracer (FMUv4)](../flight_controller/pixracer.md): `make px4_fmu-v4_default`

- [Pixhawk 3 Pro](../flight_controller/pixhawk3_pro.md): `make px4_fmu-v4pro_default`

- [Pixhawk Mini](../flight_controller/pixhawk_mini.md): `make px4_fmu-v3_default`

- [Pixhawk 2 (Cube Black) (FMUv3)](../flight_controller/pixhawk-2.md): `make px4_fmu-v3_default`

- [mRo Pixhawk (FMUv3)](../flight_controller/mro_pixhawk.md): `make px4_fmu-v3_default` (supports 2MB Flash)

- [Holybro pix32 (FMUv2)](../flight_controller/holybro_pix32.md): `make px4_fmu-v2_default`

- [Pixfalcon (FMUv2)](../flight_controller/pixfalcon.md): `make px4_fmu-v2_default`

- [Dropix (FMUv2)](../flight_controller/dropix.md): `make px4_fmu-v2_default`

- [Pixhawk 1 (FMUv2)](../flight_controller/pixhawk.md): `make px4_fmu-v2_default`

  :::warning
  You **must** use a supported version of GCC to build this board (e.g. the same as used by [CI/docker](../test_and_ci/docker.md)) or remove modules from the build. PX4가 보드의 1MB 플래시 제한에 가깝기 때문에, 지원되지 않는 GCC로 빌드가 실패할 수 있습니다.

:::

- Pixhawk 1 with 2 MB flash: `make px4_fmu-v3_default`

Build commands for non-Pixhawk NuttX fight controllers (and for all other-boards) are provided in the documentation for the individual [flight controller boards](../flight_controller/index.md).

### 펌웨어 업로드 (보드 플래싱)

Append `upload` to the make commands to upload the compiled binary to the autopilot hardware via USB.
For example

```sh
make px4_fmu-v4_default upload
```

A successful run will end with this output:

```sh
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```

:::tip
This is not supported when developing on WSL2.
See [ Windows Development Environment (WSL2-Based) > Flash a Control Board](../dev_setup/dev_env_windows_wsl.md#flash-a-flight-control-board).
:::

## 기타 보드

Build commands for other boards are given the [board-specific flight controller pages](../flight_controller/index.md) (usually under a heading _Building Firmware_).

You can also list all configuration targets using the command:

```sh
make list_config_targets
```

## 그래픽 IDE에서의 컴파일

[VSCode](../dev_setup/vscode.md) is the officially supported (and recommended) IDE for PX4 development.
It is easy to set up and can be used to compile PX4 for both simulation and hardware environments.

## 문제 해결

### 일반 빌드 오류

Many build problems are caused by either mismatching submodules or an incompletely cleaned-up build environment.
Updating the submodules and doing a `distclean` can fix these kinds of errors:

```sh
git submodule update --recursive
make distclean
```

### Flash overflowed by XXX bytes

The `region 'flash' overflowed by XXXX bytes` error indicates that the firmware is too large for the target hardware platform.
This is common for `make px4_fmu-v2_default` builds, where the flash size is limited to 1MB.

If you're building the _vanilla_ master branch, the most likely cause is using an unsupported version of GCC.
In this case, install the version specified in the [Developer Toolchain](../dev_setup/dev_env.md) instructions.

If building your own branch, it is possible that you have increased the firmware size over the 1MB limit.
PX4 빌드 시스템은 많은 수의 파일을 오픈하므로, 이 갯수를 초과할 수 있습니다.

### macOS: 열린 파일이 너무 많음 오류

MacOS allows a default maximum of 256 open files in all running processes.
The PX4 build system opens a large number of files, so you may exceed this number.

The build toolchain will then report `Too many open files` for many files, as shown below:

```sh
/usr/local/Cellar/gcc-arm-none-eabi/20171218/bin/../lib/gcc/arm-none-eabi/7.2.1/../../../../arm-none-eabi/bin/ld: cannot find NuttX/nuttx/fs/libfs.a: Too many open files
```

The solution is to increase the maximum allowed number of open files (e.g. to 300).
You can do this in the macOS _Terminal_ for each session:

- Run this script [Tools/mac_set_ulimit.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/mac_set_ulimit.sh), or
- 다음 명령어를 실행하십시오.

  ```sh
  ulimit -S -n 300
  ```

### macOS Catalina: cmake 실행 문제

As of macOS Catalina 10.15.1 there may be problems when trying to build the simulator with _cmake_.
다음을 사용하여 누락된 종속성을 확인하여 이러한 경우인지 확인할 수 있습니다.

```sh
xcode-select --install
sudo ln -s /Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/include/* /usr/local/include/
```

### Ubuntu 18.04: arm_none_eabi_gcc와 관련된 컴파일 오류

Build issues related to `arm_none_eabi_gcc`may be due to a broken g++ toolchain installation.
You can verify that this is the case by checking for missing dependencies using:

```sh
arm-none-eabi-gcc --version
arm-none-eabi-g++ --version
arm-none-eabi-gdb --version
arm-none-eabi-size --version
```

Example of bash output with missing dependencies:

```sh
arm-none-eabi-gdb --version
arm-none-eabi-gdb: command not found
```

This can be resolved by removing and [reinstalling the compiler](https://askubuntu.com/questions/1243252/how-to-install-arm-none-eabi-gdb-on-ubuntu-20-04-lts-focal-fossa).

### Ubuntu 18.04: Visual Studio Code는 이 큰 작업 영역에서 파일 변경 사항을 감시할 수 없습니다.

See [Visual Studio Code IDE (VSCode) > Troubleshooting](../dev_setup/vscode.md#troubleshooting).

### Python 패키지를 가져오지 못했습니다.

"Failed to import" errors when running the `make px4_sitl jmavsim` command indicates that some Python packages are not installed (where expected).

```sh
Failed to import jinja2: No module named 'jinja2'
You may need to install it using:
    pip3 install --user jinja2
```

다음과 같이 종속성을 명시적으로 설치하여, 이 문제를 해결할 수 있습니다.

You should be able to fix this by explicitly installing the dependencies as shown:

```sh
pip3 install --user pyserial empty toml numpy pandas jinja2 pyyaml pyros-genmsg packaging
```

## PX4 빌드 타겟 만들기

The previous sections showed how you can call _make_ to build a number of different targets, start simulators, use IDEs etc.
This section shows how _make_ options are constructed and how to find the available choices.

The full syntax to call _make_ with a particular configuration and initialization file is:

```sh
make [VENDOR_][MODEL][_VARIANT] [VIEWER_MODEL_DEBUGGER_WORLD]
```

**VENDOR_MODEL_VARIANT**: (also known as `CONFIGURATION_TARGET`)

- **VENDOR:** The manufacturer of the board: `px4`, `aerotenna`, `airmind`, `atlflight`, `auav`, `beaglebone`, `intel`, `nxp`, etc.
  The vendor name for Pixhawk series boards is `px4`.
- **MODEL:** The _board model_ "model": `sitl`, `fmu-v2`, `fmu-v3`, `fmu-v4`, `fmu-v5`, `navio2`, etc.
- **VARIANT:** Indicates particular configurations: e.g. `bootloader`, `cyphal`, which contain components that are not present in the `default` configuration.
  Most commonly this is `default`, and may be omitted.

:::tip
You can get a list of _all_ available `CONFIGURATION_TARGET` options using the command below:

```sh
make list_config_targets
```

:::

**VIEWER_MODEL_DEBUGGER_WORLD:**

- **VIEWER:** This is the simulator ("viewer") to launch and connect: `gz`, `gazebo`, `jmavsim`, `none` <!-- , ?airsim -->

  :::tip
  `none` can be used if you want to launch PX4 and wait for a simulator (jmavsim, Gazebo, Gazebo Classic, or some other simulator).
  For example, `make px4_sitl none_iris` launches PX4 without a simulator (but with the iris airframe).

:::

- **MODEL:** The _vehicle_ model to use (e.g. `iris` (_default_), `rover`, `tailsitter`, etc), which will be loaded by the simulator.
  The environment variable `PX4_SIM_MODEL` will be set to the selected model, which is then used in the [startup script](../simulation/index.md#startup-scripts) to select appropriate parameters.

- **DEBUGGER:** Debugger to use: `none` (_default_), `ide`, `gdb`, `lldb`, `ddd`, `valgrind`, `callgrind`.
  For more information see [Simulation Debugging](../debug/simulation_debugging.md).

- **WORLD:** (Gazebo Classic only).
  Set the world ([PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds](https://github.com/PX4/PX4-SITL_gazebo-classic/tree/main/worlds)) that is loaded.
  Default is [empty.world](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/worlds/empty.world).
  For more information see [Gazebo Classic > Loading a Specific World](../sim_gazebo_classic/index.md#loading-a-specific-world).

:::tip
You can get a list of _all_ available `VIEWER_MODEL_DEBUGGER_WORLD` options using the command below:

```sh
make px4_sitl list_vmd_make_targets
```

:::

::: info

- Most of the values in the `CONFIGURATION_TARGET` and `VIEWER_MODEL_DEBUGGER` have defaults, and are hence optional.
  For example, `gazebo-classic` is equivalent to `gazebo-classic_iris` or `gazebo-classic_iris_none`.
- 두 개의 다른 설정 사이에 기본값을 지정하려는 경우에는, 세 개의 밑줄을 사용할 수 있습니다.
  For example, `gazebo-classic___gdb` is equivalent to `gazebo-classic_iris_gdb`.
- You can use a `none` value for `VIEWER_MODEL_DEBUGGER` to start PX4 and wait for a simulator.
  For example start PX4 using `make px4_sitl_default none` and jMAVSim using `./Tools/simulation/jmavsim/jmavsim_run.sh -l`.

:::

The `VENDOR_MODEL_VARIANT` options map to particular _px4board_ configuration files in the PX4 source tree under the [/boards](https://github.com/PX4/PX4-Autopilot/tree/main/boards) directory.
Specifically `VENDOR_MODEL_VARIANT` maps to a configuration file **boards/VENDOR/MODEL/VARIANT.px4board**
(e.g. `px4_fmu-v5_default` corresponds to [boards/px4/fmu-v5/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board)).

추가 make 대상은 관련 섹션에서 설명합니다.

- `bloaty_compare_master`: [Binary Size Profiling](../debug/binary_size_profiling.md)
- ...

## Firmware Version & Git Tags

The _PX4 Firmware Version_ and _Custom Firmware Version_ are published using the MAVLink [AUTOPILOT_VERSION](https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION) message, and displayed in the _QGroundControl_ **Setup > Summary** airframe panel:

![Firmware info](../../assets/gcs/qgc_setup_summary_airframe_firmware.jpg)

These are extracted at build time from the active _git tag_ for your repo tree.
The git tag should be formatted as `<PX4-version>-<vendor-version>` (e.g. the tag in the image above was set to `v1.8.1-2.22.1`).

:::warning
If you use a different git tag format, versions information may not be displayed properly.
:::
