# Bootloader 更新

The _PX4 Bootloader_ is used to load firmware for [Pixhawk boards](../flight_controller/pixhawk_series.md) (PX4FMU, PX4IO).

Pixhawk控制器通常预安装了适当的引导程序。
但在某些情况下，当它不存在，或者有一个的旧版本需要更新。

此篇介绍了更新 Pixhawk bootloader 的几种常见方法。

::: info

- Most boards will need to use the [Debug Probe](#debug-probe-bootloader-update) to update the bootloader.
- On [FMUv6X-RT](../flight_controller/pixhawk6x-rt.md) you can [install bootloader/unbrick boards via USB](bootloader_update_v6xrt.md).
  This is useful if you don't have a debug probe.
- On FMUv2 and some custom firmware (only) you can use [QGC Bootloader Update](#qgc-bootloader-update).

:::

## 构建PX4 启动加载器

### PX4 Bootloader FMUv6X and later

从FMUv6X （STM32H7）的板子开始，使用代码树内的 PX4 bootloader。

This can be built from within the [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) directory using the `make` command and the board-specific target with a `_bootloader` suffix.

对于FMUv6X，命令是：

```sh
make px4_fmu-v6x_bootloader
```

This will build the bootloader binary as `build/px4_fmu-v6x_bootloader/px4_fmu-v6x_bootloader.elf`, which can be flashed via SWD or DFU.
如果你正准备构建 bootloader，你应该已经熟悉这些选项之一。

如果需要 HEX 文件而不是 ELF 文件，请使用 objcopy 参数：

```sh
arm-none-eabi-objcopy -O ihex build/px4_fmu-v6x_bootloader/px4_fmu-v6x_bootloader.elf px4_fmu-v6x_bootloader.hex
```

### PX4 Bootloader FMUv5X and earlier

PX4 boards up to FMUv5X (before STM32H7) used the [PX4 bootloader](https://github.com/PX4/Bootloader) repository.

The instructions in the repo README explain how to use it.

## 构建旧的 PX4 启动加载器

The following steps explain how you can "manually" update the bootloader using a [compatible Debug Probe](../debug/swd_debug.md#debug-probes-for-px4-hardware):

1. Get a binary containing the bootloader (either from dev team or [build it yourself](#building-the-px4-bootloader)).

2. Get a [Debug Probe](../debug/swd_debug.md#debug-probes-for-px4-hardware).
  Connect the probe your PC via USB and setup the `gdbserver`.

3. Go into the directory containing the binary and run the command for your target bootloader in the terminal:

  - FMUv6X

    ```sh
    arm-none-eabi-gdb px4_fmu-v6x_bootloader.elf
    ```

  - FMUv6X-RT

    ```sh
    arm-none-eabi-gdb px4_fmu-v6xrt_bootloader.elf
    ```

  - FMUv5

    ```sh
    ```

  ::: info
  H7 Bootloaders from [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) are named with pattern `*._bootloader.elf`.
  Bootloaders from [PX4/PX4-Bootloader](https://github.com/PX4/PX4-Bootloader) are named with the pattern `*_bl.elf`.

:::

4. The _gdb terminal_ appears and it should display the following output:

  ```sh
  GNU gdb (GNU Tools for Arm Embedded Processors 7-2017-q4-major) 8.0.50.20171128-git
  Copyright (C) 2017 Free Software Foundation, Inc.
  License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
  This is free software: you are free to change and redistribute it.
  There is NO WARRANTY, to the extent permitted by law.
  Type "show copying"    and "show warranty" for details.
  This GDB was configured as "--host=x86_64-linux-gnu --target=arm-none-eabi".
  Type "show configuration" for configuration details.
  For bug reporting instructions, please see:
  <http://www.gnu.org/software/gdb/bugs/>.
  Find the GDB manual and other documentation resources online at:
  <http://www.gnu.org/software/gdb/documentation/>.
  For help, type "help".
  Type "apropos word" to search for commands related to "word"...
  Reading symbols from px4fmuv5_bl.elf...done.
  ```

5. Find your `<dronecode-probe-id>` by running an `ls` command in the **/dev/serial/by-id** directory.

6. Now connect to the debug probe with the following command:

  ```sh
  tar ext /dev/serial/by-id/<dronecode-probe-id>
  ```

7. Power on the Pixhawk with another USB cable and connect the probe to the `FMU-DEBUG` port.

  ::: info
  If using a Dronecode probe you may need to remove the case in order to connect to the `FMU-DEBUG` port (e.g. on Pixhawk 4 you would do this using a T6 Torx screwdriver).

:::

8. Use the following command to scan for the Pixhawk\`s SWD and connect to it:

  ```sh
  (gdb) mon swdp_scan
  (gdb) attach 1
  ```

9. 将二进制文件加载到 Pixhawk 中 ：

  ```sh
  (gdb) load
  ```

After the bootloader has updated you can [Load PX4 Firmware](../config/firmware.md) using _QGroundControl_.

## QGC 引导加载器更新

The easiest approach is to first use _QGroundControl_ to install firmware that contains the desired/latest bootloader.
You can then initiate bootloader update on next restart by setting the parameter: [SYS_BL_UPDATE](../advanced_config/parameter_reference.md#SYS_BL_UPDATE).

This approach can only be used if [SYS_BL_UPDATE](../advanced_config/parameter_reference.md#SYS_BL_UPDATE) is present in firmware.

:::warning
Currently only FMUv2 and some custom firmware includes the desired bootloader.
:::

步骤如下：

1. 插入 SD 卡（使能引导日志记录，便于调试任何可能的问题）。

2. [Update the Firmware](../config/firmware.md#custom) with an image containing the new/desired bootloader.

  ::: info
  The updated bootloader might be supplied in custom firmware (i.e. from the dev team), or it or may be included in the latest main branch.

:::

3. 等待飞控重启。

4. [Find and enable](../advanced_config/parameters.md) the parameter [SYS_BL_UPDATE](../advanced_config/parameter_reference.md#SYS_BL_UPDATE).

5. 重新启动（断开/重新连接飞控板）。
  Bootloader 更新只需要几秒钟即可完成。

Generally at this point you may then want to [update the firmware](../config/firmware.md) again using the correct/newly installed bootloader.

以下是更新FMUv2引导程序的一个具体例子。

### FMUv2 引导加载器更新

If _QGroundControl_ installs the FMUv2 target (see console during installation), and you have a newer board, you may need to update the bootloader in order to access all the memory on your flight controller.

:::info
Early FMUv2 [Pixhawk-series](../flight_controller/pixhawk_series.md#fmu_versions) flight controllers had a [hardware issue](../flight_controller/silicon_errata.md#fmuv2-pixhawk-silicon-errata) that restricted them to using 1MB of flash memory.
这个问题已在新版面解决， 但您可能需要更新工厂提供的引导程序，以便安装 FMUv3 固件并访问所有 2MB 可用内存。
:::

要更新引导器：

1. 插入 SD 卡（使能引导日志记录，便于调试任何可能的问题）。

2. [Update the Firmware](../config/firmware.md) to PX4 _master_ version (when updating the firmware, check **Advanced settings** and then select **Developer Build (master)** from the dropdown list).
  _QGroundControl_ will automatically detect that the hardware supports FMUv2 and install the appropriate Firmware.

  ![FMUv2 update](../../assets/qgc/setup/firmware/bootloader_update.jpg)

  等待飞控重启。

3. [Find and enable](../advanced_config/parameters.md) the parameter [SYS_BL_UPDATE](../advanced_config/parameter_reference.md#SYS_BL_UPDATE).

4. 重新启动（断开/重新连接飞控板）。
  Bootloader 更新只需要几秒钟即可完成。

5. Then [Update the Firmware](../config/firmware.md) again.
  This time _QGroundControl_ should autodetect the hardware as FMUv3 and update the Firmware appropriately.

  ![FMUv3 update](../../assets/qgc/setup/firmware/bootloader_fmu_v3_update.jpg)

  ::: info
  If the hardware has the [Silicon Errata](../flight_controller/silicon_errata.md#fmuv2-pixhawk-silicon-errata) it will still be detected as FMUv2 and you will see that FMUv2 was re-installed (in console).
  在这种情况下，您将无法安装 FMUv3 硬件。

:::

## 其他飞控板（非 Pixhawk）

Boards that are not part of the [Pixhawk Series](../flight_controller/pixhawk_series.md) will have their own mechanisms for bootloader update.

For boards that are preflashed with Betaflight, see [Bootloader Flashing onto Betaflight Systems](bootloader_update_from_betaflight.md).
