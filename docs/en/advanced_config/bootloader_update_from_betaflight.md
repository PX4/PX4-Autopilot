# PX4 Bootloader Flashing onto Betaflight Systems

This page documents how to flash the PX4 bootloader onto boards that are already flashed with Betaflight (e.g. [OmnibusF4 SD](../flight_controller/omnibus_f4_sd.md) or [Kakute F7](../flight_controller/kakutef7.md)).

There are three tools that can be used to flash the PX4 bootloader: _Betaflight Configurator_, [dfu-util](http://dfu-util.sourceforge.net/) command line tool, or the graphical [dfuse](https://www.st.com/en/development-tools/stsw-stm32080.html) (Windows only).

::: info
The _Betaflight Configurator_ is easiest, but newer versions may not support non-betaflight bootloader update.
You might try it first, but use the other methods if firmware update does not work.
:::

## Betaflight Configurator Bootloader Update

::: info
_Betaflight Configurator_ may not support PX4 Bootloader update, as of May 2023.
Older versions should work, though the precise versions are not known.
:::

To install the PX4 bootloader using the _Betaflight Configurator_:

1. Download or build [bootloader firmware](#bootloader-firmware) for the board you want to flash.
1. Download the [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) for your platform.

   :::tip
   If using the _Chrome_ web browser, a simple cross-platform alternative is to install the configurator as an [extension from here](https://chrome.google.com/webstore/detail/betaflight-configurator/kdaghagfopacdngbohiknlhcocjccjao).
   :::

1. Connect the board to your PC and start the Configurator.
1. Press the **Load Firmware [Local]** button
   ![Betaflight Configurator - Local Firmware](../../assets/flight_controller/omnibus_f4_sd/betaflight_configurator.jpg)
1. Select the bootloader binary from the file system and then flash the board.

You should now be able to install PX4 firmware on the board.

## DFU Bootloader Update

This section explains how to flash the PX4 bootloader using the [dfu-util](http://dfu-util.sourceforge.net/) or the graphical [dfuse](https://www.st.com/en/development-tools/stsw-stm32080.html) tool (Windows only).

You will first need to download or build [bootloader firmware](#bootloader-firmware) for the board you want to flash (below, this is referred to as `<target.bin>`).

::: info
All of the methods below are safe as the STM32 MCU cannot be bricked!
DFU cannot be overwritten by flashing and will always allow you to install a new firmware, even if flashing fails.
:::

### DFU mode

Both tools require the board to be in DFU mode.
To enter DFU mode, hold the boot button down while connecting the USB cable to your computer.
The button can be released after the board has powered up.

### dfu-util

::: info
The [Holybro Kakute H7 v2](../flight_controller/kakuteh7v2.md), [Holybro Kakute H7](../flight_controller/kakuteh7.md) and [mini](../flight_controller/kakuteh7mini.md) flight controllers may require that you first run an additional command to erase flash parameters (in order to fix problems with parameter saving):

```
dfu-util -a 0 --dfuse-address 0x08000000:force:mass-erase:leave -D build/<target>/<target>.bin
```

The command may generate an error which can be ignored.
Once completed, enter DFU mode again to complete the regular flashing.
:::

To flash the bootloader onto the flight controller:

```
dfu-util -a 0 --dfuse-address 0x08000000 -D  build/<target>/<target>.bin
```

Reboot the flight controller and it let it boot without holding the boot button.

### dfuse

The dfuse manual can be found here: https://www.st.com/resource/en/user_manual/cd00155676.pdf

Use the tool to flash the `<target>.bin` file.

## Bootloader Firmware

The tools above flash pre-built bootloader firmware.
Bootloader firmware can be built for most targets using the normal PX4 source code, while other targets can only be build using source in the bootloader repository.

Flight controllers that have bootloader PX4-Autopilot `make` targets, can build the bootloader from the PX4-Autopilot source.
The list of controllers for which this applies can be obtained by running the following `make` command, and noting the `make` targets that end in `_bootloader`

```
$make list_config_targets

...
cuav_nora_bootloader
cuav_x7pro_bootloader
cubepilot_cubeorange_bootloader
holybro_durandal-v1_bootloader
holybro_kakuteh7_bootloader
holybro_kakuteh7v2_bootloader
holybro_kakuteh7mini_bootloader
matek_h743-mini_bootloader
matek_h743-slim_bootloader
modalai_fc-v2_bootloader
mro_ctrl-zero-classic_bootloader
mro_ctrl-zero-h7_bootloader
mro_ctrl-zero-h7-oem_bootloader
mro_pixracerpro_bootloader
px4_fmu-v6u_bootloader
px4_fmu-v6x_bootloader
```

To build for these flight controllers, download and build the [PX4-Autopilot source](https://github.com/PX4/PX4-Autopilot), and then make the target using the following commands:

```sh
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make <target> # For example: holybro_kakuteh7mini_bootloader
```

For other flight controllers download the [PX4/Bootloader](https://github.com/PX4/Bootloader) repository and build the source code using the appropriate targets:

```
git clone --recursive  https://github.com/PX4/Bootloader.git
cd Bootloader
make <target> # For example: omnibusf4sd_bl or kakutef7_bl
```

## Reinstall Betaflight

In order to switch back to _Betaflight_:

1. Backup the PX4 parameters.
   You can do this by [exporting](../advanced/parameters_and_configurations.md#exporting-and-loading-parameters) them to an SD card.
1. Keep the **bootloader** button pressed while attaching the USB cable
1. Flash _Betaflight_ as usual with the _Betaflight-configurator_
