# Loading Firmware

_QGroundControl_ **desktop** versions can be used to install PX4 firmware onto [Pixhawk-series](../getting_started/flight_controller_selection.md) flight-controller boards.

:::warning
**Before you start installing Firmware** all USB connections to the vehicle must be _disconnected_ (both direct or through a telemetry radio).
The vehicle must _not be_ powered by a battery.
:::

## Install Stable PX4

Generally you should use the most recent _released_ version of PX4, in order to benefit from bug fixes and get the latest and greatest features.

:::tip
This is the version that is installed by default.
:::

To install PX4:

1. Start _QGroundControl_ and connect the vehicle.
1. Select **"Q" icon > Vehicle Setup > Firmware** (sidebar) to open _Firmware Setup_.

   ![Firmware disconnected](../../assets/qgc/setup/firmware/firmware_disconnected.png)

1. Connect the flight controller directly to your computer via USB.

   ::: info
   Connect directly to a powered USB port on your machine (do not connect through a USB hub).
   :::

1. Select the **PX4 Pro Stable Release vX.x.x** option to install the latest stable version of PX4 _for your flight controller_ (autodetected).

   ![Install PX4 default](../../assets/qgc/setup/firmware/firmware_connected_default_px4.png)

1. Click the **OK** button to start the update.

   The firmware will then proceed through a number of upgrade steps (downloading new firmware, erasing old firmware etc.).
   Each step is printed to the screen and overall progress is displayed on a progress bar.

   ![Firmware upgrade complete](../../assets/qgc/setup/firmware/firmware_upgrade_complete.png)

   Once the firmware has completed loading, the device/vehicle will reboot and reconnect.

   :::tip
   If _QGroundControl_ installs the FMUv2 target (see console during installation) and you have a newer board, you may need to [update the bootloader](#bootloader) in order to access all the memory on your flight controller.
   :::

Next you will need to specify the [vehicle airframe](../config/airframe.md) (and then sensors, radio, etc.)

<a id="custom"></a>

## Installing PX4 Main, Beta or Custom Firmware

To install a different version of PX4:

1. Connect the vehicle as above, and select **PX4 Pro Stable Release vX.x.x**.
   ![Install PX4 version](../../assets/qgc/setup/firmware/qgc_choose_firmware.png)
1. Check **Advanced settings** and select the version from the dropdown list:
   - **Standard Version (stable):** The default version (i.e. no need to use advanced settings to install this!)
   - **Beta Testing (beta):** A beta/candidate release.
     Only available when a new release is being prepared.
   - **Developer Build (master):** The latest build of PX4/PX4-Autopilot _main_ branch.
   - **Custom Firmware file...:** A custom firmware file (e.g. [that you have built locally](../dev_setup/building_px4.md)).
     If you select this you will have to choose the custom firmware from the file system in the next step.

Firmware update then continues as before.

<a id="bootloader"></a>

## Bootloader Update

Pixhawk hardware usually comes with an appropriate bootloader version pre-installed.

A case where you may need to update is newer Pixhawk boards that install FMUv2 firmware.
If _QGroundControl_ installs the FMUv2 target (see console during installation), and you have a newer board, you may need to update the bootloader in order to access all the memory on your flight controller.

![FMUv2 update](../../assets/qgc/setup/firmware/bootloader_update.jpg)

You can update it by following the instructions in [Bootloader update > FMUv2 Bootloader Update](../advanced_config/bootloader_update.md#fmuv2-bootloader-update).

## Further Information

- [QGroundControl User Guide > Firmware](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/firmware.html).
- [PX4 Setup Video](https://youtu.be/91VGmdSlbo4) (Youtube)
