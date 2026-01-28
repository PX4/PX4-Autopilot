# Raspberry Pi Companion with Pixhawk

This topic describes how to setup a Raspberry Pi ("RPi") companion companion running [ROS 2](../ros2/user_guide.md) on Linux Ubuntu OS, connecting to a [Pixhawk](../flight_controller/autopilot_pixhawk_standard.md) flight controller using a serial connection between the Pixhawk `TELEM2` port and the RPi's TX/RX pins.

These instructions should be readily extensible to other RPi and flight controller configurations.

::: info
Other common ways to connect RPi and Pixhawk are:

- Ethernet connection between RPi and Pixhawk.
  Pixhawk controllers based on FMUv5x, FMUv6x and later may have an inbuilt Ethernet port.
  See [PX4 Ethernet > Supported Controllers](../advanced_config/ethernet_setup.md#supported-flight-controllers).
- Serial connection to the RPi USB port.
  This is simple and reliable, but requires an additional FTDI Chip USB-to-serial adapter board.
  This option is covered in [Pixhawk Companion > Serial Port Setup](../companion_computer/pixhawk_companion.md#serial-port-setup).
  :::

## Wiring

### Serial connection

First wire up the serial connection between the RPi and PX4 that is to be used for offboard control.

This setup connects the Pixhawk `TELEM2` port, which is generally recommended for offboard control.
It is initially configured in PX4 to use with MAVLink, which we will change later when setting up ROS 2.
Pixhawk ports can be located anywhere on the flight controller, but are almost always well labeled, and should be obvious on your particular [flight controller](../flight_controller/index.md).

Connect the Pixhawk `TELEM2` `TX`/`RX`/`GND` pins to the complementary `RXD`/`TXD`/`Ground` pins on the RPi GPIO board:

| PX4 TELEM2 Pin | RPi GPIO Pin           |
| -------------- | ---------------------- |
| UART5_TX (2)   | RXD (GPIO 15 - pin 10) |
| UART5_RX (3)   | TXD (GPIO 14 - pin 8)  |
| GND (6)        | Ground (pin 6)         |

The diagram shows Pixhawk `TELEM2` port pins on the left and RPi GPIO board pins on the right.
The pins on the `TELEM2` port are normally numbered right-to-left as shown.

| `TELEM2`                                                                                                      | RPi GPIO                                                      |
| ------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------- |
| ![Pin numbering showing left-most pin is pin 1](../../assets/companion_computer/pixhawk_rpi/pins_numbers.png) | ![](../../assets/companion_computer/pixhawk_rpi/rpi_gpio.png) |

::: info
Almost all recent Pixhawk boards, such as the Pixhawk-6C, use the same connectors and pin numbers for corresponding ports, as defined in the Pixhawk Connector Standard.
You can check the specific board documentation to confirm the pin layout.

The standard `TELEM2` pin assignments are shown below.

| Pins      | Signal          | Voltage |
| --------- | --------------- | ------- |
| 1 (Red)   | VCC             | +5V     |
| 2 (Black) | UART5_TX (out)  | +3.3V   |
| 3 (Black) | UART5_RX (in)   | +3.3V   |
| 4 (Black) | UART5_CTS (in)  | +3.3V   |
| 5 (Black) | UART5_RTS (out) | +3.3V   |
| 6 (Black) | GND             | GND     |

:::

### TELEM1/Telemetry Radio

The Pixhawk `TELEM1` port is preconfigured for connecting to a GCS via MAVLink over a telemetry radio.

You can plug an [appropriate radio](../telemetry/index.md) into the Pixhawk `TELEM1` port and in most cases it should just work.
Generally the other radio needs to be connected to the ground station USB port.
If you have any issues, check the radio documentation.

### Power Supply

Pixhawk boards usually require a reliable 5V DC supply, which is commonly supplied from LiPO batteries via a [Power Module and/or Power Distribution board](../power_module/index.md) to a port labeled `POWER` (or similar).

The instructions for your flight controller will normally explain the recommended setup.
For example:

- [Holybro Pixhawk 6C > Voltage Ratings](../flight_controller/pixhawk6c.md#voltage-ratings)
- [Holybro Pixhawk 6C Wiring Quick Start > Power](../assembly/quick_start_pixhawk6c.md#power)

Pixhawk controllers can supply power to a _small_ number of low-power peripherals, such as GPS modules and low-range telemetry radios.
The RPi companion computer, servos, high power radios, and other peripherals require a separate power supply, which is usually from a battery elimination circuit (BEC) wired to the same or another battery.
Some power modules have a separate BEC included.

:::warning
Overloading your Pixhawk is a good way to destroy it.
:::

::: info
During PX4 setup and configuration the USB connection with your ground station laptop is sufficient to power the Pixhawk board, and your companion computer might be powered from a desktop charger.
:::

## PX4 Setup

These instructions work on PX4 v1.14 and later.

If you need to update the firmware then connect the Pixhawk to your laptop/desktop via the `USB` port and use QGroundControl to update the firmware as described [Firmware > Install Stable PX4](../config/firmware.md#install-stable-px4). 
If you want the latest developer version then update the firmware to the "main" as described in [Firmware > Installing PX4 Master, Beta or Custom Firmware](../config/firmware.md#installing-px4-main-beta-or-custom-firmware).

::: info
You can alternatively [setup a development environment](../dev_setup/dev_env.md), [build](../dev_setup/building_px4.md#building-for-nuttx) and [upload](../dev_setup/building_px4.md#uploading-firmware-flashing-the-board) the firmware manually.
:::

<!-- Keeping this line as record - this is only unexpected dependency:
```
sudo apt -y install stlink-tools
```
-->
<!-- Keeping this because we might need it for updating linux instructions
On Linux, the default name of a USB connection is `/dev/ttyACM0`:

```
sudo chmod a+rw /dev/ttyACM0
cd /PX4-Autopilot
make px4_fmu-v6c_default upload
```
-->

## Ubuntu Setup on RPi

The following steps show how to install and setup Ubuntu 22.04 on the RPi.
Note that ROS 2 versions target particular Ubuntu versions.
We're using Ubuntu 22.04 to match ROS 2 "Humble", so if you're working with ROS 2 "Foxy" you would instead install Ubuntu 20.04.

First install Ubuntu onto the RPi:

1. Prepare a Ubuntu 22.04 bootable Ubuntu Desktop SD card by following the official tutorial: [How to install Ubuntu Desktop on Raspberry Pi 4](https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4#1-overview)
1. Connect the mouse, keyboard, monitor and connect the RPi to a 5V Power Supply (external source/charger).
1. Insert the SD card into the RPi and turn on the RPi to boot from the SD card.
1. Follow the on-screen instructions to install Ubuntu.

Enter the following commands (in sequence) a terminal to configure Ubuntu for RPi:

1. Install `raspi-config`:

   ```sh
   sudo apt update
   sudo apt upgrade
   sudo apt-get install raspi-config
   ```

1. Open `raspi-config`:

   ```sh
   sudo raspi-config
   ```

1. Go to the **Interface Option** and then click **Serial Port**.

   - Select **No** to disable serial login shell.
   - Select **Yes** to enable the serial interface.
   - Click **Finish** and restart the RPi.

1. Open the firmware boot configuration file in the `nano` editor on RPi:

   ```sh
   sudo nano /boot/firmware/config.txt
   ```

1. Append the following text to the end of the file (after the last line):

   ```sh
   enable_uart=1
   dtoverlay=disable-bt
   ```

1. Then save the file and restart the RPi.

   - In `nano` you can save the file using the following sequence of keyboard shortcuts: **ctrl+x**, **ctrl+y**, **Enter**.

1. Check that the serial port is available.
   In this case we use the following terminal commands to list the serial devices:

   ```sh
   cd /
   ls /dev/ttyAMA0
   ```

   The result of the command should include the RX/TX connection `/dev/ttyAMA0` (note that this serial port is also available as `/dev/serial0`).

The RPi is now setup to work with RPi and communicate using the `/dev/ttyAMA0` serial port.
Note that we'll install more software in the following sections to work with MAVLink and ROS 2.

## MAVLink Communication

[MAVLink](https://mavlink.io/en/) is the default and stable communication interface for working with PX4.
MAVLink applications running on the companion computer can connect to the `/dev/ttyAMA0` serial port you just set up on the RPi and should automatically (by default) connect to `TELEM 2` on the Pixhawk.

PX4 recommends [MAVSDK](https://mavsdk.mavlink.io/main/en/index.html) for writing MAVLink companion computer applications, as it provides simple APIs for using many common MAVLink services in many different programming languages.
You can also write applications using the libraries provided by [MAVLink](https://mavlink.io/en/#mavlink-project-generatorslanguages), such as [Pymavlink](https://mavlink.io/en/mavgen_python/), but then you are more likely to have to provide your own implementations of some microservices.

For this tutorial we're not going to go into MAVLink control in any detail (it is well covered in the respective SDKs).
However we will install and use a simple developer MAVLink GCS called `mavproxy`.
This will allow us to verify the MAVLink connection, and therefore that our physical connection has been set up properly.
A very similar connection pattern would be used for MAVSDK and other MAVLink applications.

First check the Pixhawk `TELEM 2` configuration:

1. Connect the Pixhawk with the laptop using a USB cable.
1. Open QGroundControl (the vehicle should connect).
1. [Check/change the following parameters](../advanced_config/parameters.md) in QGroundControl:

   ```ini
   MAV_1_CONFIG = TELEM2
   UXRCE_DDS_CFG = 0 (Disabled)
   SER_TEL2_BAUD = 57600
   ```

   Note that the parameters may already be set appropriately.
   For information about how serial ports and MAVLink configuration work see [Serial Port Configuration](../peripherals/serial_configuration.md) and [MAVLink Peripherals](../peripherals/mavlink_peripherals.md).

Then install setup MAVProxy on the RPi using the following terminal commands:

1. Install MAVProxy:

   ```sh
   sudo apt install python3-pip
   sudo pip3 install mavproxy
   sudo apt remove modemmanager
   ```

1. Run MAVProxy, setting the port to connect to `/dev/ttyAMA0` and the baud rate to match the PX4:

   ```sh
   sudo mavproxy.py --master=/dev/serial0 --baudrate 57600
   ```

   ::: info
   Note that above we used `/dev/serial0`, but we could equally well have used `/dev/ttyAMA0`.
   If we were connecting via USB then we would instead set the port as `/dev/ttyACM0`:

   ```sh
   sudo chmod a+rw /dev/ttyACM0
   sudo mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
   ```

   :::

MAVProxy on RPi should now connect to the Pixhawk, via RX/TX pins.
You should be able to see this in the RPi terminal.

We have now verified that our connection is wired up properly.
In the next section we'll set up the both Pixhawk and RPi to use uXRCE-DDS and ROS2 instead of MAVLink.

## ROS 2 and uXRCE-DDS

The [ROS 2 Guide](../ros2/user_guide.md) and [uXRCE-DDS](../middleware/uxrce_dds.md) pages cover the options for setting up the uXRCE-DDS and ROS, focussing on ROS 2 "Foxy".
This tutorial uses ROS 2 "Humble" and covers the specific setup for working with RPi.
It is worth reading both!

### Pixhawk/PX4 Setup

Next we set up ROS 2 instead of MAVLink on `TELEM2`.
We do this by changing parameters in QGroundControl, which can be connected via USB, or using a telemetry radio connected to `TELEM1`.

The configuration steps are:

1. Connect the Pixhawk with the laptop using a USB cable and open QGroundControl (if not currently connected).
1. [Check/change the following parameters](../advanced_config/parameters.md) in QGroundControl:

   ```ini
   MAV_1_CONFIG = 0 (Disabled)
   UXRCE_DDS_CFG = 102 (TELEM2)
   SER_TEL2_BAUD = 921600
   ```

   [MAV_1_CONFIG=0](../advanced_config/parameter_reference.md#MAV_1_CONFIG) and [UXRCE_DDS_CFG=102](../advanced_config/parameter_reference.md#UXRCE_DDS_CFG) disable MAVLink on TELEM2 and enable the uXRCE-DDS client on TELEM2, respectively.
   The `SER_TEL2_BAUD` rate sets the comms link data rate.  
   You could similarly configure a connection to `TELEM1` using either `MAV_1_CONFIG` or `MAV_0_CONFIG`.

   ::: info
   You will need to reboot the flight controller to apply any changes to these parameters.
   :::

1. Check that the [uxrce_dds_client](../modules/modules_system.md#uxrce-dds-client) module is now running.
   YOu can do this by running the following command in the QGroundControl [MAVLink Console](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_console.html):

   ```sh
   uxrce_dds_client status
   ```

::: info
If the client module is not running you can start it manually in the MAVLink console:

```sh
uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600
```

Note that `/dev/ttyS3` is the PX4 port for `TELEM2` on the [Holybro Pixhawk 6c](../flight_controller/pixhawk6c.md#serial-port-mapping).
For other flight controllers check the serial port mapping section in their overview page.
:::

### ROS Setup on RPi

The steps to setup ROS 2 and the Micro XRCE-DDS Agent on the RPi are:

1. Install ROS 2 Humble by following the [official tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
2. Install the git using the RPi terminal:

   ```sh
   sudo apt install git
   ```

3. Install the uXRCE_DDS agent:

   ```sh
   git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
   cd Micro-XRCE-DDS-Agent
   mkdir build
   cd build
   cmake ..
   make
   sudo make install
   sudo ldconfig /usr/local/lib/
   ```

   See [uXRCE-DDS > Micro XRCE-DDS Agent Installation](../middleware/uxrce_dds.md#micro-xrce-dds-agent-installation) for alternative ways of installing the agent.

4. Start the agent in the RPi terminal:

   ```sh
   sudo MicroXRCEAgent serial --dev /dev/serial0 -b 921600
   ```

   Note how we use the serial port set up earlier and the same baud rate as for PX4.

Now that both the agent and client are running, you should see activity on both the MAVLink console and the RPi terminal.
You can view the available topics using the following command on the RPi:

```sh
source /opt/ros/humble/setup.bash
ros2 topic list
```

That's it.
Once you have the connection working, see the [ROS 2 Guide](../ros2/user_guide.md) for more information about working with PX4 and ROS 2.
