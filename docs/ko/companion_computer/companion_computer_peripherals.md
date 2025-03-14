# 보조 컴퓨터 주변 기기

This section contains information about companion computer peripherals.
These include both components that might be connected to a companion computer (potentially triggered/accessed by PX4), and for connecting the computer to the flight controller.

## Companion/Pixhawk Communication

This section lists devices that may be used for the physical serial/data connection between a companion computer and a flight controller.

:::info
PX4 configuration for communicating with a companion computer over MAVLink via TELEM2 is covered in [MAVLink (OSD / Telemetry)](../peripherals/mavlink_peripherals.md#telem2).
Other relevant topics/sections include: [Companion Computers](../companion_computer/index.md), [Robotics](../robotics/index.md) and [uXRCE-DDS (PX4-ROS 2/DDS Bridge)](../middleware/uxrce_dds.md).
:::

### FTDI Devices

The FTDI USB adapters are the most common way of communicating between companion computer and Pixhawk.
They are usually plug and play as long as the IO of the adapter is set to 3.3V.
In order to utilize the full capability/reliability of the serial link offered on the Pixhawk hardware, flow control is recommended.

A few "turnkey" options are listed below:

| 장치                                                                                                                                                                                                                             | 3.3v IO (Default) | Flow Control | Tx/Rx LEDs | JST-GH |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------- | ------------ | ---------- | ------ |
| [mRo USB FTDI Serial to JST-GH (Basic)][mro_usb_ftdi_serial_to_jst_gh]                                                                                                                                                         | Capable                                              | Capable      | No         | Yes    |
| [SparkFun FTDI Basic Breakout][sparkfun_ftdi__breakout] | Yes                                                  | No           | Yes        | No     |

<!-- Reference links for above table -->

[mro_usb_ftdi_serial_to_jst_gh]: https://store.mrobotics.io/USB-FTDI-Serial-to-JST-GH-p/mro-ftdi-jstgh01-mr.htm
[sparkfun_ftdi basic_breakout]: https://www.sparkfun.com/products/9873

You can also use an off-the-shelf FTDI cable [like this one](https://www.sparkfun.com/products/9717) and connect it to flight controller using the appropriate header adaptor
(JST-GH connectors are specified in the Pixhawk standard, but you should confirm the connectors for your flight controller).

### Logic Level Shifters

On occasion a companion computer may expose hardware level IO that is often run at 1.8v or 5v, while the Pixhawk hardware operates at 3.3v IO.
In order to resolve this, a level shifter can be implemented to safely convert the transmitting/receiving signal voltage.

Options include:

- [SparkFun Logic Level Converter - Bi-Directional](https://www.sparkfun.com/products/12009)
- [4-channel I2C-safe Bi-directional Logic Level Converter - BSS138](https://www.adafruit.com/product/757)

## Cameras

Cameras are used image and video capture, and more generally to provide data for [computer vision](../computer_vision/index.md) applications (in this case the "cameras" may only provide processed data, not raw images).

### Stereo Cameras

Stereo cameras are typically used for depth perception, path planning and SLAM.
They are in no way guaranteed to be plug and play with your companion computer.

Popular stereo cameras include:

- [Intel® RealSense™ Depth Camera D435](https://www.intelrealsense.com/depth-camera-d435/)
- [Intel® RealSense™ Depth Camera D415](https://www.intelrealsense.com/depth-camera-d415/)
- [DUO MLX](https://duo3d.com/product/duo-minilx-lv1)

### VIO Cameras/Sensors

The following sensors can be used for [Visual Inertial Odometry (VIO)](../computer_vision/visual_inertial_odometry.md):

- [T265 Realsense Tracking Camera](../peripherals/camera_t265_vio.md)

## Data Telephony (LTE)

An LTE USB module can be attached to a companion computer and used to route MAVLink traffic between the flight controller and the Internet.

There is no "standard method" for a ground station and companion to connect over the Internet.
Generally you can't connect them directly because neither of them will have a public/static IP on the Internet.

:::info
Typically your router (or the mobile network) has a public IP address, and your GCS computer/vehicle are on a _local_ network.
The router uses network address translation (NAT) to map _outgoing_ requests from your local network to the Internet, and can use the map to route the _responses_ back to requesting system.
However NAT has no way to know where to direct the traffic from an arbitrary external system, so there is no way to _initiate_ a connection to a GCS or vehicle running in the local network.
:::

A common approach is to set up a virtual private network between the companion and GCS computer (i.e. install a VPN system like [zerotier](https://www.zerotier.com/) on both computers).
The companion then uses [mavlink-router](https://github.com/intel/mavlink-router) to route traffic between the serial interface (flight controller) and GCS computer on the VPN network.

This method has the benefit that the GCS computer address can be static within the VPN, so the configuration of the _mavlink router_ does not need to change over time.
In addition, the communication link is secure because all VPN traffic is encrypted (MAVLink 2 itself does not support encryption).

:::info
You can also choose to route to the VPN broadcast address (i.e. `x.x.x.255:14550`, where 'x' depends on the VPN system).
This approach means that you do not need to know the IP address of the GCS computer, but may result in more traffic than desired (since packets are broadcast to every computer on the VPN network).
:::

Some USB modules that are known to work include:

- [Huawei E8372](https://consumer.huawei.com/en/mobile-broadband/e8372/) and [Huawei E3372](https://consumer.huawei.com/en/mobile-broadband/e3372/)
  - The _E8372_ includes WiFi which you can use to configure the SIM while it is plugged into the companion (making the development workflow a little easier). The _E3372_ lacks WiFi, so you have to configure it by plugging the stick into a laptop.
