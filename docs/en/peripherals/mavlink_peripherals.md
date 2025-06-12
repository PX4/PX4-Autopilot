# MAVLink Peripherals (GCS/OSD/Gimbal/Camera/Companion)

Ground Control Stations (GCS), On-Screen Displays (OSD), MAVLink Cameras & Gimbals, Remote IDs, Companion Computers, ADS-B receivers, and other MAVLink peripherals interact with PX4 using separate MAVLink streams, sent via different serial ports.

In order to configure that a particular serial port is used for MAVLink traffic with a particular peripheral, we use [Serial Port Configuration](../peripherals/serial_configuration.md), assigning one of the abstract "MAVLink instance" configuration parameters to the desired port.
We then set other properties of the MAVLink channel using the parameters associated with our selected MAVLink instance, so that they match the requirements of our particular peripheral.

The most relevant parameters are described below (the full set are listed in the [Parameter Reference > MAVLink](../advanced_config/parameter_reference.md#mavlink)).

## MAVLink Instances

In order to assign a particular peripheral to a serial port we use the concept of a _MAVLink instance_.

Each MAVLink instance represents a particular MAVLink configuration that you can apply to a particular port.
At time of writing three MAVLink _instances_ are defined, each represented by a parameter [MAV_X_CONFIG](#MAV_X_CONFIG), where X is 0, 1, 2.

Each instance has associated parameters that you can use to define the properties of the instance on that port, such as the set of streamed messages (see [MAV_X_MODE](#MAV_X_MODE) below), data rate ([MAV_X_RATE](#MAV_X_RATE)), whether incoming traffic is forwarded to other MAVLink instances ([MAV_X_FORWARD](#MAV_X_FORWARD)), and so on.

::: info
MAVLink instances are an abstract concept for a particular MAVLink configuration.
The number in the name means nothing; you can assign any instance to any port.
:::

The parameters for each instance are:

- <a id="MAV_X_CONFIG"></a>[MAV_X_CONFIG](../advanced_config/parameter_reference.md#MAV_0_CONFIG) - Set the serial port (UART) for this instance "X", where X is 0, 1, 2.
  It can be any unused port, e.g.: `TELEM2`, `TELEM3`, `GPS2` etc.
  For more information see [Serial Port Configuration](../peripherals/serial_configuration.md).
- <a id="MAV_X_MODE"></a>[MAV_X_MODE](../advanced_config/parameter_reference.md#MAV_0_MODE) - Specify the telemetry mode/target (the set of messages to stream for the current instance and their rate).
  The default values are:

  - _Normal_: Standard set of messages for a GCS.
  - _Custom_ or _Magic_: Nothing (in the default PX4 implementation).
    Modes may be used for testing when developing a new mode.
  - _Onboard_: Standard set of messages for a companion computer.
  - _OSD_: Standard set of messages for an OSD system.
  - _Config_: Standard set of messages and rate configuration for a fast link (e.g. USB).
  - _Minimal_: Minimal set of messages for use with a GCS connected on a high latency link.
  - _External Vision_: Messages for offboard vision systems.
  - _Gimbal_: Messages for a gimbal. Note this also enables [message forwarding](#MAV_X_FORWARD)
  - _Onboard Low Bandwidth_: Standard set of messages for a companion computer connected on a lower speed link.
  - _uAvionix_: Messages for a uAvionix ADS-B beacon.

  ::: info
  If you need to find the specific set of message for each mode search for `MAVLINK_MODE_` in [/src/modules/mavlink/mavlink_main.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_main.cpp).
  :::

  :::tip
  The mode defines the _default_ messages and rates.
  A connected MAVLink system can still request the streams/rates that it wants using [MAV_CMD_SET_MESSAGE_INTERVAL](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL).
  :::

- <a id="MAV_X_RATE"></a>[MAV_X_RATE](../advanced_config/parameter_reference.md#MAV_0_MODE) - Set the maximum _data rate_ for this instance (bytes/second).
  - This is the combined rate for all streams of individual message (the rates for individual messages are reduced if the total rate exceeds this value).
  - The default setting will generally be acceptable, but might be reduced if the telemetry link becomes saturated and too many messages are being dropped.
  - A value of 0 sets the data rate to half the theoretical value.
- <a id="MAV_X_FORWARD"></a>[MAV_X_FORWARD](../advanced_config/parameter_reference.md#MAV_0_FORWARD) - Enable forwarding of MAVLink packets received by the current instance onto other interfaces.
  This might be used, for example, to transfer messages between a GCS and a companion computer so that the GCS can talk to a MAVLink enabled camera connected to the companion computer.

Next you need to set the baud rate for the serial port you assigned above (in `MAV_X_CONFIG`).

:::tip
You will need to reboot PX4 to make the parameter available (i.e. in QGroundControl).
:::

The parameter used will depend on the [assigned serial port](../advanced_config/parameter_reference.md#serial) - for example: `SER_GPS1_BAUD`, `SER_TEL2_BAUD`, etc.
The value you use will depend on the type of connection and the capabilities of the connected MAVLink peripheral.

## Default MAVLink Ports {#default_ports}

### TELEM1

The `TELEM 1` port is almost always configured by default for the GCS telemetry stream ("Normal").

To support this there is a [default serial port mapping](../peripherals/serial_configuration.md#default_port_mapping) of MAVLink instance 0 as shown below:

- [MAV_0_CONFIG](../advanced_config/parameter_reference.md#MAV_0_CONFIG) = `TELEM 1`
- [MAV_0_MODE](../advanced_config/parameter_reference.md#MAV_0_MODE) = `Normal`
- [MAV_0_RATE](../advanced_config/parameter_reference.md#MAV_0_RATE)= `1200` Bytes/s
- [MAV_0_FORWARD](../advanced_config/parameter_reference.md#MAV_0_FORWARD) = `True`
- [SER_TEL1_BAUD](../advanced_config/parameter_reference.md#SER_TEL1_BAUD) = `57600`

### TELEM2

The `TELEM 2` port usually configured by default for a companion computer telemetry stream ("Onboard").

To support this there is a [default serial port mapping](../peripherals/serial_configuration.md#default_port_mapping) of MAVLink instance 0 as shown below:

- [MAV_1_CONFIG](../advanced_config/parameter_reference.md#MAV_0_CONFIG) = `TELEM 2`
- [MAV_1_MODE](../advanced_config/parameter_reference.md#MAV_0_MODE) = `Onboard`
- [MAV_1_RATE](../advanced_config/parameter_reference.md#MAV_0_RATE)= `0` (Half maximum)
- [MAV_1_FORWARD](../advanced_config/parameter_reference.md#MAV_0_FORWARD) = `Disabled`
- [SER_TEL2_BAUD](../advanced_config/parameter_reference.md#SER_TEL2_BAUD) = `921600`

### ETHERNET

Pixhawk 5x devices (and later) that have an Ethernet port, configure it by default to connect to a GCS:

On this hardware, there is a [default serial port mapping](../peripherals/serial_configuration.md#default_port_mapping) of MAVLink instance 2 as shown below:

- [MAV_2_CONFIG](../advanced_config/parameter_reference.md#MAV_2_CONFIG) = `Ethernet` (1000)
- [MAV_2_BROADCAST](../advanced_config/parameter_reference.md#MAV_2_BROADCAST) = `1`
- [MAV_2_MODE](../advanced_config/parameter_reference.md#MAV_2_MODE) = `0` (normal/GCS)
- [MAV_2_RADIO_CTL](../advanced_config/parameter_reference.md#MAV_2_RADIO_CTL) = `0`
- [MAV_2_RATE](../advanced_config/parameter_reference.md#MAV_2_RATE) = `100000`
- [MAV_2_REMOTE_PRT](../advanced_config/parameter_reference.md#MAV_2_REMOTE_PRT)= `14550` (GCS)
- [MAV_2_UDP_PRT](../advanced_config/parameter_reference.md#MAV_2_UDP_PRT) = `14550` (GCS)

For more information see: [PX4 Ethernet Setup](../advanced_config/ethernet_setup.md)

## Device Specific Setup

Links to setup instructions for specific MAVLink components:

- [MAVLink Cameras (Camera Protocol v2) > PX4 Configuration](../camera/mavlink_v2_camera.md#px4-configuration)
- [Gimbal Configuration > MAVLink Gimbal (MNT_MODE_OUT=MAVLINK)](../advanced/gimbal_control.md#mavlink-gimbal-mnt-mode-out-mavlink)

## See Also

- [Serial Port Configuration](../peripherals/serial_configuration.md)
- [PX4 Ethernet Setup > PX4 MAVLink Serial Port Configuration](../advanced_config/ethernet_setup.md#px4-mavlink-serial-port-configuration)
- [Serial Port Mapping](../hardware/serial_port_mapping.md)

