# MAVLink Peripherals (GCS/OSD/Gimbal/Camera/Companion)

Ground Control Stations (GCS), [MAVLink On-Screen Displays (OSD)](../peripherals/osd.md#mavlink-osd), MAVLink [Cameras](../camera/mavlink_v2_camera.md) and [Gimbals](../advanced/gimbal_control.md), [Remote IDs](../peripherals/remote_id.md), Companion Computers, [ADS-B receivers](../peripherals/adsb_flarm.md), and other MAVLink peripherals interact with PX4 using separate MAVLink streams, sent via different serial ports.

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
- <a id="MAV_X_MODE"></a>[MAV_X_MODE](../advanced_config/parameter_reference.md#MAV_0_MODE) - Specify the [MAVLink profile](../mavlink/mavlink_profiles.md) for the instance, such as _Normal_ or _OSD_.
  Profiles define a particular set of streamed messages and their rates — you should choose a profile that is appropriate for your channel and the peripheral.
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
- [MAV_2_REMOTE_IP](../advanced_config/parameter_reference.md#MAV_2_REMOTE_IP) = `0` (auto-discover partner, see below)
- [MAV_2_REMOTE_PRT](../advanced_config/parameter_reference.md#MAV_2_REMOTE_PRT)= `14550` (GCS)
- [MAV_2_UDP_PRT](../advanced_config/parameter_reference.md#MAV_2_UDP_PRT) = `14550` (GCS)

By default PX4 learns its MAVLink partner's address from the first UDP packet it receives on [MAV_X_UDP_PRT](../advanced_config/parameter_reference.md#MAV_2_UDP_PRT), and sends replies to that address on [MAV_X_REMOTE_PRT](../advanced_config/parameter_reference.md#MAV_2_REMOTE_PRT).
This works well when the peer connects to PX4 first, but not if PX4 needs to initiate the connection to a peer with a known, fixed address (for example a companion computer that never sends the first packet).

For that case, set [MAV_X_REMOTE_IP](../advanced_config/parameter_reference.md#MAV_2_REMOTE_IP) to the partner's address, encoded as a 32-bit integer with the first octet in the most significant byte (for example `192.168.1.2` is `-1062731518`).
This pins the instance's target address for the lifetime of the MAVLink task and disables auto-discovery.
Set it back to `0` (the default) to restore auto-discovery.

::: tip
The startup script applies this by passing the parameter to the `mavlink start` command's `-t` option as `-t p:MAV_X_REMOTE_IP`.
If you start a MAVLink instance manually from the console instead, `-t` accepts either form: a literal address (`-t 192.168.1.2`) or a reference to the parameter (`-t p:MAV_2_REMOTE_IP`).
See the [mavlink module reference](../modules/modules_communication.md#mavlink_usage) for the full command usage.
:::

For more information see: [PX4 Ethernet Setup](../advanced_config/ethernet_setup.md)

## Device Specific Setup

Links to setup instructions for specific MAVLink components:

- [MAVLink Cameras (Camera Protocol v2) > PX4 Configuration](../camera/mavlink_v2_camera.md#px4-configuration)
- [Gimbal Configuration > MAVLink Gimbal (MNT_MODE_OUT=MAVLINK)](../advanced/gimbal_control.md#mavlink-gimbal-mnt-mode-out-mavlink)

## See Also

- [MAVLink Profiles](../mavlink/mavlink_profiles.md)
- [Serial Port Configuration](../peripherals/serial_configuration.md)
- [PX4 Ethernet Setup > PX4 MAVLink Serial Port Configuration](../advanced_config/ethernet_setup.md#px4-mavlink-serial-port-configuration)
- [Serial Port Mapping](../hardware/serial_port_mapping.md)
