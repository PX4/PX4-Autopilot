# MAVLink Peripherals (GCS/OSD/Gimbal/Camera/Companion)

Ground Control Stations (GCS), [MAVLink On-Screen Displays (OSD)](../peripherals/osd.md#mavlink-osd), MAVLink [Cameras](../camera/mavlink_v2_camera.md) and [Gimbals](../advanced/gimbal_control.md), [Remote IDs](../peripherals/remote_id.md), Companion Computers, [ADS-B receivers](../peripherals/adsb_flarm.md), and other MAVLink peripherals interact with PX4 using separate MAVLink streams, sent via different serial ports.

Set `SER_<tag>_PROTO` to MAVLink on the UART (see [Serial Port Configuration](../peripherals/serial_configuration.md)).
UARTs are numbered 0, 1, 2 in [port order](../peripherals/serial_configuration.md#configuration-parameters). Ethernet ([MAV_ETH_EN](../advanced_config/parameter_reference.md#MAV_ETH_EN)) takes the next free instance.
Then set `MAV_X_MODE`, `MAV_X_RATE`, `MAV_X_FORWARD` for that instance.

The most relevant parameters are described below (the full set are listed in the [Parameter Reference > MAVLink](../advanced_config/parameter_reference.md#mavlink)).

## MAVLink Instances

In order to assign a particular peripheral to a serial port we use the concept of a _MAVLink instance_.

Each MAVLink instance is the configuration for one UART (or Ethernet) MAVLink stream.
UARTs with `SER_*_PROTO` = MAVLink are numbered 0, 1, 2 in [port order](../peripherals/serial_configuration.md#configuration-parameters). Ethernet uses the next free instance.

- <a id="MAV_X_MODE"></a>[MAV_X_MODE](../advanced_config/parameter_reference.md#MAV_0_MODE) — [MAVLink profile](../mavlink/mavlink_profiles.md) (Normal, Onboard, OSD, …).
- <a id="MAV_X_RATE"></a>[MAV_X_RATE](../advanced_config/parameter_reference.md#MAV_0_RATE) — max data rate (bytes/second). 0 is half the theoretical baud.
- <a id="MAV_X_FORWARD"></a>[MAV_X_FORWARD](../advanced_config/parameter_reference.md#MAV_0_FORWARD) — forward incoming packets to other MAVLink instances.

Set the UART baud with `SER_<tag>_BAUD`.

:::tip
You will need to reboot PX4 to make the parameter available (i.e. in QGroundControl).
:::

The parameter used will depend on the [assigned serial port](../advanced_config/parameter_reference.md#serial) - for example: `SER_GPS1_BAUD`, `SER_TEL2_BAUD`, etc.
The value you use will depend on the type of connection and the capabilities of the connected MAVLink peripheral.

## Default MAVLink Ports {#default_ports}

### TELEM1

The `TELEM 1` port is almost always configured by default for the GCS telemetry stream ("Normal").

Default mapping of MAVLink instance 0:

- [SER_TEL1_PROTO](../advanced_config/parameter_reference.md#SER_TEL1_PROTO) = MAVLink
- [MAV_0_MODE](../advanced_config/parameter_reference.md#MAV_0_MODE) = `Normal`
- [MAV_0_RATE](../advanced_config/parameter_reference.md#MAV_0_RATE) = `1200` Bytes/s
- [MAV_0_FORWARD](../advanced_config/parameter_reference.md#MAV_0_FORWARD) = `True`
- [SER_TEL1_BAUD](../advanced_config/parameter_reference.md#SER_TEL1_BAUD) = `57600`

### TELEM2

`TELEM 2` is disabled by default. For a companion computer set [SER_TEL2_PROTO](../advanced_config/parameter_reference.md#SER_TEL2_PROTO) = MAVLink (instance 1 if TELEM 1 is also MAVLink):

- [MAV_1_MODE](../advanced_config/parameter_reference.md#MAV_1_MODE) = `Onboard`
- [MAV_1_RATE](../advanced_config/parameter_reference.md#MAV_1_RATE) = `0` (half maximum)
- [MAV_1_FORWARD](../advanced_config/parameter_reference.md#MAV_1_FORWARD) = `Disabled`
- [SER_TEL2_BAUD](../advanced_config/parameter_reference.md#SER_TEL2_BAUD) = `921600`

### ETHERNET

Pixhawk 5x devices (and later) that have an Ethernet port, configure it by default to connect to a GCS:

Enable with [MAV_ETH_EN](../advanced_config/parameter_reference.md#MAV_ETH_EN). With TELEM1 as the only UART MAVLink port, ethernet is instance 1 (`MAV_1_UDP_PRT` = 14550).

- [MAV_1_BROADCAST](../advanced_config/parameter_reference.md#MAV_1_BROADCAST) = `1`
- [MAV_1_MODE](../advanced_config/parameter_reference.md#MAV_1_MODE) = `0` (normal/GCS)
- [MAV_1_RADIO_CTL](../advanced_config/parameter_reference.md#MAV_1_RADIO_CTL) = `0`
- [MAV_1_RATE](../advanced_config/parameter_reference.md#MAV_1_RATE) = `100000`
- [MAV_1_REMOTE_PRT](../advanced_config/parameter_reference.md#MAV_1_REMOTE_PRT) = `14550` (GCS)
- [MAV_1_UDP_PRT](../advanced_config/parameter_reference.md#MAV_1_UDP_PRT) = `14550` (GCS)

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
