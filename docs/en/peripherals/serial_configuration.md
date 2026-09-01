# Serial Port Configuration

PX4 defines [default functions](#default-serial-port-configuration) for many flight controller ports, which is why you can plug a GPS module into the port labelled `GPS 1`, an RC receiver into `RC IN`, or a telemetry module into `TELEM 1`, and generally they will just work.

The functions assigned to ports are fully configurable using appropriate parameters (in most cases).
You can assign any unused port to any function, or reassign a port to use it for something else.

The configuration makes it easy to (for example):

- Run MAVLink on a different port, change the streamed messages, or switch a TELEM port to use ROS 2/XRCE-DDS.
- Change the baud rate on a port or set the UDP port
- Setup dual GPS.
- Enable sensors that run on a serial port, such as some [distance sensors](../sensor/rangefinders.md).

::: info

- Some ports cannot be configured because they are used for a very specific purpose such as the system console.
- The mapping of specific devices to port names on the flight controller is explained in [Serial Port Mapping](../hardware/serial_port_mapping.md).
  :::

## Configuration Parameters

Each UART has two parameters, named from the port tag (`TEL1`, `GPS1`, `RC`, …):

- [SER_TEL1_PROT](../advanced_config/parameter_reference.md#SER_TEL1_PROT) (and `SER_<tag>_PROT`) — protocol started on that port.
- [SER_TEL1_BAUD](../advanced_config/parameter_reference.md#SER_TEL1_BAUD) (and `SER_<tag>_BAUD`) — baud rate.

_QGroundControl_ only lists protocols compiled into the connected firmware.
A protocol already running on another port cannot be started a second time (except MAVLink, up to three UART instances, and GPS/Septentrio, up to two ports).

MAVLink still has per-instance settings ([MAV_0_MODE](../advanced_config/parameter_reference.md#MAV_0_MODE), [MAV_0_RATE](../advanced_config/parameter_reference.md#MAV_0_RATE), [MAV_0_FORWARD](../advanced_config/parameter_reference.md#MAV_0_FORWARD), …).
Instance 0 is the first UART whose `SER_*_PROT` is MAVLink (by port-tag index), instance 1 the next, instance 2 the third.
Ethernet MAVLink is [MAV_ETH_EN](../advanced_config/parameter_reference.md#MAV_ETH_EN) and uses instance 2.

## How to Configure a Port

1. Set `SER_<tag>_PROT` for the silkscreen port (`SER_TEL2_PROT` = GPS, `SER_RC_PROT` = CRSF, …).
1. Reboot.
1. Set `SER_<tag>_BAUD` if the driver does not autodetect.
1. Set module-specific parameters (MAVLink mode/rate, GPS protocol, CRSF telemetry, …).

[GPS/Compass > Secondary GPS](../gps_compass/index.md#dual_gps) is the dual-GPS example: set a second port's `SER_*_PROT` to GPS.
[MAVLink Peripherals](../peripherals/mavlink_peripherals.md) covers MAVLink modes.
[PX4 Ethernet Setup](../advanced_config/ethernet_setup.md#px4-mavlink-serial-port-configuration) covers UDP MAVLink.

## Deconflicting Ports

A port runs at most one protocol.
Startup skips extra copies of a singleton driver and extra MAVLink instances beyond the limit.

<a id="default_port_mapping"></a>

## Default Serial Port Configuration

:::tip
These port mappings can be disabled by setting the associated configuration parameter to _Disabled_.
:::

The following ports are commonly mapped to specific functions on all boards:

- `GPS 1`: [SER_GPS1_PROT](../advanced_config/parameter_reference.md#SER_GPS1_PROT) = GPS, [SER_GPS1_BAUD](../advanced_config/parameter_reference.md#SER_GPS1_BAUD) = Auto.
- `RC IN` on an FMU UART: [SER_RC_PROT](../advanced_config/parameter_reference.md#SER_RC_PROT) = SBUS.
  On IO boards the RC connector is scanned by the IO firmware; it is not a `SER_*` slot.
- `TELEM 1`: [SER_TEL1_PROT](../advanced_config/parameter_reference.md#SER_TEL1_PROT) = MAVLink (instance 0, Normal).
- `TELEM 2`: disabled (`SER_TEL2_PROT` = Disabled). Set it to MAVLink for a companion link (instance 1 defaults to Onboard).
- Ethernet: [MAV_ETH_EN](../advanced_config/parameter_reference.md#MAV_ETH_EN) (MAVLink instance 2). See [PX4 Ethernet Setup](../advanced_config/ethernet_setup.md#px4-mavlink-serial-port-configuration).

- `USB-C` (the USB-C port normally used for connecting to QGroundControl)

  This is configured by default as a MAVLink port the onboard profile (for companion computers).
  The configuration for MAVLink is unique to this port (it does not use `SER_*_PROT`).
  - [SYS_USB_AUTO](../advanced_config/parameter_reference.md#SYS_USB_AUTO) sets whether the port is set to no particular protocol, autodetects the protocol, or sets the comms link to MAVLink.
  - [USB_MAV_MODE](../advanced_config/parameter_reference.md#USB_MAV_MODE) sets the MAVLink profile that is used if MAVLink is set or detected.

Other ports generally have no assigned functions by default (are disabled).

## Troubleshooting

<a id="parameter_not_in_firmware"></a>

### Configuration Parameter Missing from _QGroundControl_

_QGroundControl_ only displays the parameters for services/drivers that are present in firmware.
If a parameter is missing, then you may need to add it in firmware.

::: info
PX4 firmware includes most drivers by default on [Pixhawk-series](../flight_controller/pixhawk_series.md) boards.
Flash-limited boards may comment out/omit the driver (at time of writing this only affects boards based on FMUv2).
:::

You can include the missing driver in firmware by enabling the driver in the **default.px4board** config file that corresponds to the [board](https://github.com/PX4/PX4-Autopilot/tree/main/boards/px4) you want to build for.
For example, to enable the SRF02 driver, you would a the following line to the px4board.

```
CONFIG_DRIVERS_DISTANCE_SENSOR_SRF02=y
```

An easier method would be using boardconfig which launches a GUI where you can easily search, disable and enable modules.
To launch boardconfig type:

```
make <vendor>_<board>_<label> boardconfig
```

You will then need to build the firmware for your platform, as described in [Building PX4 Software](../dev_setup/building_px4.md).

## Further Information

- [MAVLink Peripherals (OSD/GCS/Companion Computers/etc.)](../peripherals/mavlink_peripherals.md)
- [PX4 Ethernet Setup > PX4 MAVLink Serial Port Configuration](../advanced_config/ethernet_setup.md#px4-mavlink-serial-port-configuration)
- [Serial Port Mapping](../hardware/serial_port_mapping.md)
