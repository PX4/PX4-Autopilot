# 직렬 포트 설정

PX4 defines [default functions](#default-serial-port-configuration) for many flight controller ports, which is why you can plug a GPS module into the port labelled `GPS 1`, an RC receiver into `RC IN`, or a telemetry module into `TELEM 1`, and generally they will just work.

The functions assigned to ports are fully configurable using appropriate parameters (in most cases).
You can assign any unused port to any function, or reassign a port to use it for something else.

설정을 통하여 아래의 작업들이 용이해집니다.(예 :)

- Run MAVLink on a different port, change the streamed messages, or switch a TELEM port to use ROS 2/XRCE-DDS.
- Change the baud rate on a port or set the UDP port
- Setup dual GPS.
- Enable sensors that run on a serial port, such as some [distance sensors](../sensor/rangefinders.md).

::: info

- Some ports cannot be configured because they are used for a very specific purpose such as the system console.
- The mapping of specific devices to port names on the flight controller is explained in [Serial Port Mapping](../hardware/serial_port_mapping.md).

:::

## Configuration Parameters

The serial port configuration parameters allow you to assign a particular function or support for particular hardware to a particular port.
These parameters follow the naming pattern `*_CONFIG` or `*_CFG`

:::info
_QGroundControl_ only displays the parameters for services/drivers that are present in firmware.
:::

At time of writing the current set is:

- GNSS configuration:
  - GPS Module: [GPS_1_CONFIG](../advanced_config/parameter_reference.md#GPS_1_CONFIG), [GPS_2_CONFIG](../advanced_config/parameter_reference.md#GPS_2_CONFIG) (for most GPS/GNSS systems)
  - [Septentrio GNSS Modules](../gps_compass/septentrio.md): [SEP_PORT1_CFG](../advanced_config/parameter_reference.md#SEP_PORT1_CFG), [SEP_PORT2_CFG](../advanced_config/parameter_reference.md#SEP_PORT2_CFG)
- [Iridium Satellite radio](../advanced_features/satcom_roadblock.md): [ISBD_CONFIG](../advanced_config/parameter_reference.md#ISBD_CONFIG)
- [MAVLink Ports](../peripherals/mavlink_peripherals.md): [MAV_0_CONFIG](../advanced_config/parameter_reference.md#MAV_0_CONFIG), [MAV_1_CONFIG](../advanced_config/parameter_reference.md#MAV_1_CONFIG), [MAV_2_CONFIG](../advanced_config/parameter_reference.md#MAV_2_CONFIG)
- VOXL ESC: [VOXL_ESC_CONFIG](../advanced_config/parameter_reference.md#VOXL_ESC_CONFIG)
- MSP OSD: [MSP_OSD_CONFIG](../advanced_config/parameter_reference.md#MSP_OSD_CONFIG)
- RC Port: [RC_PORT_CONFIG](../advanced_config/parameter_reference.md#RC_PORT_CONFIG)
- [FrSky Telemetry](../peripherals/frsky_telemetry.md): [TEL_FRSKY_CONFIG](../advanced_config/parameter_reference.md#TEL_FRSKY_CONFIG)
- HoTT Telemetry: [TEL_HOTT_CONFIG](../advanced_config/parameter_reference.md#TEL_HOTT_CONFIG)
- [uXRCE-DDS](../middleware/uxrce_dds.md) port: [UXRCE_DDS_CFG](../advanced_config/parameter_reference.md#UXRCE_DDS_CFG),
- Sensors (optical flow, distance sensors): [SENS_CM8JL65_CFG](../advanced_config/parameter_reference.md#SENS_CM8JL65_CFG), [SENS_LEDDAR1_CFG](../advanced_config/parameter_reference.md#SENS_LEDDAR1_CFG), [SENS_SF0X_CFG](../advanced_config/parameter_reference.md#SENS_SF0X_CFG), [SENS_TFLOW_CFG](../advanced_config/parameter_reference.md#SENS_TFLOW_CFG), [SENS_TFMINI_CFG](../advanced_config/parameter_reference.md#SENS_TFMINI_CFG), [SENS_ULAND_CFG](../advanced_config/parameter_reference.md#SENS_ULAND_CFG), [SENS_VN_CFG](../advanced_config/parameter_reference.md#SENS_VN_CFG),
- CRSF RC Input Driver: [RC_CRSF_PRT_CFG](../advanced_config/parameter_reference.md#RC_CRSF_PRT_CFG)
- Sagetech MXS: [MXS_SER_CFG](../advanced_config/parameter_reference.md#MXS_SER_CFG)
- Ultrawideband position sensor: [UWB_PORT_CFG](../advanced_config/parameter_reference.md#UWB_PORT_CFG)
- DShot driver: [DSHOT_TEL_CFG](../advanced_config/parameter_reference.md#DSHOT_TEL_CFG)

Some functions/features may define additional configuration parameters, which will follow a similar naming pattern to the port configuration prefix.
For example, `MAV_0_CONFIG` enables MAVLink on a particular port, but you may also need to set [MAV_0_FLOW_CTRL](../advanced_config/parameter_reference.md#MAV_0_FLOW_CTRL), [MAV_0_FORWARD](../advanced_config/parameter_reference.md#MAV_0_FLOW_CTRL), [MAV_0_MODE](../advanced_config/parameter_reference.md#MAV_0_MODE) and so on.

## 포트를 설정 방법

모든 직렬 드라이버와 포트는 동일한 방식으로 설정합니다.

1. 서비스와 주변기기에 대한 매개변수를 사용할 포트로 설정하십시오.
2. 추가 설정 매개변수를 표시하기 위하여 기체를 재부팅합니다.
3. Set the baud rate parameter for the selected port to the desired value (e.g. [SER_GPS1_BAUD](../advanced_config/parameter_reference.md#SER_GPS1_BAUD))
4. Configure module-specific parameters (i.e. MAVLink streams and data rate configuration).

The [GPS/Compass > Secondary GPS](../gps_compass/index.md#dual_gps) section provides a practical example of how to configure a port in _QGroundControl_ (it shows how to use `GPS_2_CONFIG` to run a secondary GPS on the `TELEM 2` port).

Similarly [PX4 Ethernet Setup > PX4 MAVLink Serial Port Configuration](../advanced_config/ethernet_setup.md#px4-mavlink-serial-port-configuration) explains the setup for Ethernet serial ports, and [MAVLink Peripherals (OSD/GCS/Companion Computers/etc.)](../peripherals/mavlink_peripherals.md) explains the configuration for MAVLink serial ports.

## Deconflicting Ports

Port conflicts are handled by system startup, which ensures that at most one service is run on a specific port.
For example, it is not possible to start a MAVLink instance on a specific serial device, and then launch a driver that uses the same serial device.

:::warning
At time of writing there is no user feedback about conflicting ports.
:::

<a id="default_port_mapping"></a>

## Default Serial Port Configuration

:::tip
These port mappings can be disabled by setting the associated configuration parameter to _Disabled_.
:::

The following ports are commonly mapped to specific functions on all boards:

- `GPS 1` is configured as a GPS port (using [GPS_1_CONFIG](../advanced_config/parameter_reference.md#GPS_1_CONFIG)).

  The default baud rate is set in the [gps driver](../modules/modules_driver.md#gps) by the value of [SER_GPS1_BAUD](../advanced_config/parameter_reference.md#SER_GPS1_BAUD), which has a default rate of _Auto_.
  With this setting a GPS will automatically detect the baudrate — except for the Trimble MB-Two, which you will need to explicitly set to 115200 baud rate.

- `RC IN` is configured as an RC input (using [RC_PORT_CONFIG](../advanced_config/parameter_reference.md#RC_PORT_CONFIG)).

- `TELEM 1` is configured as a MAVLink serial port suitable for connection to a GCS via a [telemetry module](../telemetry/index.md).

  The configuration uses [MAV_0_CONFIG](../advanced_config/parameter_reference.md#MAV_0_CONFIG) to set the port, [MAV_0_RATE](../advanced_config/parameter_reference.md#MAV_0_RATE) to set the baud rate to 57600, and [MAV_0_MODE](../advanced_config/parameter_reference.md#MAV_1_MODE) to set the messages streamed to "Normal".
  For more information see: [MAVLink Peripherals (OSD/GCS/Companion Computers/etc.)](../peripherals/mavlink_peripherals.md).

- `TELEM 2` is configured by default as a MAVLink serial port suitable for connection to an Onboard/Companion computer via a wired connection.

  The configuration uses [MAV_1_CONFIG](../advanced_config/parameter_reference.md#MAV_1_CONFIG) to set the port, [MAV_1_RATE](../advanced_config/parameter_reference.md#MAV_1_RATE) to set the baud rate, and [MAV_1_MODE](../advanced_config/parameter_reference.md#MAV_2_MODE) to set the messages streamed to "Onboard".
  For more information see: [MAVLink Peripherals (OSD/GCS/Companion Computers/etc.)](../peripherals/mavlink_peripherals.md).

- `Ethernet` is mapped as a MAVLink port on Pixhawk devices that have an Ethernet port.

  The configuration uses [MAV_2_CONFIG](../advanced_config/parameter_reference.md#MAV_2_CONFIG) and appropriate settings for the UDP port etc.
  For more information see [PX4 Ethernet Setup > PX4 MAVLink Serial Port Configuration](../advanced_config/ethernet_setup.md#px4-mavlink-serial-port-configuration) and [MAVLink Peripherals (OSD/GCS/Companion Computers/etc.)](../peripherals/mavlink_peripherals.md).

- `USB-C` (the USB-C port normally used for connecting to QGroundControl)

  This is configured by default as a MAVLink port the onboard profile (for companion computers).
  The configuration for MAVLink is unique to this port (it doesn't use the `MAV_X_CONFIG` parameters).

  - [SYS_USB_AUTO](../advanced_config/parameter_reference.md#SYS_USB_AUTO) sets whether the port is set to no partiular protocol, autodetects the protocol, or sets the comms link to MAVLink.
  - [USB_MAV_MODE](../advanced_config/parameter_reference.md#USB_MAV_MODE) sets the MAVLink profile that is used if MAVLink is set or detected.

Other ports generally have no assigned functions by default (are disabled).

## 문제 해결

<a id="parameter_not_in_firmware"></a>

### Configuration Parameter Missing from _QGroundControl_

_QGroundControl_ only displays the parameters for services/drivers that are present in firmware.
펌웨어에 누락된 매개변수를 추가할 수 있습니다.

:::info
PX4 firmware includes most drivers by default on [Pixhawk-series](../flight_controller/pixhawk_series.md) boards.
플래시 제한 보드는 드라이버를 주석 처리하거나 생략할 수 있습니다(작성 시점에는 FMUv2 기반 보드에만 영향을 미침).
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

## 추가 정보

- [MAVLink Peripherals (OSD/GCS/Companion Computers/etc.)](../peripherals/mavlink_peripherals.md)
- [PX4 Ethernet Setup > PX4 MAVLink Serial Port Configuration](../advanced_config/ethernet_setup.md#px4-mavlink-serial-port-configuration)
- [Serial Port Mapping](../hardware/serial_port_mapping.md)
