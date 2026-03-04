# Налаштування послідовного порту

PX4 defines [default functions](#default-serial-port-configuration) for many flight controller ports, which is why you can plug a GPS module into the port labelled `GPS 1`, an RC receiver into `RC IN`, or a telemetry module into `TELEM 1`, and generally they will just work.

Функції, призначені для портів, повністю конфігуруються за допомогою відповідних параметрів (у більшості випадків).
Ви можете призначити будь-який не використаний порт для будь-якої функції або перепризначити порт, щоб використовувати його для іншої цілі.

Конфігурація дозволяє легко (наприклад):

- Виконайте MAVLink на іншому порту, змініть потокові повідомлення або перемкніть порт TELEM на використання ROS 2/XRCE-DDS.
- Змініть швидкість передачі даних на порту або встановіть UDP-порт
- Налаштування подвійного GPS.
- Enable sensors that run on a serial port, such as some [distance sensors](../sensor/rangefinders.md).

::: info

- Деякі порти не можуть бути налаштовані, оскільки вони використовуються для дуже конкретної цілі, наприклад, для системної консолі.
- The mapping of specific devices to port names on the flight controller is explained in [Serial Port Mapping](../hardware/serial_port_mapping.md).

:::

## Налаштування параметрів

Параметри конфігурації послідовного порту дозволяють вам призначити певну функцію або підтримку певного обладнання для конкретного порту.
These parameters follow the naming pattern `*_CONFIG` or `*_CFG`

:::info
_QGroundControl_ only displays the parameters for services/drivers that are present in firmware.
:::

На момент написання поточний набір:

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

Деякі функції / можливості можуть визначати додаткові параметри конфігурації, які будуть містити схожий шаблон іменування до префіксу конфігурації порту.
For example, `MAV_0_CONFIG` enables MAVLink on a particular port, but you may also need to set [MAV_0_FLOW_CTRL](../advanced_config/parameter_reference.md#MAV_0_FLOW_CTRL), [MAV_0_FORWARD](../advanced_config/parameter_reference.md#MAV_0_FLOW_CTRL), [MAV_0_MODE](../advanced_config/parameter_reference.md#MAV_0_MODE) and so on.

## Як налаштувати порт

Всі послідовні драйвери/порти налаштовані однаковим чином:

1. Встановіть параметр конфігурації для сервісу/периферійного пристрою на порт, який він буде використовувати.
2. Перезавантажте апарат, щоб побачити додаткові параметри конфігурації.
3. Set the baud rate parameter for the selected port to the desired value (e.g. [SER_GPS1_BAUD](../advanced_config/parameter_reference.md#SER_GPS1_BAUD))
4. Налаштуйте параметри, специфічні для модуля (тобто. Потоки та конфігурація швидкості передачі даних MAVLink).

The [GPS/Compass > Secondary GPS](../gps_compass/index.md#dual_gps) section provides a practical example of how to configure a port in _QGroundControl_ (it shows how to use `GPS_2_CONFIG` to run a secondary GPS on the `TELEM 2` port).

Similarly [PX4 Ethernet Setup > PX4 MAVLink Serial Port Configuration](../advanced_config/ethernet_setup.md#px4-mavlink-serial-port-configuration) explains the setup for Ethernet serial ports, and [MAVLink Peripherals (OSD/GCS/Companion Computers/etc.)](../peripherals/mavlink_peripherals.md) explains the configuration for MAVLink serial ports.

## Розконфліктовування портів

Конфлікти портів вирішуються під час запуску системи, що забезпечує, що на конкретному порту запускається не більше однієї служби.
Наприклад, неможливо запустити екземпляр MAVLink на конкретному послідовному пристрої, а потім запустити драйвер, який використовує той самий послідовний пристрій.

:::warning
At time of writing there is no user feedback about conflicting ports.
:::

<a id="default_port_mapping"></a>

## Налаштування послідовного порту за замовчуванням

:::tip
These port mappings can be disabled by setting the associated configuration parameter to _Disabled_.
:::

Наступні порти зазвичай відображаються на конкретні функції на всіх платах:

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

Інші порти, як правило, за замовчуванням не мають призначених функцій (вимкнені).

## Усунення проблем

<a id="parameter_not_in_firmware"></a>

### Configuration Parameter Missing from _QGroundControl_

_QGroundControl_ only displays the parameters for services/drivers that are present in firmware.
Якщо параметр відсутній, то можливо вам потрібно додати його в прошивку.

:::info
PX4 firmware includes most drivers by default on [Pixhawk-series](../flight_controller/pixhawk_series.md) boards.
Дошки з обмеженням Flash можуть закоментувати/пропустити драйвер (на момент написання цього це стосується лише дошок на основі FMUv2).
:::

You can include the missing driver in firmware by enabling the driver in the **default.px4board** config file that corresponds to the [board](https://github.com/PX4/PX4-Autopilot/tree/main/boards/px4) you want to build for.
Наприклад, щоб увімкнути драйвер SRF02, ви додасте наступний рядок до px4board.

```
CONFIG_DRIVERS_DISTANCE_SENSOR_SRF02=y
```

Простіший метод полягає в використанні boardconfig, який запускає GUI, де ви можете легко шукати, вимикаючи та увімкнюючи модулі.
Для запуску типу boardconfig введіть:

```
make <vendor>_<board>_<label> boardconfig
```

You will then need to build the firmware for your platform, as described in [Building PX4 Software](../dev_setup/building_px4.md).

## Подальша інформація

- [MAVLink Peripherals (OSD/GCS/Companion Computers/etc.)](../peripherals/mavlink_peripherals.md)
- [PX4 Ethernet Setup > PX4 MAVLink Serial Port Configuration](../advanced_config/ethernet_setup.md#px4-mavlink-serial-port-configuration)
- [Serial Port Mapping](../hardware/serial_port_mapping.md)
