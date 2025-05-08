# MAVLink Peripherals (GCS/OSD/Gimbal/Camera/Companion)

Ground Control Stations (GCS), On-Screen Displays (OSD), MAVLink Cameras & Gimbals, Remote IDs, Companion Computers, ADS-B receivers, and other MAVLink peripherals interact with PX4 using separate MAVLink streams, sent via different serial ports.

In order to configure that a particular serial port is used for MAVLink traffic with a particular peripheral, we use [Serial Port Configuration](../peripherals/serial_configuration.md), assigning one of the abstract "MAVLink instance" configuration parameters to the desired port.
Потім ми встановлюємо інші властивості каналу MAVLink, використовуючи параметри, пов'язані з обраним екземпляром MAVLink, щоб вони відповідали вимогам нашого конкретного периферійного пристрою.

The most relevant parameters are described below (the full set are listed in the [Parameter Reference > MAVLink](../advanced_config/parameter_reference.md#mavlink)).

## Екземпляри MAVLink

In order to assign a particular peripheral to a serial port we use the concept of a _MAVLink instance_.

Кожен екземпляр MAVLink представляє певну конфігурацію MAVLink, яку ви можете застосувати до певного порту.
At time of writing three MAVLink _instances_ are defined, each represented by a parameter [MAV_X_CONFIG](#MAV_X_CONFIG), where X is 0, 1, 2.

Each instance has associated parameters that you can use to define the properties of the instance on that port, such as the set of streamed messages (see [MAV_X_MODE](#MAV_X_MODE) below), data rate ([MAV_X_RATE](#MAV_X_RATE)), whether incoming traffic is forwarded to other MAVLink instances ([MAV_X_FORWARD](#MAV_X_FORWARD)), and so on.

:::info
MAVLink instances are an abstract concept for a particular MAVLink configuration.
Число в назві нічого не означає; ви можете призначити будь-який екземпляр до будь-якого порту.
:::

Параметри для кожного екземпляру є:

- <a id="MAV_X_CONFIG"></a>[MAV_X_CONFIG](../advanced_config/parameter_reference.md#MAV_0_CONFIG) - Set the serial port (UART) for this instance "X", where X is 0, 1, 2.
  It can be any unused port, e.g.: `TELEM2`, `TELEM3`, `GPS2` etc.
  For more information see [Serial Port Configuration](../peripherals/serial_configuration.md).

- <a id="MAV_X_MODE"></a>[MAV_X_MODE](../advanced_config/parameter_reference.md#MAV_0_MODE) - Specify the telemetry mode/target (the set of messages to stream for the current instance and their rate).
  Значення за замовчуванням:

  - _Normal_: Standard set of messages for a GCS.
  - _Custom_ or _Magic_: Nothing (in the default PX4 implementation).
    Режими можуть бути використані для тестування при розробці нового режиму.
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
  - Це комбінована ставка для всіх потоків окремого повідомлення (ставки для окремих повідомлень зменшуються, якщо загальна ставка перевищує це значення).
  - За замовчуванням налаштування, як правило, буде прийнятним, але може бути зменшено, якщо телеметричний зв'язок стає насиченим і занадто багато повідомлень втрачається.
  - Значення 0 встановлює швидкість передачі даних вдвічі менше теоретичного значення.

- <a id="MAV_X_FORWARD"></a>[MAV_X_FORWARD](../advanced_config/parameter_reference.md#MAV_0_FORWARD) - Enable forwarding of MAVLink packets received by the current instance onto other interfaces.
  Це може бути використано, наприклад, для передачі повідомлень між GCS та супутнім комп'ютером, щоб GCS міг спілкуватися з камерою, підключеною до супутнього комп'ютера, яка підтримує MAVLink.

Next you need to set the baud rate for the serial port you assigned above (in `MAV_X_CONFIG`).

:::tip
You will need to reboot PX4 to make the parameter available (i.e. in QGroundControl).
:::

The parameter used will depend on the [assigned serial port](../advanced_config/parameter_reference.md#serial) - for example: `SER_GPS1_BAUD`, `SER_TEL2_BAUD`, etc.
Значення, яке ви використовуєте, буде залежати від типу підключення та можливостей підключеного периферійного пристрою MAVLink.

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

Пристрої Pixhawk 5x (і пізніші), які мають порт Ethernet, за замовчуванням налаштовані на підключення до GCS:

On this hardware, there is a [default serial port mapping](../peripherals/serial_configuration.md#default_port_mapping) of MAVLink instance 2 as shown below:

- [MAV_2_CONFIG](../advanced_config/parameter_reference.md#MAV_2_CONFIG) = `Ethernet` (1000)
- [MAV_2_BROADCAST](../advanced_config/parameter_reference.md#MAV_2_BROADCAST) = `1`
- [MAV_2_MODE](../advanced_config/parameter_reference.md#MAV_2_MODE) = `0` (normal/GCS)
- [MAV_2_RADIO_CTL](../advanced_config/parameter_reference.md#MAV_2_RADIO_CTL) = `0`
- [MAV_2_RATE](../advanced_config/parameter_reference.md#MAV_2_RATE) = `100000`
- [MAV_2_REMOTE_PRT](../advanced_config/parameter_reference.md#MAV_2_REMOTE_PRT)= `14550` (GCS)
- [MAV_2_UDP_PRT](../advanced_config/parameter_reference.md#MAV_2_UDP_PRT) = `14550` (GCS)

For more information see: [PX4 Ethernet Setup](../advanced_config/ethernet_setup.md)

## Налаштування пристрою

Links to setup instructions for specific MAVLink components:

- [MAVLink Cameras (Camera Protocol v2) > PX4 Configuration](../camera/mavlink_v2_camera.md#px4-configuration)
- [Gimbal Configuration > MAVLink Gimbal (MNT_MODE_OUT=MAVLINK)](../advanced/gimbal_control.md#mavlink-gimbal-mnt-mode-out-mavlink)

## Дивіться також

- [Serial Port Configuration](../peripherals/serial_configuration.md)
- [PX4 Ethernet Setup > PX4 MAVLink Serial Port Configuration](../advanced_config/ethernet_setup.md#px4-mavlink-serial-port-configuration)
- [Serial Port Mapping](../hardware/serial_port_mapping.md)

