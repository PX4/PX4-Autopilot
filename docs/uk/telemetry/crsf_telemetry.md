# Телеметрія CRSF (TBS Crossfire Telemetry)

CRSF is a telemetry protocol that can be used for both [RC control](../getting_started/rc_transmitter_receiver.md) and to get telemetry information from the vehicle/flight controller on a compatible RC transmitter.

The protocol was developed by [Team BlackSheep](https://www.team-blacksheep.com/) for their Crossfire RC system, but is also used by [ExpressLRS (ELRS)](https://www.expresslrs.org/) RC systems.
Це бідирекціональний протокол, який потребує лише одного UART для обміну даними як з RC, так і з телеметрією.

The [supported telemetry messages are listed here](#telemetry-messages), and include: flight mode, battery level, GPS data RC signal strength, speed, altitude, and so on.

:::info
If you don't need telemetry you can connect a TBS Crossfire to the `RCIN` port and configure the receiver to use S.BUS.
Crossfire radio systems can also be used as [Telemetry Radios](../telemetry/index.md).
:::

:::warning
PX4 does not include the CRSF protocol support by default.
The [instructions below](#px4-configuration) explain how to build and upload custom PX4 firmware that includes the required modules.
:::

## Налаштування системи радіо

To use CRSF telemetry you will need a [TBS Crossfire radio system](#tbs-radio-systems) or [ExpressLRS radio system](#expresslrs-radio-systems) that includes an [RC controller](#rc-controllers) with a transmitter, and a receiver (from the same vendor).

:::info
An RC radio system historically consisted of a ground-based controller that transmitted to an on-vehicle receiver.
Навіть якщо багато радіосистем зараз є двосторонніми, земельний модуль все ще може називатися передавачем, а повітряний блок може називатися приймачем.
:::

Generally you will need to separately setup and configure the transmitter and receiver, and then _bind_ them together.

A transmitter might come as an integral part of an [RC controller](#rc-controllers), or it might be a separate module that you plug into a controller.
Якщо це окремий модуль, то вам може знадобитися також оновити програмне забезпечення модуля на передавачі на прошивку, яка підтримує CRSF, таку як OpenTX або EdgeTx.
У обох випадках вам потрібно налаштувати передавач для активації CRSF.

The receiver must be [wired](#wiring) to a spare port (UART) on the Flight Controller.
Then you can _bind_ the transmitter and receiver together.

Інструкції для вищезазначених кроків описані в

- [TBS Crossfire Manual](https://www.team-blacksheep.com/tbs-crossfire-manual.pdf)
- [Express LRS: QuickStart](https://www.expresslrs.org/quick-start/getting-started/)

### Підключення

TX і RX на обраному екрані контролю польоту потрібно підключитися до окремих каналів на приймачі.
Сигнал зазвичай є неінвертованим і може бути підключений безпосередньо (в кабелі не потрібна додаткова логіка інвертора).
Вам слід перевірити посібник для вашого конкретного приймача, однак!

#### Проводка приймача TBS

Для приймачів TBS ви підключаєте UART і приймач FC, як показано (це передбачає TBS Nano RX).

| FC UART | Nano RX |
| ------- | ------- |
| TX      | Ch2     |
| RX      | Ch1     |

#### Проводка приймача ExpressLRS

For ExpressLRS receivers wire to the flight controller UART as shown below (wiring is covered [in detail here](https://www.expresslrs.org/quick-start/receivers/wiring-up/)):

| FC UART | ExpressLRS |
| ------- | ---------- |
| TX      | RX         |
| RX      | TX         |
| VCC     | VCC        |
| GND     | GND        |

## Конфігурація PX4

### Конфігурація прошивки/збірка

Підтримка телеметрії CRSF не включена в жодне ПЗ PX4 за замовчуванням.
To use this feature you must build and upload custom firmware that includes [crsf-rc](../modules/modules_driver.md#crsf-rc) and removes [rc_input](../modules/modules_driver.md#rc-input).

Кроки наступні:

1. [Setup a development environment](../dev_setup/dev_env.md) for building PX4.

  As part of this process you will have used `git` to fetch source code into the **PX4-Autopilot** directory.

2. Open a terminal and `cd` into the `PX4-Autopilot` directory.

  ```sh
  cd PX4-Autopilot
  ```

3. Launch the [PX4 board config tool (`menuconfig`)](../hardware/porting_guide_config.md#px4-menuconfig-setup) for your `make` target using the `boardconfig` option (here the target is the [ARK Electronics ARKV6X](../flight_controller/ark_v6x.md) flight controller):

  ```sh
  make ark_fmu-v6x_default boardconfig
  ```

4. У інструменті конфігурації плати PX4:

  - Disable the default `rc_input` module
    1. Navigate to the `drivers` submenu, then scroll down to highlight `rc_input`.
    2. Use the enter key to remove the `*` from `rc_input` checkbox.
  - Enable the `crsf_rc` module
    1. Scroll to highlight the `RC` submenu, then press enter to open it.
    2. Scroll to highlight `crsf_rc` and press enter to enable it.

  Збережіть і вийдіть з інструменту конфігурації плати PX4.

5. [Build the PX4 source code](../dev_setup/building_px4.md) with your changes (again assuming you are using ARKV6X):

  ```sh
  make ark_fmu-v6x_default
  ```

Це побудує вашу власну прошивку, яку зараз потрібно завантажити на ваш контролер польоту.

### Завантаження прошивки

Щоб завантажити кастомну прошивку, спочатку підключіть ваш контролер польотів до комп’ютера розробки через USB.

You can upload firmware as part of the build process using the `upload` options:

```sh
make ark_fmu-v6x_default upload
```

Alternatively you can use QGroundControl to install the firmware, as described in [Firmware > Installing PX4 master, beta, or custom firmware](../config/firmware.md#installing-px4-main-beta-or-custom-firmware).

### Налаштування параметрів

[Find and set](../advanced_config/parameters.md) the following parameters:

1. [RC_CRSF_PRT_CFG](../advanced_config/parameter_reference.md#RC_CRSF_PRT_CFG) — Set to the port that is connected to the CRSF receiver (such as `TELEM1`).

  This [configures the serial port](../peripherals/serial_configuration.md) to use the CRSF protocol.
  Note that some serial ports may already have a [default serial port mapping](../peripherals/serial_configuration.md#default-serial-port-configuration) or [default MAVLink serial port mapping](../peripherals/mavlink_peripherals.md#default-mavlink-ports) that you will have to un-map before you can assign the port to CRSF.
  For example, if you want to use `TELEM1` or `TELEM2` you first need to modify [MAV_0_CONFIG](../advanced_config/parameter_reference.md#MAV_0_CONFIG) or [MAV_1_CONFIG](../advanced_config/parameter_reference.md#MAV_1_CONFIG) to stop setting those ports.

  Немає потреби встановлювати швидкість передачі для порту, оскільки це налаштовано драйвером.

2. [RC_CRSF_TEL_EN](../advanced_config/parameter_reference.md#RC_CRSF_TEL_EN) — Enable to activate Crossfire telemetry.

### Налаштування радіо

[Radio Control Setup](../config/radio.md) explains how to map your RC controller's attitude control sticks (roll, pitch, yaw, throttle) to channels, and to calibrate the minimum, maximum, trim and reverse settings for all other transmitter controls/RC channels.

## Контролери RC

Трансмітер може бути неот'ємною частиною керування RC, або це може бути окремий модуль, який ви вставляєте в контролер.

RC Контролери, які підтримують модулі TX TBS Crossfire та ExpressLRS:

- [FrSky Taranis X9D Plus](https://www.frsky-rc.com/product/taranis-x9d-plus-2/) has an external module bay that can be used with TBS or ExpressLRS transmitter modules that are "JR module bay" compatible.
  Вам потрібно буде встановити програмне забезпечення OpenTX, яке підтримує CRSF, та увімкнути зовнішній модуль та CRSF.
- [Radiomaster TX16S](https://www.radiomasterrc.com/collections/tx16s-mkii) has an internal ExpressLRS transmitter module.
  Він також має зовнішній модульний бей, який може бути використаний з передавальними модулями TBS або ExpressLRS, які сумісні з "JR module bay".
  Він працює як на програмному забезпеченні OpenTX, так і на програмному забезпеченні EdgeTx, кожне з яких може підтримувати CRSF.

## Радіосистеми TBS

[TBS Crossfire Radio Systems are listed here](https://www.team-blacksheep.com/shop/cat:cat_crossfire#product_listing).
Нижче наведено кілька опцій «turnkey».

Модулі передавача:

- [TBS CROSSFIRE TX - LONG RANGE R/C TRANSMITTER](https://www.team-blacksheep.com/products/prod:crossfire_tx)

Приймачі:

- [TBS Crossfire Nano RX](http://team-blacksheep.com/products/prod:crossfire_nano_rx) - designed for small quadcopters.

## Системи радіо ExpressLRS

Express LRS provide Radio System guidance in the [Hardware Selection](https://www.expresslrs.org/hardware/hardware-selection/) page.
Нижче наведено кілька перевірених варіантів.

Модулі передавача:

- Уточнюється

Приймачі:

- [ExpressLRS Matek Diversity RX](http://www.mateksys.com/?portfolio=elrs-r24).

  ::: info
  This is used in the [Reptile Dragon 2 Build Log](../frames_plane/reptile_dragon_2.md).
  See sections [ELRS Rx](../frames_plane/reptile_dragon_2.md#elrs-rx) and [Radio Setup](../frames_plane/reptile_dragon_2.md#radio-setup).

:::

## Телеметричні повідомлення

The supported telemetry messages and their source are listed below (this table is reproduced from the [TBS Crossfire Manual: "Available sensors with OpenTX"](https://www.team-blacksheep.com/tbs-crossfire-manual.pdf)).

| Datapoint | Опис                                                              | Джерела даних                    |
| --------- | ----------------------------------------------------------------- | -------------------------------- |
| 1RSS      | Uplink - отримана сила сигналу антени 1 (RSSI) | TBS CROSSFIRE RX                 |
| 2RSS      | Uplink - отримана сила сигналу антени 2 (RSSI) | TBS CROSSFIRE RX                 |
| RQly      | Uplink - якість посилання (дійсні пакети)      | TBS CROSSFIRE RX                 |
| RSNR      | Uplink - відношення сигнал/шум                                    | TBS CROSSFIRE RX                 |
| RFMD      | Uplink - частота оновлення, 0 = 4Гц, 1 = 50Гц, 2 = 150Гц          | TBS CROSSFIRE RX                 |
| TPWR      | Uplink - потужність передачі                                      | TBS CROSSFIRE TX                 |
| TRSS      | Downlink - сила сигналу антени                                    | TBS CROSSFIRE TX                 |
| TQly      | Downlink - якість посилання (дійсні пакети)    | TBS CROSSFIRE TX                 |
| TSNR      | Downlink - відношення сигнал/шум                                  | TBS CROSSFIRE TX                 |
| ANT       | Датчик лише для налагодження                                      | TBS CROSSFIRE TX                 |
| GPS       | GPS координати                                                    | TBS GPS / FC                     |
| Alt       | GPS Висоти                                                        | TBS GPS / FC                     |
| Sats      | Супутники GPS отримано                                            | TBS GPS / FC                     |
| Hdg       | Магнітна орієнтація                                               | TBS GPS / FC                     |
| RXBt      | Напруга батареї                                                   | TBS GPS / FC/ CROSSFIRE RX/ CORE |
| Curr      | Поточне витягування                                               | TBS GPS / FC// CORE              |
| Capa      | Поточне споживання                                                | TBS GPS / FC/ CORE               |
| Ptch      | Кут нахилу поля FC                                                | FC                               |
| Roll      | Кут кочення FC                                                    | FC                               |
| Yaw       | Кут курсу FC                                                      | FC                               |
| FM        | Режим польоту                                                     | FC                               |
| VSPD      | Барометр                                                          | FC                               |

## Дивіться також

- [TBS Crossfire Manual](https://www.team-blacksheep.com/tbs-crossfire-manual.pdf)
- [ExpressLRS Documentation](https://www.expresslrs.org/quick-start/getting-started/)
- [FrSky Telemetry](../peripherals/frsky_telemetry.md)
- [Radio Control Setup](../config/radio.md)
- [Radio Control Systems](../getting_started/rc_transmitter_receiver.md)
