# Приймачі ADS-B/FLARM/UTM: Уникнення повітряного трафіку

PX4 supports simple air traffic avoidance in [missions](../flying/missions.md) using [ADS-B](https://en.wikipedia.org/wiki/Automatic_dependent_surveillance_%E2%80%93_broadcast), [FLARM](https://en.wikipedia.org/wiki/FLARM), or [UTM](https://www.faa.gov/uas/research_development/traffic_management) transponders that use the standard MAVLink interfaces.

If a potential collision is detected, PX4 can _warn_, immediately [land](../flight_modes_mc/land.md), or [return](../flight_modes_mc/return.md) (depending on the value of [NAV_TRAFF_AVOID](#NAV_TRAFF_AVOID)).

## Підтримуване обладнання

PX4 traffic avoidance works with ADS-B or FLARM products that supply transponder data using the MAVLink [ADSB_VEHICLE](https://mavlink.io/en/messages/common.html#ADSB_VEHICLE) message, and UTM products that supply transponder data using the MAVLink [UTM_GLOBAL_POSITION](https://mavlink.io/en/messages/common.html#UTM_GLOBAL_POSITION) message.

Було протестовано з наступними пристроями:

- [PingRX ADS-B Receiver](https://uavionix.com/product/pingrx-pro/) (uAvionix)
- [FLARM](https://flarm.com/products/uav/atom-uav-flarm-for-drones/) <!-- I think originally https://flarm.com/products/powerflarm/uav/ -->

## Налаштування програмного забезпечення

Будь-який з пристроїв може бути підключений до будь-якого вільного/не використаного послідовного порту на контролері польоту.
Most commonly they are connected to `TELEM2` (if this is not being use for some other purpose).

### PingRX

Порт PingRX MAVLink використовує роз'єм-матинг JST ZHR-4 з роз'ємом, як показано нижче.

| Pin                        | Сигнал                      | Вольтаж      |
| -------------------------- | --------------------------- | ------------ |
| 1 (red) | RX (IN)  | +5V tolerant |
| 2 (blk) | TX (OUT) |              |
| 3 (blk) | Power                       | +4 to 6V     |
| 4 (blk) | GND                         | GND          |

The PingRX comes with connector cable that can be attached directly to the TELEM2 port (DF13-6P) on an [mRo Pixhawk](../flight_controller/mro_pixhawk.md).
Для інших портів або плат, вам доведеться отримати свій власний кабель.

## FLARM

FLARM has an on-board DF-13 6 Pin connector that has an identical pinout to the [mRo Pixhawk](../flight_controller/mro_pixhawk.md).

| Pin                        | Сигнал                      | Вольтаж               |
| -------------------------- | --------------------------- | --------------------- |
| 1 (red) | VCC                         | +4В до +36В           |
| 2 (blk) | TX (OUT) | +3.3V |
| 3 (blk) | RX (IN)  | +3.3V |
| 4 (blk) | -                           | +3.3V |
| 5 (blk) | -                           | +3.3V |
| 6 (blk) | GND                         | GND                   |

:::info
The TX and RX on the flight controller must be connected to the RX and TX on the FLARM, respectively.
:::

## Конфігурація програмного забезпечення

### Конфігурація порту

The recievers are configured in the same way as any other [MAVLink Peripheral](../peripherals/mavlink_peripherals.md).
The only _specific_ setup is that the port baud rate must be set to 57600 and the a low-bandwidth profile (`MAV_X_MODE`).

Assuming you have connected the device to the TELEM2 port, [set the parameters](../advanced_config/parameters.md) as shown:

- [MAV_1_CONFIG](../advanced_config/parameter_reference.md#MAV_1_CONFIG) = TELEM 2
- [MAV_1_MODE](../advanced_config/parameter_reference.md#MAV_1_MODE) = Normal
- [MAV_1_RATE](../advanced_config/parameter_reference.md#MAV_1_RATE) = 0 (default sending rate for port).
- [MAV_1_FORWARD](../advanced_config/parameter_reference.md#MAV_1_FORWARD) = Enabled

Перезавантажте пристрій.

You will now find a new parameter called [SER_TEL2_BAUD](../advanced_config/parameter_reference.md#SER_TEL2_BAUD), which must be set to 57600.

### Налаштування передачі трафіку

Налаштуйте дію у випадку потенційної зіткнення за допомогою параметри нижче:

| Параметр                                                                                                                                                                   | Опис                                                                                                                                                                                                                                         |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="NAV_TRAFF_AVOID"></a>[NAV_TRAFF_AVOID](../advanced_config/parameter_reference.md#NAV_TRAFF_AVOID)                         | Увімкніть режим уникнення трафіку, вказавши відповідь уникнення. 0: Вимкнути, 1: Лише попередження, 2: Режим повернення, 3: Режим посадки.   |
| <a id="NAV_TRAFF_A_HOR"></a>[NAV_TRAFF_A_HOR](../advanced_config/parameter_reference.md#NAV_TRAFF_A_HOR)    | Горизонтальний радіус циліндра навколо транспортного засобу, який визначає його повітряний простір (тобто повітряний простір на земельній площині).                                                       |
| <a id="NAV_TRAFF_A_VER"></a>[NAV_TRAFF_A_VER](../advanced_config/parameter_reference.md#NAV_TRAFF_A_VER)    | Vertical height above and below vehicle of the cylinder that defines its airspace (also see [NAV_TRAFF_A_HOR](#NAV_TRAFF_A_HOR)).          |
| <a id="NAV_TRAFF_COLL_T"></a>[NAV_TRAFF_COLL_T](../advanced_config/parameter_reference.md#NAV_TRAFF_COLL_T) | Поріг часу зіткнення. Уникнення буде викликати якщо передбачуваний час, поки зіткнення не знизиться нижче цієї вартості (орієнтовний час ґрунтується на відносній швидкості руху та UAV). |

## Імплементація

### ADSB/FLARM

PX4 слухає дійсні звіти про транспондери під час місій.

Якщо отримано дійсний звіт від транспондера, PX4 спочатку використовує інформацію про транспондер руху, щоб оцінити, чи показує напрямок та висота руху, що буде перетинатися з повітряним простором БПЛА.
The UAV airspace consists of a surrounding cylinder defined by the radius [NAV_TRAFF_A_HOR](#NAV_TRAFF_A_HOR) and height [NAV_TRAFF_A_VER](#NAV_TRAFF_A_VER), with the UAV at it's center.
The traffic detector then checks if the time until intersection with the UAV airspace is below the [NAV_TRAFF_COLL_T](#NAV_TRAFF_COLL_T) threshold based on the relative speed.
If the both checks are true, the [Traffic Avoidance Failsafe](../config/safety.md#traffic-avoidance-failsafe) action is started, and the vehicle will either warn, land, or return.

The code can be found in `Navigator::check_traffic` ([/src/modules/navigator/navigator_main.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/navigator/navigator_main.cpp)).

PX4 також передаватиме дані транспондера на GCS, якщо це було налаштовано для екземпляра MAVLink (це рекомендується).
Останні 10 цифр GUID відображаються як ідентифіфікація дрона.

### UTM

PX4 listens for `UTM_GLOBAL_POSITION` MAVLink messages during missions.
When a valid message is received, its validity flags, position and heading are mapped into the same `transponder_report` UORB topic used for _ADS-B traffic avoidance_.

The implementation is otherwise _exactly_ as described in the section above.

:::info
[UTM_GLOBAL_POSITION](https://mavlink.io/en/messages/common.html#UTM_GLOBAL_POSITION) contains additional fields that are not provided by an ADSB transponder (see [ADSB_VEHICLE](https://mavlink.io/en/messages/common.html#ADSB_VEHICLE)).
Поточна реалізація просто відкидає додаткові поля (включаючи інформацію про заплановану наступну точку шляху транспортного засобу).
:::

## Тестування/Симульований трафік ADSB

Ви можете симулювати трафік ADS-B для тестування.
Note that this requires that you [Build PX4](../dev_setup/building_px4.md).

:::info
Simulated ADS-B traffic can trigger real failsafe actions.
Використовуйте обережно в реальному польоті!
:::

Щоб увімкнути цю функцію:

1. Uncomment the code in `AdsbConflict::run_fake_traffic()`([AdsbConflict.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/adsb/AdsbConflict.cpp#L342C1-L342C1)).
2. Перебудувати та запустити PX4.
3. Execute the [`navigator fake_traffic` command](../modules/modules_controller.md#navigator) in the [QGroundControl MAVLink Shell](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_console.html) (or some other [PX4 Console or MAVLink shell](../debug/consoles.md), such as the PX4 simulator terminal).

The code in `run_fake_traffic()` is then executed.
Ви повинні бачити попередження ADS-B в консолі/оболонці MAVLink, і QGC також повинен показувати спливаюче вікно з даними про рух літаків ADS-B.

By default `run_fake_traffic()` publishes a number of traffic messages (it calls [`AdsbConflict::fake_traffic()`](#fake-traffic-method) to emit each report).
Ці симулюють трафік ADS-B там, де може виникнути конфлікт, де конфлікту не буде, а також спамять буфер трафіку.

:::details
Information about the test methods

The relevent methods are defined in [AdsbConflict.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/adsb/AdsbConflict.cpp#L342C1-L342C1).

#### `run_fake_traffic()` method

The `run_fake_traffic()` method is run when the `navigator fake_traffic` command is called.

The method calls the `fake_traffic()` method to generate simulated transponder messages around the current vehicle position.
Воно проходить в поточному положенні транспортного засобу, інформація про симульований трафік, такий як виклик, відстані, напрямки, різниці в висоті, швидкості та типи емітерів.

The (commented out) code in `run_fake_traffic()` simulates a number of different scenarios, including conflicts and non-conflicts, as well as spamming the traffic buffer.

#### `fake_traffic()` method

`AdsbConflict::fake_traffic()` is called by the [`run_fake_traffic()`](#run-fake-traffic-method) to create individual ADS-B transponder reports.

Це приймає кілька параметрів, які вказують на характеристики фальшивого трафіку:

- `callsign`: Callsign of the fake transponder.
- `distance`: Horizontal distance to the fake vehicle from the current vehicle.
- `direction`: Direction in NED from this vehicle to the fake in radians.
- `traffic_heading`: Travel direction of the traffic in NED in radians.
- `altitude_diff`: Altitude difference of the fake traffic. Позитив вгору.
- `hor_velocity`: Horizontal velocity of fake traffic, in m/s.
- `ver_velocity`: Vertical velocity of fake traffic, in m/s.
- `emitter_type`: Type of fake vehicle, as an enumerated value.
- `icao_address`: ICAO address.
- `lat_uav`: Lat of this vehicle (used to position fake traffic around vehicle)
- `on_uav`: Lon of this vehicle (used to position fake traffic around vehicle)
- `alt_uav`: Altitude of the vehicle (as reference - used to position fake traffic around vehicle)

Метод створює симульоване повідомлення транспондера біля транспортного засобу, використовуючи наступні кроки:

- Обчислює широту та довготу трафіку на основі позиції БПЛА, відстані та напрямку.
- Обчислює нову висоту, додавши різницю висоти до висоти БПЛА.
- Populates a [TransponderReport](../msg_docs/TransponderReport.md) topic with the simulated traffic data.
- If the board supports a Universally Unique Identifier (UUID), the method retrieves the UUID using `board_get_px4_guid` and copies it to the `uas_id` field of the structure.
  В іншому випадку воно генерує симульований GUID.
- Publishes the simulated traffic message using `orb_publish`.

:::

<!-- See also implementation PR: https://github.com/PX4/PX4-Autopilot/pull/21283 -->

<!-- See also bug to make this work without uncommenting: https://github.com/PX4/PX4-Autopilot/issues/21810 -->

## Подальша інформація

- [MAVLink Peripherals](../peripherals/mavlink_peripherals.md)
- [Serial Port Configuration](../peripherals/serial_configuration.md)
