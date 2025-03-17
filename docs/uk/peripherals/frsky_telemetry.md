# Телеметрія FrSky

FrSky telemetry allows you to access vehicle [telemetry/status](#messages) information on a compatible RC transmitter.

Available [telemetry is listed here](#messages), and includes: flight mode, battery level, RC signal strength, speed, altitude etc.
Деякі передавачі можуть додатково надавати аудіо- та вібраційний зворотний зв'язок, що особливо корисно для попереджень про низький рівень заряду акумулятора та інших аварійних сигналів.

PX4 supports both [S.Port](#s_port) (new) and D (old) FrSky telemetry ports.

## Налаштування програмного забезпечення

FrSky телеметрія вимагає:

- An [FrSky-compatible RC transmitter](#transmitters) like the FrSky Taranis X9D Plus.
- An [FrSky telemetry-capable receiver](#receivers) like the XSR and X8R.
- Кабель для підключення приймача FrSky Smart Port (SPort) до UART контролера польоту.

First [connect the receiver for RC channels](../getting_started/rc_transmitter_receiver.md#connecting-receivers), e.g. connect the S.Bus ports on the receiver and the flight controller.

Then set up FrSky telemetry by separately connecting the SPort on the receiver to any free UART on the flight controller, and then [configure PX4 to run FrSky telemetry on that UART](#configure).

Це робиться трохи по-іншому, залежно від того, чи є у приймача SPort контакт для невертованого виходу, і/або версія Pixhawk.

### Pixhawk FMUv4 (і попередні)

For Pixhawk FMUv4 and earlier, UART ports and receiver telemetry ports are typically incompatible (with the exception of [Pixracer](../flight_controller/pixracer.md)).

Generally SPort receivers have an _inverted_ S.Port signal and you have to use a converter cable to split the S.Port into uninverted TX and RX for connecting to the Pixhawk UART.
Приклад показано нижче.

![FrSky-Taranis-Telemetry](../../assets/hardware/telemetry/frsky_telemetry_overview.jpg)

:::tip
When connecting to an inverted S.Port it is usually cheaper and easier to buy a [ready made cable](#ready_made_cable) that contains this adapter and has the appropriate connectors for the autopilot and receiver.
Creating a [DIY cable](#diy_cables) requires electronics assembly expertise.
:::

If using an S.Port receiver with a pin for _uninverted output_ you can simply attach one of the UART's TX pins.

<!-- FYI only: The uninverted output can be used in single-wire mode so you don't need both RX and TX wires.
Discussion of that here: https://github.com/PX4/PX4-user_guide/pull/755#pullrequestreview-464046128 -->

Then [configure PX4](#configure).

### Pixhawk FMUv5/STM32F7 та пізніше

Для Pixhawk FMUv5 та пізніших версій PX4 може читати сигнали S.Port безпосередньо у зворотньому (або незворотньому) вигляді - не потрібен жоден спеціальний кабель.

:::info
More generally this is true on autopilots with STM32F7 or later (e.g. [Durandal](../flight_controller/durandal.md) has a STM32H7 and can read inverted or uninverted S.Port signals directly).
:::

Просто підключіть один з TX-пінів UART до інвертованого або неінвертованого піна SPort (PX4 автоматично виявить і обробить будь-який тип).
Then [configure PX4](#configure).

<a id="configure"></a>

## Конфігурація PX4

[Configure the serial port](../peripherals/serial_configuration.md) on which FrSky will run using [TEL_FRSKY_CONFIG](../advanced_config/parameter_reference.md#TEL_FRSKY_CONFIG).
Немає потреби встановлювати швидкість передачі для порту, оскільки це налаштовано драйвером.

:::info
You can use any free UART, but typically `TELEM 2` is used for FrSky telemetry (except for [Pixracer](../flight_controller/pixracer.md), which is pre-configured to use the _FrSky_ port by default).
:::

:::tip
If the configuration parameter is not available in _QGroundControl_ then you may need to [add the driver to the firmware](../peripherals/serial_configuration.md#parameter_not_in_firmware):

```
drivers/telemetry/frsky_telemetry
```

:::

Додаткова конфігурація не потрібна; телеметрія FrSky автоматично запускається при підключенні та виявляє режим D або S.

<a id="transmitters"></a>

## Сумісні RC передавачі

Вам знадобиться передавач RC, який може отримувати поток телеметрії (і який зв'язаний з приймачем FrSky).

Серед популярних альтернатив:

- FrSky Taranis X9D Plus (рекомендовано)
- FrSky Taranis X9D
- FrSky Taranis X9D
- FrSky Taranis Q X7
- Turnigy 9XR Pro

Вищезазначені передавачі можуть відображати телеметричні дані без будь-якої додаткової конфігурації. Наступний розділ(и) пояснюють, як ви можете налаштувати відображення телеметрії (наприклад, для створення кращого інтерфейсу користувача).

### Taranis - Налаштування LuaPilot

Сумісні приймачі Taranis (наприклад, X9D Plus), які працюють на OpenTX 2.1.6 або новіше, можуть використовувати сценарій LuaPilot для зміни відображеної телеметрії (як показано на знімку екрану нижче).

![Telemetry Screen on the Taranis](../../assets/hardware/telemetry/taranis_telemetry.jpg)

Instructions for installing the script can be found here: [LuaPilot Taranis Telemetry script > Taranis Setup OpenTX 2.1.6 or newer](http://ilihack.github.io/LuaPilot_Taranis_Telemetry/)

If you open the `LuaPil.lua` script with a text editor, you can edit the configuration. Запропоновані модифікації включають:

- `local BattLevelmAh = -1` - Use the battery level calculation from the vehicle
- `local SayFlightMode = 0` - There are no WAV files for the PX4 flight modes

<a id="messages"></a>

## Телеметричні повідомлення

Телеметрія FrySky може передавати більшість корисної інформації про стан з PX4.
Отримувачі S-Port та D-Port передають різні набори повідомлень, як перелічено в наступних розділах.

<a id="s_port"></a>

### S-Port

S-Port receivers transmit the following messages from PX4 (from [here](https://github.com/iNavFlight/inav/blob/master/docs/Telemetry.md#available-smartport-sport-sensors)):

- **AccX, AccY, AccZ:** Accelerometer values.
- **Alt:** Barometer based altitude, relative to home location.
- **Curr:** Actual current consumption (Amps).
- **Fuel:** Remaining battery percentage if `battery_capacity` variable set and variable `smartport_fuel_percent = ON`, mAh drawn otherwise.
- **GAlt:** GPS altitude, sea level is zero.
- **GPS:** GPS coordinates.
- **GSpd:** Current horizontal ground speed, calculated by GPS.
- **Hdg:** Heading (degrees - North is 0°).
- **VFAS:** Actual battery voltage value (Voltage FrSky Ampere Sensor).
- **VSpd:** Vertical speed (cm/s).
- **Tmp1:** [Flight mode](../flight_modes/index.md#flight-modes), sent as an integer: 18 - Manual, 23 - Altitude, 22 - Position, 27 - Mission, 26 - Hold, 28 - Return, 19 - Acro, 24 0 Offboard, 20 - Stabilized, 25 - Takeoff, 29 - Land, 30 - Follow Me.
- **Tmp2:** GPS information. Найправіший розрядок - це тип виправлення GPS (0 = жоден, 2 = 2D, 3 = 3D). Інші цифри - це кількість супутників.

:::info
The following "standard" S-Port messages are not supported by PX4: **ASpd**, **A4**.
:::

<!-- FYI:
Values of FRSKY_ID_TEMP1 and FRSKY_ID_TEMP1 set:
- https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/telemetry/frsky_telemetry/frsky_telemetry.cpp#L85  (get_telemetry_flight_mode)
- https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/telemetry/frsky_telemetry/frsky_data.cpp#L234-L237
Lua map of flight modes:
- https://github.com/ilihack/LuaPilot_Taranis_Telemetry/blob/master/SCRIPTS/TELEMETRY/LuaPil.lua#L790
-->

### D-порт

D-Port receivers transmit the following messages (from [here](https://github.com/cleanflight/cleanflight/blob/master/docs/Telemetry.md)):

- **AccX, AccY, AccZ:** Accelerometer values.
- **Alt:** Barometer based altitude, init level is zero.
- **Cels:** Average cell voltage value (battery voltage divided by cell number).
- **Curr:** Actual current consumption (Amps).
- **Fuel:** Remaining battery percentage if capacity is set, mAh drawn otherwise.
- **Date:** Time since powered.
- **GAlt:** GPS altitude, sea level is zero.
- **GPS:** GPS coordinates.
- **GSpd:** Current speed, calculated by GPS.
- **Hdg:** Heading (degrees - North is 0°).
- **RPM:** Throttle value if armed, otherwise battery capacity. Зверніть увагу, що номер леза повинен бути встановлений на 12 в Тараніс.
- **Tmp1:** Flight mode (as for S-Port).
- **Tmp2:** GPS information (as for S-Port).
- **VFAS:** Actual battery voltage value (Voltage FrSky Ampere Sensor).
- **Vspd:** Vertical speed (cm/s).

<a id="receivers"></a>

## FrSky телеметрія Receivers

Pixhawk/PX4 підтримує D (старий) та S (новий) телеметрію FrSky. Таблиця нижче всі FrSky приймачі, які підтримують телеметрію через D/S.PORT (теоретично всі вони повинні працювати).

:::tip
Note that the X series receivers listed below are recommended (e.g. XSR, X8R). Серії R та G не були протестовані / перевірені тестовою командою, але повинні працювати.
:::

| Приймач     | Діапазон              | Комбінований вихід                                                          | Цифровий вхід телеметрії      | Розміри                                                               | Вага                  |
| ----------- | --------------------- | --------------------------------------------------------------------------- | ----------------------------- | --------------------------------------------------------------------- | --------------------- |
| D4R-II      | 1.5km | CPPM (8)                                                 | D.Port        | 40х22.5х6мм                                           | 5.8г  |
| D8R-XP      | 1.5km | CPPM (8)                                                 | D.Port        | 55х25х14мм                                                            | 12,4г                 |
| D8R-II Plus | 1.5km | no                                                                          | D.Port        | 55х25х14мм                                                            | 12,4г                 |
| X4R         | 1.5km | CPPM (8)                                                 | Smart Port                    | 40х22.5х6мм                                           | 5.8г  |
| X4R-SB      | 1.5km | S.Bus (16)                               | Smart Port                    | 40х22.5х6мм                                           | 5.8г  |
| X6R / S6R   | 1.5km | S.Bus (16)                               | Smart Port                    | 47.42×23.84×14.7мм    | 15.4г |
| X8R / S8R   | 1.5km | S.Bus (16)                               | Smart Port                    | 46.25 x 26.6 x 14.2мм | 16,6г                 |
| XSR / XSR-M | 1.5km | S.Bus (16) / CPPM (8) | Smart Port                    | 26x19.2x5мм                                           | 3,8 г                 |
| RX8R        | 1.5km | S.Bus (16)                               | Smart Port                    | 46.25x26.6x14.2мм     | 12.1г |
| RX8R PRO    | 1.5km | S.Bus (16)                               | Smart Port                    | 46.25x26.6x14.2мм     | 12.1г |
| R-XSR       | 1.5km | S.Bus (16) / CPPM (8) | Smart Port                    | 16x11x5.4мм                                           | 1.5г  |
| G-RX8       | 1.5km | S.Bus (16)                               | Smart Port + integrated vario | 55.26_17_8mm                                          | 5.8г  |
| R9          | 10км                  | S.Bus (16)                               | Smart Port                    | 43.3x26.8x13.9мм      | 15,8г                 |
| R9 slim     | 10км                  | S.Bus (16)                               | Smart Port                    | 43.3x26.8x13.9мм      | 15,8г                 |

:::info
The above table originates from http://www.redsilico.com/frsky-receiver-chart and FrSky [product documentation](https://www.frsky-rc.com/product-category/receivers/).
:::

<a id="ready_made_cable"></a>

## Готові кабелі

Готові кабелі для використання з Pixhawk FMUv4 та раніше (крім Pixracer) доступні за адресою:

- [Craft and Theory](http://www.craftandtheoryllc.com/telemetry-cable). Versions are available with DF-13 compatible _PicoBlade connectors_ (for FMUv2/3DR Pixhawk, FMUv2/HKPilot32) and _JST-GH connectors_ (for FMUv3/Pixhawk 2 "The Cube" and FMUv4/PixRacer v1).

  <a href="http://www.craftandtheoryllc.com/telemetry-cable"><img src="../../assets/hardware/telemetry/craft_and_theory_frsky_telemetry_cables.jpg" alt="Purchase cable here from Craft and Theory"></a>

<a id="diy_cables"></a>

## DIY Кабелі

Можливо створити власні адаптерні кабелі.
You will need connectors that are appropriate for your autopilot (e.g. _JST-GH connectors_ for FMUv3/Pixhawk 2 "The Cube" and FMUv4/PixRacer v1, and DF-13 compatible _PicoBlade connectors_ for older autopilots).

Pixracer включає електроніку для перетворення сигналів S.PORT і UART, але для інших плат вам знадобиться адаптер UART на S.PORT.
Ці можна отримати з:

- [FrSky FUL-1](https://www.frsky-rc.com/product/ful-1/): [unmannedtech.co.uk](https://www.unmannedtechshop.co.uk/frsky-transmitter-receiver-upgrade-adapter-ful-1/)
- SPC: [getfpv.com](http://www.getfpv.com/frsky-smart-port-converter-cable.html), [unmannedtechshop.co.uk](https://www.unmannedtechshop.co.uk/frsky-smart-port-converter-spc/)

Додаткова інформація про з'єднання для різних плат подається нижче.

### Pixracer до приймачів S-порту

Підключіть лінії TX та RX Pixracer FrSky разом (припаяйте проводи разом) до контакту S.port приймача серії X.
GND не потрібно прикріплювати, оскільки це буде зроблено під час прикріплення до S.Bus (звичайне з'єднання RC).

З'єднання S-порту показано нижче (використовуючи наданий роз'єм введення/виведення).

![Grau b Pixracer FrSkyS.Port Connection](../../assets/flight_controller/pixracer/grau_b_pixracer_frskys.port_connection.jpg)

![Pixracer FrSkyS.Port Connection](../../assets/flight_controller/pixracer/pixracer_FrSkyTelemetry.jpg)

### Pixracer до приймачів D-порту

:::tip
The vast majority of users now prefer to use S.PORT.
:::

Підключіть лінію Pixracer FrSky TX (FS out) до лінії RX приймача.
Підключіть лінію Pixracer FrSky RX (FS in) до лінії TX приймача.
GND не потрібно підключати, оскільки це буде зроблено під час приєднання до RC/SBus (для звичайного RC).

<!-- Image would be nice -->

### Pixhawk 4

[Pixhawk 3 Pro](../flight_controller/pixhawk3_pro.md) can be connected to TELEM4 (no additional software configuration is needed).
You will need to connect via a UART to S.PORT adapter board, or a [ready-made cable](#ready_made_cable).

### Pixhawk FMUv5 та попередній

Просто підключіть один з TX-пінів UART до інвертованого або неінвертованого піна SPort (PX4 автоматично виявить і обробить будь-який тип).

### Інші плати

Більшість інших плат з'єднуються з приймачем для телеметрії FrSky через UART TELEM2.
This includes, for example: [Pixhawk 1](../flight_controller/pixhawk.md), [mRo Pixhawk](../flight_controller/mro_pixhawk.md), Pixhawk2.

You will need to connect via a UART to S.PORT adapter board, or a [ready-made cable](#ready_made_cable).

<!-- ideally add diagram here -->

## Додаткова інформація

Для отримання додаткової інформації дивіться наступні посилання:

- [FrSky Taranis Telemetry](https://github.com/Clooney82/MavLink_FrSkySPort/wiki/1.2.-FrSky-Taranis-Telemetry)
- [Taranis X9D: Setting Up Telemetry](https://www.youtube.com/watch?v=x14DyvOU0Vc) (Video Tutorial)
- [Px4 FrSky Telemetry Setup with Pixhawk2 and X8R Receiver](https://discuss.px4.io//t/px4-frsky-telemetry-setup-with-pixhawk2-and-x8r-receiver/6362) (DIY Cables)
