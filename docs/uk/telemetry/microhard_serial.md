# Microhard Serial Telemetry Radios

[Microhard Pico Serial Radios](http://microhardcorp.com/P900.php) integrate the [Microhard Pico Serial](http://microhardcorp.com/P900.php) P900 RF module.

Це відносно невеликий за розміром та недорогий радіопередавач, який підтримує такі режими як точка-точка, точка-багатоточка та мережеві режими.
Він має налаштований вихідну потужність та також може бути налаштований на використання корекції помилок передачі.
Радіопередавачі також можна замовити з підтримкою безпечних/шифрованих каналів, хоча це підлягає обмеженням на експорт.

Manufacturers typically default-configure the radios in peer-to-peer mode and match the baud rate expected by PX4 and _QGroundControl_ (57600 baud).
This allows plug and play telemetry when the radios are connected to the usual telemetry ports on a Pixhawk flight controllers (`TELEM1` or `TELEM2`) along with auto-detection of the connection in _QGroundControl_.

Кілька виробників пропонують рішення на основі цих радіопродуктів:

- [ARK Electron Microhard Serial Telemetry Radio](../telemetry/ark_microhard_serial.md)
- [Holybro Microhard P900 Telemetry Radio](../telemetry/holybro_microhard_p900_radio.md)

## Компроміси щодо дальності передачі

Дальність передачі радіо залежить від кількох факторів, включаючи: швидкість передачі даних, вихідну потужність, режим, включено корекцію помилок передачі, включено шифрування, використання антени тощо.

Вибір цих параметрів є компромісом:

- збільшення швидкості передачі даних зменшує дальність радіопередачі.
- збільшення потужності радіо збільшує дальність, але зменшує час польоту.
- точка-багатоточка означає можливість однієї земної станції, яка спілкується з кількома транспортними засобами, але збільшує пропускну спроможність каналу.
- мережеві конфігурації надають подібний зручність і вартість.

Максимальна заявлена дальність в специфікаціях становить приблизно 60 км.
ARK Electron пропонує приблизно 8 км дальності з вихідною потужністю, встановленою на рівні 1 Вт та використанням налаштувань за замовчуванням.

## Налаштування

For convenience, radios are usually default-configured so that they can be used with PX4 and _QGroundControl_ out of the box.

Розробники можуть змінювати конфігурацію.
The only "requirement" is that the: ground radio, air radio, PX4, and _QGroundControl_ must all be set to use the **same** baud rate (and of course each MAVLink system must have a unique System ID).

### Конфігурація PX4

PX4 is configured to use `TELEM1` for telemetry radios, with a default baud rate of 57600.
You can configure PX4 to use any other free serial port a different baud rate, by following the instructions in [MAVLink Peripherals](../peripherals/mavlink_peripherals.md).

### Налаштування QGroundControl

QGroundControl автоматично виявляє послідовне зв'язку телеметрії з швидкістю передачі даних 57600.

Для будь-якого іншого тарифу вам потрібно додати послідовне з'єднання, яке встановлює тариф, який був використаний.
See [Application Settings > Comms Links](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/settings_view.html).

### Конфігурація радіо

Microhard serial radios are configured using the _PicoConfig_ application (Windows only).
This can be downloaded here: [PicoConfig-1.7.zip](https://arkelectron.com/wp-content/uploads/2021/04/PicoConfig-1.7.zip) (ARK Electron) or [picoconfig-1-10](https://docs.holybro.com/telemetry-radio/microhard-radio/download) (Holybro).

У режимах роботи «точка-точка» для забезпечення мережевої синхронізації системи має бути головний пристрій, тому один радіоприймач має бути налаштований на головний PP, а інший — на віддалений PP.

The screen shots below show the default radio configuration settings for connecting to PX4 and _QGroundControl_.

<img src="../../assets/hardware/telemetry/holybro_pico_config.png" width="400px" title="Holybro Pico Config" />
<img src="../../assets/hardware/telemetry/holybro_pico_config1.png" width="400px" title="Holybro Pico Config" />

The [Pico Series P900.Operating Manual.v1.8.7](https://github.com/PX4/PX4-user_guide/raw/main/assets/hardware/telemetry/Pico-Series-P900.Operating-Manual.v1.8.7.pdf) has additional information on radio configuration (including mesh and multipoint modes).

### Режими Mesh та Multipoint

Підтримуються режими Mesh та point to multi-point, але всі транспортні засоби повинні мати унікальний ідентифікатор Mavlink.

Додатково:

- На найвищій швидкості зв'язку, без FEC, ми можемо мати 201 дрона в одній мережі, що передає 80 байт один раз на секунду.
- Ви можете мати кілька мереж, які працюють разом одночасно без взаємного втручання за допомогою "спільно розташованих систем".
  Наприклад, для розгортання більше ніж 500 транспортних засобів вам знадобиться розгорнути три координатори мережі P900, кожен з яких обслуговуватиме до 201 дронів у відповідних локальних мережах.
