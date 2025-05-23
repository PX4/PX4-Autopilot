# ARK Electron Microhard Серійне Телеметрійне Радіо

_ARK Electron Microhard Serial Telemetry Radios_ integrate the [Microhard Pico Serial](http://microhardcorp.com/P900.php) P900 RF module.
Це може бути використано для активації комунікації MAVLink між радіо на транспортному засобі та GCS.

Радіостанції Microhard Pico Serial - це радіостанції з вихідною потужністю (до) 1 Вт, які підтримують режими точка-точка, точка-багато точок та мережеві режими.
Радіопристрої Microhard Pico також можуть бути замовлені з шифруванням AES-256.

Приблизний діапазон з вихідною потужністю, встановленою на рівні 1 Вт, становить 8 км (5 миль), коли використовуються налаштування за замовчуванням.
Одне радіо земної станції може бути використано для зв'язку з кількома транспортними засобами за допомогою точка-багатоточної або мережевої топології меш.
Транспортні засоби повинні мати різні ідентифікатори MAVLINK.

![Microhard Radio](../../assets/hardware/telemetry/ark_microhard_serial.jpg)

## Де купити

- [1W 900MHz Serial Telemetry Radio](https://arkelectron.com/product/1w-900mhz-serial-telemetry-air-radio/) (vehicle)
- [1W 900MHz USB Serial Telemetry Radio](https://arkelectron.com/product/1w-900mhz-serial-telemetry-ground-radio/) (ground station)
- [1W 2.4GHz Serial Telemetry Radio](https://arkelectron.com/product/1w-2400mhz-serial-telemetry-radio/) (vehicle)
- [1W 2.4GHz USB Serial Telemetry Radio](https://arkelectron.com/product/1w-2400mhz-usb-serial-telemetry-radio/) (ground station)

## З'єднання

### Транспортний радіопередавач

Connect the vehicle radio to the flight controller `TELEM1` port.
Для цієї мети надається кабель телеметрії з 6 контактами JST GH стандарту Pixhawk.

Радіо може бути живлений за допомогою телеметричного кабелю, якщо вихідна потужність встановлена менше 100 мВт.
Для вищих рівнів виводу радіо повинно бути окремо живлене через 2-контактний роз'єм Molex Nano-Fit (тобто від батареї).

![Microhard Radio on Vehicle](../../assets/hardware/telemetry/microhard_serial_on_vehicle.jpg)

### Радіо наземної станції

Підключіть земельне радіо до земельної станції за допомогою USB C.
Радіо не потребує окремого живлення при використанні USB PD (може бути постачана потужність 1 Вт).

## Налаштування/Конфігурація

Радіостанції налаштовані за замовчуванням на використання режиму peer-to-peer та швидкість передачі 57600 бод.
This allows them to connect to the PX4 `TELEM1` port and _QGroundControl_ **without any further configuration**.

:::info
You can use a different baud rate, mode or flight controller port.
Єдине "вимога" - земний радіопередавач, повітряний радіопередавач, PX4 та QGroundControl повинні мати однакову швидкість передачі даних.
:::

[Microhard Serial Telemetry Radios > Configuration](../telemetry/microhard_serial.md#configuration) explains how to configure the radios, _QGroundControl_, and PX4.

The ARK Electron radios must be connected to the computer running the _PicoConfig_ configuration tool as described below:

- For vehicle radio configuration you will have to connect an FTDI adapter between the radio's 3 pin JST-GH Config port and a Windows PC running _Pico Config_ (the radio must be powered, which you can do from battery or the data connection to the flight-controller's `TELEM1` port).

  ![Ark Microhard Serial - Ports](../../assets/hardware/telemetry/ark_microhard_serial_ports.jpg)

  _Pico Config_ will automatically detect the radio.
  Налаштуйте швидкість передачі даних (бод-швидкість) так, щоб вона відповідала PX4 (і радіо земної станції).

- З'єднання USB C радіостанції наземної станції може бути використане для налаштування радіо (а також для телеметричних даних).
  _Pico Config_ will automatically detect and connect to the configuration port.
  Налаштуйте параметри так, щоб швидкість передачі даних відповідала PX4.

Як тільки радіостанції та PX4 будуть налаштовані на використання одного темпу передачі, ви зможете підключити QGroundControl до транспортного засобу через радіо.

### Конфігурація за замовчуванням

The default radio configuration as shipped is shown in _PicoConfig_ below.

![Pico Config](../../assets/hardware/telemetry/pico_configurator.png)

## Подальша інформація

- [Pico Config 1.7](https://arkelectron.com/wp-content/uploads/2021/04/PicoConfig-1.7.zip) - Radio configuration tool
