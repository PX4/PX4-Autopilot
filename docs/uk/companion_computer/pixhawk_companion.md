# Використання комп'ютера-компаньйона з контролерами Pixhawk

PX4, що працює на контролерах польоту серії Pixhawk, може підключатися до супутнього комп'ютера за допомогою будь-якого вільного конфігурованого послідовного порту, включаючи Ethernet-порт (якщо підтримується).

[Супутникові комп'ютери](../companion_computer/Readme.md) для інформації про підтримуваний обладнання та загальну настройку.

## Програмне забезпечення супутнього комп'ютера

На супутньому комп'ютері має бути встановлене програмне забезпечення, яке зв'язується з диспетчером польоту і спрямовує трафік на наземні станції та в хмару.

Common options are listed in [Companion Computers > Companion Computer Setup](../companion_computer/index.md#companion-computer-software).

## Налаштування Ethernet

Ethernet - рекомендоване з'єднання, якщо воно підтримується вашим польотним контролером.
У розділі [Налаштування Ethernet](../advanced_config/ethernet_setup.md).

## Налаштування послідовного порту

Ці інструкції пояснюють, як налаштувати з'єднання, якщо ви не використовуєте Ethernet.

### Конфігурація Pixhawk

PX4 очікує, що супутні комп'ютери будуть підключатися через `TELEM2 `для віддаленого керування. Порт конфігурується за замовчуванням для інтерфейсу за допомогою MAVLink.
Порт конфігурується за замовчуванням для інтерфейсу за допомогою MAVLink.

Якщо використовується MAVLink, іншої конфігурації з боку PX4 не потрібно.
Щоб використовувати MAVLink на іншому порту і/або вимкнути його на `TELEM2`, див. [Периферійні пристрої MAVLink (GCS/OSD/Companion)](../peripherals/mavlink_peripherals.md) та [Налаштування послідовного порту](../peripherals/serial_configuration.md).

To use [ROS 2/uXRCE-DDS](../ros2/user_guide.md) instead of MAVLink on `TELEM2`, disable MAVLink on the port and then enable the uXRCE-DDS client on `TELEM2`(see [uXRCE-DDS > Starting the client](../middleware/uxrce_dds.md#starting-the-client)).

### Налаштування апаратної частини послідовного порту

Якщо ви підключаєтеся за допомогою послідовного порту, підключіть порт згідно з інструкціями нижче.
Всі послідовні порти Pixhawk працюють на напрузі 3,3 В і сумісні з рівнем 5 В.

:::warning
Багато сучасних супутникових комп'ютерів підтримують лише рівні 1,8 В на їх апаратному UART і можуть бути пошкоджені рівнями 3,3 В.
Використовуйте рівні перетворювачі.
У більшості випадків до доступних апаратних послідовних портів вже призначена певна функція (модем або консоль), яка повинна бути _переконфігурована в Linux_ перед їх використанням.
:::

Безпечним і легким у налаштуванні варіантом є використання плати адаптера USB-послідовного порту від FTDI Chip для підключення від `TELEM2` на Pixhawk до USB-порту на супутниковому комп'ютері.
Нижче наведено зв'язку карту `TELEM2` до FTDI.

| TELEM2 |                                | FTDI | &nbsp;                                   |
| ------ | ------------------------------ | ---- | ------------------------------------------------------------ |
| 1      | +5V (red)   |      | DO NOT CONNECT!                                              |
| 2      | Tx (out)    | 5    | FTDI RX (yellow) (in)  |
| 3      | Rx (in)     | 4    | FTDI TX (orange) (out) |
| 4      | CTS (вхід)  | 6    | FTDI RTS (green) (out) |
| 5      | RTS (вивід) | 2    | FTDI CTS (brown) (in)  |
| 6      | GND                            | 1    | FTDI GND (black)                          |

Ви також можете безпосередньо підключити `TELEM2` безпосередньо до послідовного порту супутнього комп'ютера.
Це показано для Raspberry Pi в [Raspberry Pi Companion with Pixhawk](../companion_computer/pixhawk_rpi.md).

### Налаштування програмного забезпечення для USB-послідовного порту на Linux

У Linux за замовчуванням ім'я USB-послідовного порту буде таким, як `/dev/ttyUSB0`.
Якщо у вас є другий FTDI, підключений через USB, або Arduino, він буде зареєстрований як `/dev/ttyUSB1`.
Щоб уникнути плутанини між першим підключеним і другим підключеним пристроєм, ми рекомендуємо створити символічне посилання з `ttyUSBx` на дружнє ім'я, залежно від ідентифікатора виробника та продукту (Product ID) USB-пристрою.

За допомогою команди`lsusb` ми можемо отримати ідентифікатори виробника та продукту.

```sh
$ lsusb

Bus 006 Device 002: ID 0bda:8153 Realtek Semiconductor Corp.
Bus 006 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 005 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 004 Device 002: ID 05e3:0616 Genesys Logic, Inc.
Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 003 Device 004: ID 2341:0042 Arduino SA Mega 2560 R3 (CDC ACM)
Bus 003 Device 005: ID 26ac:0011
Bus 003 Device 002: ID 05e3:0610 Genesys Logic, Inc. 4-port hub
Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 001: ID 1d6b:0001 Linux Foundation 1.1 root hub
Bus 001 Device 002: ID 0bda:8176 Realtek Semiconductor Corp. RTL8188CUS 802.11n WLAN Adapter
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```

The Arduino is `Bus 003 Device 004: ID 2341:0042 Arduino SA Mega 2560 R3 (CDC ACM)`

The Pixhawk is `Bus 003 Device 005: ID 26ac:0011`

:::info
Якщо ви не знайшли свій пристрій, відключіть його, виконайте `lsusb`, підключіть його, знову виконайте `lsusb` і перегляньте доданий пристрій.
:::

Тому ми можемо створити нове правило UDEV у файлі під назвою `/etc/udev/rules.d/99-pixhawk.rules` із таким вмістом, змінивши idVendor та idProduct на ваші.

```sh
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="ttyArduino"
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", SYMLINK+="ttyPixhawk"
```

Нарешті, після **перезавантаження** ви можете бути впевнені, що знаєте, який пристрій є яким, і вставте `/dev/ttyPixhawk` замість `/dev/ttyUSB0` у свій сценарії.

:::info
Обов’язково додайте себе до груп `tty` і `dialout` через `usermod`, щоб уникнути необхідності виконувати сценарії від імені користувача root.
:::

```sh
usermod -a -G tty ros-user
usermod -a -G dialout ros-user
```
