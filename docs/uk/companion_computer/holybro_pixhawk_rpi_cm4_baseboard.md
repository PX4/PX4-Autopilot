# Holybro Pixhawk RPi CM4 Baseboard

[Holybro Pixhawk RPi CM4 Baseboard](https://holybro.com/products/pixhawk-rpi-cm4-baseboard) - це одноплатне рішення, яке передбачає попередню інтеграцію (змінну) контролера польоту Pixhawk з супутниковим комп'ютером Raspberry Pi CM4 ("RPi").
Базова плата має компактний форм-фактор з усіма необхідними з'єднаннями для розробки.

![RPi CM4 with Pixhawk](../../assets/companion_computer/holybro_pixhawk_rpi_cm4_baseboard/baseboard_hero.jpg)

Модуль контролера польоту внутрішньо підключений до RPi CM4 через `TELEM2`, але також може бути підключений за допомогою Ethernet за допомогою зовнішнього кабелю, що надається.

Ця базова плата сумісна з [Holybro Pixhawk 5X](../flight_controller/pixhawk5x.md), [Holybro Pixhawk 6X](../flight_controller/pixhawk6x.md) та будь-яким іншим контролером Pixhawk, який відповідає стандартам роз'ємів [Pixhawk Autopilot Bus Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf) для механічної сумісності між виробниками.

:::info
Плата відповідає стандарту роз'ємів [Pixhawk](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) і стандарту [Pixhawk Autopilot Bus](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf) (включаючи вказівки щодо "механічної сумісності між виробниками").
:::

## Купівля

- [Holybro Pixhawk RPi CM4 Baseboard](https://holybro.com/products/pixhawk-rpi-cm4-baseboard) (www.holybro.com)

  Базову плату можна придбати з або без RPi CM4 та/або контролером польоту:

  - Модуль Raspberry Pi CM4 (CM4008032), постачений компанією Holybro, має наступні технічні характеристики:
    - RAM: 8GB
    - eMMC: 32GB
    - Wireless: No
  - Рекомендована мінімальна специфікація для RPi CM4:
    - RAM: 4GB (or 8GB)
    - eMMC: 16GB
    - Wireless: Yes

## Connections & Ports

:::info
[Документація Holybro](https://docs.holybro.com/autopilot/pixhawk-baseboards/pixhawk-rpi-cm4-baseboard/connections-and-ports) містить більше детальної (і можливо більш "актуальної") інформації про порти та з'єднання.
:::

Діаграма нижче показує всі роз'єми та порти на базовій платі.

![Schematic diagram](../../assets/companion_computer/holybro_pixhawk_rpi_cm4_baseboard/baseboard_ports.jpg)

### RPi CM4 та FC Послідовне підключення

Порт керування польотом `TELEM2` внутрішньо підключений до RPi CM4, як показано:

| RPi CM4 | FC TELEM2 (FMU) |
| ------- | ---------------------------------- |
| GPIO14  | TXD                                |
| GPIO15  | RXD                                |
| GPIO16  | CTS                                |
| GPIO17  | RTS                                |

:::info
Підключення також повинно бути [налаштоване як на RPi, так і на PX4](#configure-px4-to-cm4-mavlink-serial-connection) (якщо не використовується [Ethernet](#ethernet-connection-optional)).
:::

## Встановлення Політного Контролера

Плагін-сумісний контролер польоту, такий як [Holybro Pixhawk 5X](../flight_controller/pixhawk5x.md) та [Holybro Pixhawk 6X](../flight_controller/pixhawk6x.md), може просто бути вставлений у роз'єм модуля.

Контролери польоту, які мають інший форм-фактор, будуть потребувати додаткового підключення проводів.

## Встановлення супутника RPi CM4

This section shows how to install/attach an RPi CM4 to the baseboard.

![Image showing separate baseboard, baseboard cover, RPi, Flight controller, screws](../../assets/companion_computer/holybro_pixhawk_rpi_cm4_baseboard/baseboard_disassembled.jpg)

To install the RPi CM4 companion computer:

1. Disconnect the `FAN` wiring.

  ![HB_Pixhawk_CM4_Fan](../../assets/companion_computer/holybro_pixhawk_rpi_cm4_baseboard/baseboard_fan.jpg)

2. Видаліть ці 4 гвинти на задній стороні підлогової дошки.

  ![Bottom of the board showing screws in corners holding the cover](../../assets/companion_computer/holybro_pixhawk_rpi_cm4_baseboard/baseboard_bottom.jpg)

3. Видаліть підставку корпусу, встановіть CM4 та використовуйте 4 гвинти для його кріплення (як показано):

  ![HB_Pixhawk_CM4_Screws](../../assets/companion_computer/holybro_pixhawk_rpi_cm4_baseboard/baseboard_screws.jpg)

4. Прикріпіть кришку знову.

## Проводка силового модуля

Модуль живлення PM03D постачається разом з платою.

RPi CM4 та контролер польоту повинні бути живлені окремо:

- Контролер польоту живиться через кабель CLIK-Mate до порту `POWER1` або порту `POWER2`
- RPi CM4 працює від підключення `USB C` (CM4 Slave).
  Ви також можете використовувати власний блок живлення для живлення базової плати RPi CM4.

На зображенні нижче показана проводка більш детально.

![Image showing writing from the PM03D power module to the baseboard](../../assets/companion_computer/holybro_pixhawk_rpi_cm4_baseboard/baseboard_wiring_guide.jpg)

## Прошивка RPi CM4

Цей розділ пояснює, як встановити вашу улюблену дистрибутив Linux, таку як "Raspberry Pi OS 64bit", на RPi EMCC.

Примітки:

- Якщо ви використовуєте PX4, вам потрібно використовувати версію PX4 1.13.1 або новішу, щоб PX4 впізнав цю базову плату.
- Вентилятор не показує, чи живиться/працює RPi CM4.
- Модуль живлення, вставлений у Power1/2, не живить частину RPi.
  Ви можете використовувати додатковий кабель USB-C від модуля живлення PM03D до порту USB-C CM4 Slave.
- Порт Micro-HDMI є вихідним портом.
- RPi CM4 boards that do not have WiFi device will not connect automatically.
  In this case you will need to plug it into a router or plug a compatible WiFi dongle into the CM4 Host ports.

### Flash EMMC

Записати образ RPi на EMMC.

1. Switch Dip-Switch to `RPI`.

  ![](../../assets/companion_computer/holybro_pixhawk_rpi_cm4_baseboard/cm4_dip_switch.png)

2. Підключіть комп'ютер до порту USB-C _CM4 Slave_, що використовується для живлення та прошивки RPi.

  ![](../../assets/companion_computer/holybro_pixhawk_rpi_cm4_baseboard/cm4_usbc_slave_port.png)

3. Отримайте `usbboot`, зберіть його та запустіть.

  ```sh
  sudo apt install libusb-1.0-0-dev
  git clone --depth=1 https://github.com/raspberrypi/usbboot
  cd usbboot
  make
  sudo ./rpiboot
  ```

4. Тепер ви можете встановити свою перевагу Linux дистрибутив за допомогою `rpi-imager`.
  Переконайтеся, що ви додали налаштування WiFi та SSH (приховані за символом шестерні / розширеним).

  ```sh
  sudo apt install rpi-imager
  rpi-imager
  ```

5. Після завершення відключення USB-C CM4 Slave (це відмонтує томи та вимкне CM4).

6. Перемикач Dip-Switch поверніть на `EMMC`.

7. Увімкніть CM4, надаючи живлення через порт USB-C CM4 Slave.

8. Щоб перевірити, чи запускається/працює, ви можете або:
  - Перевірте, чи є вихід HDMI
  - Підключіться через SSH (якщо налаштовано в rpi-imager, і є доступ до WiFi).

## Налаштуйте послідовне підключення PX4 до CM4 MAVLink

:::info
Якщо ви використовуєте [Ethernet](#ethernet-connection-optional) для підключення FC та RPi, ця настройка не потрібна.
:::

Модуль Pixhawk FC [внутрішньо підключено до RPi CM4](#rpi-cm4-fc-serial-connection) за допомогою `TELEM2` (`/dev/ttyS4`).
FC та RPi CM4 повинні бути налаштовані для зв'язку через цей порт.

### Налаштування послідовного порту FC

FC повинен бути налаштований для підключення до порту `TELEM2` правильно за замовчуванням.
Якщо ні, ви можете налаштувати порт за допомогою параметрів, як показано.

Для активації цього екземпляру MAVLink на FC:

1. Підключіть комп'ютер, на якому працює QGroundControl, через порт USB Type C на базовій платі, позначеній як `FC`

  ![Image of baseboard showing FC USB-C connector](../../assets/companion_computer/holybro_pixhawk_rpi_cm4_baseboard/baseboard_fc_usb_c.jpg)

2. [Встановіть параметри](../advanced_config/parameters.md):

  - `MAV_1_CONFIG` = `102`
  - `MAV_1_MODE = 2`
  - `SER_TEL2_BAUD` = `921600`

3. Перезавантажте FC.

### Налаштування послідовного порту RPi

На стороні RPi:

1. Connect to the RPi (using WiFi, a router, or a WiFi Dongle).

2. Увімкніть послідовний порт RPi, запустивши `RPi-config`

  - Перейдіть до `3 Варіанти інтерфейсу`, потім `I6 Серійний порт`.
    Потім введіть:
    - `login shell accessible over serial → No`
    - `serial port hardware enabled` → `Yes`

3. Завершіть і перезавантажте.
  This will add `enable_uart=1` to `/boot/config.txt`, and remove `console=serial0,115200` from `/boot/cmdline.txt`.

4. Тепер MAVLink-трафік повинен бути доступний на `/dev/serial0` з швидкістю передачі даних 921600.

## Спробуйте MAVSDK-Python

1. Make sure the CM4 is connected to the internet, e.g. using a WiFi, or Ethernet.

2. Встановіть MAVSDK Python:

  ```sh
  python3 -m pip install mavsdk
  ```

3. Скопіюйте приклад з [прикладів MAVSDK-Python](https://github.com/mavlink/MAVSDK-Python/tree/main/examples).

4. Змініть `system_address="udp://:14540"` на `system_address="serial:///dev/serial0:921600"`

5. Спробуйте приклад. Permission for the serial port should already be available through the `dialout` group.

## Підключення Ethernet (Необов'язково)

The flight controller module is [internally connected to RPi CM4](#rpi-cm4-fc-serial-connection) from `TELEM2` (serial).

Ви також можете налаштувати локальне підключення Ethernet між ними за допомогою постачаного кабелю.
Підключення через Ethernet надає швидкий, надійний та гнучкий спосіб зв'язку, який може бути альтернативою використанню USB або інших послідовних з'єднань.

:::info
For general Ethernet setup information see: [PX4 Ethernet Setup](../advanced_config/ethernet_setup.md).

The setup here is much the same, except that we have used the following `netplan` config on PX4:

```sh
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      addresses:
        - 10.41.10.1/24
      nameservers:
        addresses: [10.41.10.1]
      routes:
        - to: 10.41.10.0/24  # Local route to access devices on this subnet
          scope: link        # Scope link to restrict it to local subnet
```

This sets `eth0` as our channel for the local Ethernet link from the RPi (instead of `enp2s0`, which is assumed in [Ethernet Setup](../advanced_config/ethernet_setup.md#ubuntu-ethernet-network-setup)).

Note that we could have used WiFi for the link, but by setting up a dedicated route we leave our WiFi free for Internet comms.
:::

### Підключіть кабель

To set up a local ethernet connection between CM4 and the flight computer, the two Ethernet ports need to be connected using the provided 8 pin to 4 pin connector.

![HB_Pixhawk_CM4_Ethernet_Cable](../../assets/companion_computer/holybro_pixhawk_rpi_cm4_baseboard/baseboard_ethernet_cable.png)

Схема виводів кабелю:

| CM4 Eth 8 Pin | FC ETH 4 Pin |
| ------------- | ------------ |
| A             | B            |
| B             | A            |
| C             | D            |
| D             | C            |
| -             | N/A          |
| -             | N/A          |
| -             | N/A          |
| -             | N/A          |

### Налаштування IP на CM4

Оскільки в цій конфігурації відсутній активний DHCP-сервер, IP-адреси повинні бути встановлені вручну:

First, connect to the CM4 via SSH by connecting to the CM4's WiFi (or use a WiFi dongle).
Once the Ethernet cables are plugged in, the `eth0` network interface seems to switch from DOWN to UP.

Ви можете перевірити статус за допомогою:

```sh
ip address show eth0
```

Ви також можете спробувати увімкнути його вручну:

```sh
sudo ip link set dev eth0 up
```

This sets a link-local address.
For this example it looks like this:

```sh
$: ip address show eth0
2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc mq state UP group default qlen 1000
    link/ether e4:5f:01:bf:e0:17 brd ff:ff:ff:ff:ff:ff
    inet 10.41.10.1/24 brd 10.41.10.255 scope global noprefixroute eth0
       valid_lft forever preferred_lft forever
    inet6 fe80::e65f:1ff:febf:e017/64 scope link
       valid_lft forever preferred_lft forever
```

This means the CM4's Ethernet IP is `10.41.10.1` .

#### Тест пінгу

First ping PX4 from the CM4 (using the PX4's default address):

```sh
$ ping 10.41.10.2
```

Should give something like:

```sh
PING 10.41.10.2 (10.41.10.2) 56(84) bytes of data.
64 bytes from 10.41.10.2: icmp_seq=1 ttl=64 time=0.187 ms
64 bytes from 10.41.10.2: icmp_seq=2 ttl=64 time=0.109 ms
64 bytes from 10.41.10.2: icmp_seq=3 ttl=64 time=0.091 ms
^C
--- 10.41.10.2 ping statistics ---
3 packets transmitted, 3 received, 0% packet loss, time 2049ms
rtt min/avg/max/mdev = 0.091/0.129/0.187/0.041 ms
```

:::info
If this step fails, check if a [firewall](https://wiki.ubuntu.com/UncomplicatedFirewall) is active.
:::

Then ping the CM4 from the flight controlle.
Enter the following command in the Nuttx Shell:

```sh
nsh> ping 10.41.10.1
```

This should result in output like:

```sh
PING 10.41.10.1 56 bytes of data
56 bytes from 10.41.10.1: icmp_seq=0 time=0.0 ms
56 bytes from 10.41.10.1: icmp_seq=1 time=0.0 ms
56 bytes from 10.41.10.1: icmp_seq=2 time=0.0 ms
56 bytes from 10.41.10.1: icmp_seq=3 time=0.0 ms
56 bytes from 10.41.10.1: icmp_seq=4 time=0.0 ms
56 bytes from 10.41.10.1: icmp_seq=5 time=0.0 ms
56 bytes from 10.41.10.1: icmp_seq=6 time=0.0 ms
56 bytes from 10.41.10.1: icmp_seq=7 time=0.0 ms
56 bytes from 10.41.10.1: icmp_seq=8 time=0.0 ms
56 bytes from 10.41.10.1: icmp_seq=9 time=0.0 ms
10 packets transmitted, 10 received, 0% packet loss, time 10010 ms
rtt min/avg/max/mdev = 0.000/0.000/0.000/0.000 ms
```

#### MAVLink/MAVSDK Test

Для цього нам потрібно встановити екземпляр MAVLink для відправлення трафіку на IP-адресу CM4:

Для початкового тесту ми можемо зробити:

```sh
mavlink start -o 14540 -t 10.41.10.1
```

Це буде відправляти трафік MAVLink по UDP на порт 14540 (порт MAVSDK/MAVROS) на цей IP, що означає, що MAVSDK може просто слухати будь-який UDP, що надходить на цей типовий порт.

Для запуску прикладу MAVSDK встановіть mavsdk через pip і спробуйте приклад з [MAVSDK-Python/examples](https://github.com/mavlink/MAVSDK-Python/tree/main/examples).

#### XRCE-Client Ethernet Setup

Next we enable `XRCE-DDS` on the new Ethernet Link.

You can [modify the required parameters](../advanced_config/parameters.md) in QGroundControl parameter editor, or using `param set` in the [MAVLINK shell](../debug/mavlink_shell.md).
Below we show the settings assuming you're setting the parameters using the shell.

First ensure `MAV_2_CONFIG` is not set to use the Ethernet port (`1000`) as this would clash with XRCE-DDS (see [enable MAVLINK on Ethernet](../advanced_config/ethernet_setup.md#px4-mavlink-serial-port-configuration)):

```sh
nsh>
param set MAV_2_CONFIG     0           # Change to 0 IFF value is 1000
```

Then enable uXRCE-DDS on the Ethernet port (see [starting uXRCE-DDS client](../middleware/uxrce_dds.md#starting-the-client)):

```sh
param set UXRCE_DDS_AG_IP  170461697   # The int32 version of 10.41.10.1
param set UXRCE_DDS_CFG    1000        # Set Serial Configuration for uXRCE-DDS Client to Ethernet
param set UXRCE_DDS_DOM_ID 0           # Set uXRCE-DDS domain ID
param set UXRCE_DDS_KEY    1           # Set uXRCE-DDS session key
param set UXRCE_DDS_PRT    8888        # Set uXRCE-DDS UDP port
param set UXRCE_DDS_PTCFG  0           # Set uXRCE-DDS participant configuration
param set UXRCE_DDS_SYNCC  0           # Disable uXRCE-DDS system clock synchronization
param set UXRCE_DDS_SYNCT  1           # Enable uXRCE-DDS timestamp synchronization
```

Then run the Agent:

```sh
MicroXRCEAgent udp4 -p 8888
```

And such output is expected if everything is set up correctly:

```sh
[1731210063.537033] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 8888
[1731210063.538279] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
[1731210066.577413] info     | Root.cpp           | create_client            | create                 | client_key: 0x00000001, session_id: 0x81
[1731210066.577515] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x00000001, address: 10.41.10.2:58900
[1731210066.583965] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x00000001, participant_id: 0x001(1)
[1731210066.584754] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x00000001, topic_id: 0x800(2), participant_id: 0x001(1)
[1731210066.584988] info     | ProxyClient.cpp    | create_subscriber        | subscriber created     | client_key: 0x00000001, subscriber_id: 0x800(4), participant_id: 0x001(1)
[1731210066.589864] info     | ProxyClient.cpp    | create_datareader        | datareader created     | client_key: 0x00000001, datareader_id: 0x800(6), subscriber_id: 0x800(4)
[1731210066.591007] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x00000001, topic_id: 0x801(2), participant_id: 0x001(1)
[1731210066.591164] info     | ProxyClient.cpp    | create_subscriber        | subscriber created     | client_key: 0x00000001, subscriber_id: 0x801(4), participant_id: 0x001(1)
[1731210066.591912] info     | ProxyClient.cpp    | create_datareader        | datareader created     | client_key: 0x00000001, datareader_id: 0x801(6), subscriber_id: 0x801(4)
[1731210066.592701] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x00000001, topic_id: 0x802(2), participant_id: 0x001(1)
[1731210066.592846] info     | ProxyClient.cpp    | create_subscriber        | subscriber created     | client_key: 0x00000001, subscriber_id: 0x802(4), participant_id: 0x001(1)
[1731210066.593640] info     | ProxyClient.cpp    | create_datareader        | datareader created     | client_key: 0x00000001, datareader_id: 0x802(6), subscriber_id: 0x802(4)
[1731210066.594749] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x00000001, topic_id: 0x803(2), participant_id: 0x001(1)
[1731210066.594883] info     | ProxyClient.cpp    | create_subscriber        | subscriber created     | client_key: 0x00000001, subscriber_id: 0x803(4), participant_id: 0x001(1)
[1731210066.595592] info     | ProxyClient.cpp    | create_datareader        | datareader created     | client_key: 0x00000001, datareader_id: 0x803(6), subscriber_id: 0x803(4)
[1731210066.596188] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x00000001, topic_id: 0x804(2), participant_id: 0x001(1)
[1731210066.596334] info     | ProxyClient.cpp    | create_subscriber        | subscriber created     | client_key: 0x00000001, subscriber_id: 0x804(4), participant_id: 0x001(1)
[1731210066.597046] info     | ProxyClient.cpp    | create_datareader        | datareader created     | client_key: 0x00000001, datareader_id: 0x804(6), subscriber_id: 0x804(4)
```

## Дивіться також

- [Отримати базову плату Pixhawk Raspberry Pi CM4 від Holybro, яка спілкується з PX4](https://px4.io/get-the-pixhawk-raspberry-pi-cm4-baseboard-by-holybro-talking-with-px4/) (блог px4.io):
  - Урок, який показує, як підключити Pixhawk 6X + Raspberry Pi на базі CM4 через провідний Ethernet.
