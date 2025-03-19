# JLink Debug Probe

The [J-Link debug probe][jlink] is a closed-source, commercial hardware probe which supports almost all ARM Cortex-M devices.
You need to install the [J-Link drivers][drivers] for this probe to work:

```sh
# Ubuntu
wget --post-data "accept_license_agreement=accepted" https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.deb
sudo dpkg -i JLink_Linux_x86_64.deb
# macOS
brew install segger-jlink
```

Після встановлення ви можете запустити сервер за допомогою:

```sh
JLinkGDBServer -if swd -device STM32F765II
```

Можливо, перед вами відобразиться запит на оновлення JLink, що й рекомендується зробити, а потім вказати, з яким пристроєм він взаємодіє.
Перевірте документацію вашого автопілота для конкретного пристрою.

Once that's done, the GDB server should be start listening on port `2331`, e.g. like so:

```sh
Checking target voltage...
Target voltage: 3.28 V
Listening on TCP/IP port 2331
Connecting to target...
Connected to target
Waiting for GDB connection...
```

Тепер ви можете запустити GDB з точним elf-файлом, який зараз вивантажений на автопілот (у окремому терміналі):

```sh
arm-none-eabi-gdb build/px4_fmu-v5x_default/px4_fmu-v5x_default.elf -ex "target extended-remote :2331"
```

Тепер ви повинні бути підключені.

To use an IDE instead, see the instructions for [Eclipse](../debug/eclipse_jlink.md) or [VSCode](../dev_setup/vscode.md#hardware-debugging).
See the [Embedded Debug Tools][emdbg] for more advanced debug options.

<a id="segger_jlink_edu_mini"></a>

### Embedded Debug Tools

The [Segger JLink EDU Mini](https://www.segger.com/products/debug-probes/j-link/models/j-link-edu-mini/) is an inexpensive and popular SWD debug probe.
The probe's connector pinout looks like the image below (connect to this using an ARM 10-pin mini connector like [FTSH-105-01-F-DV-K](https://www.digikey.com/products/en?keywords=SAM8796-ND)).

![connector_jlink_mini.png](../../assets/debug/connector_jlink_mini.png)

The pin mapping to connect the J-Link Edu Mini to [Pixhawk Debug Mini](swd_debug.md#pixhawk-debug-mini) is shown below.

| Pin | Сигнал     | JLink |
| --: | :--------- | ----: |
|   1 | **VREF**   |     1 |
|   2 | Console TX |       |
|   3 | Console RX |       |
|   4 | **SWDIO**  |     2 |
|   5 | **SWDCLK** |     4 |
|   6 | **GND**    |  3, 5 |

Зверніть увагу, що жоден з JLink debug зондів не має вбудованого послідовного з'єднання, тому вам потрібно підключити консоль окремо.

<!-- Image of SWD cable and connector to debug port - proposed? -->

[jlink]: https://www.segger.com/products/debug-probes/j-link/
[drivers]: https://www.segger.com/downloads/jlink/
[emdbg]: https://pypi.org/project/emdbg/
