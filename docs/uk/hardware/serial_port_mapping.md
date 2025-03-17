# Налаштування послідовного порту

This topic shows how to determine the mapping between USART/UART serial port device names (e.g. "ttyS0") and the associated ports on a flight controller, such as `TELEM1`, `TELEM2`, `GPS1`, `RC SBUS`, `Debug console`.

Інструкції використовуються для генерації таблиць відповідності портів серійного зв'язку у документації контролера польоту.
For example: [Pixhawk 4 > Serial Port Mapping](../flight_controller/pixhawk4.md#serial-port-mapping).

:::info
The function assigned to each port does not _have to_ match the name (in most cases), and is set using a [Serial Port Configuration](../peripherals/serial_configuration.md).
Usually the port function is configured to match the name, which is why the port labelled `GPS1` will work with a GPS out of the box.
:::

## NuttX на STMxxyyy

<!-- instructions from DavidS here: https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

У цьому розділі показано, як отримати зіставлення для збірок NuttX на архітектурах STMxxyyy, перевіряючи файли конфігурації плати.
Інструкції використовують FMUv5, але можуть бути аналогічно розширені для інших версій FMU або плат NuttX.

### default.px4board

The **default.px4board** lists a number of serial port mappings (search for the text "SERIAL_PORTS").

From [/boards/px4/fmu-v5/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board):

```
CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS0"
CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS1"
CONFIG_BOARD_SERIAL_TEL2="/dev/ttyS2"
CONFIG_BOARD_SERIAL_TEL4="/dev/ttyS3"
```

Alternatively you can launch boardconfig using `make px4_fmu-v5 boardconfig` and access the serial port menu

```
    Serial ports  --->
        (/dev/ttyS0) GPS1 tty port
        ()  GPS2 tty port
        ()  GPS3 tty port
        ()  GPS4 tty port
        ()  GPS5 tty port
        (/dev/ttyS1) TEL1 tty port
        (/dev/ttyS2) TEL2 tty port
        ()  TEL3 tty port
        (/dev/ttyS3) TEL4 tty port
        ()  TEL5 tty port
```

### nsh/defconfig

The _nsh/defconfig_ allows you to determine which ports are defined, whether they are UART or USARTs, and the mapping between USART/UART and device.
You can also determine which port is used for the [serial/debug console](../debug/system_console.md).

Open the board's defconfig file, for example: [/boards/px4/fmu-v5/nuttx-config/nsh/defconfig](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/nuttx-config/nsh/defconfig#L215-L221)

Search for the text "ART" until you find a section like with entries formatted like `CONFIG_STM32xx_USARTn=y` (where `xx` is a processor type and `n` is a port number).
Наприклад:

```
CONFIG_STM32F7_UART4=y
CONFIG_STM32F7_UART7=y
CONFIG_STM32F7_UART8=y
CONFIG_STM32F7_USART1=y
CONFIG_STM32F7_USART2=y
CONFIG_STM32F7_USART3=y
CONFIG_STM32F7_USART6=y
```

Записи показують, які порти визначено, і чи вони є UART або USART.

Скопіюйте вищенаведений розділ і перегрупуйте його числово за "n".
Increment the device number _ttyS**n**_ alongside (zero based) to get the device-to-serial-port mapping.

```
ttyS0 CONFIG_STM32F7_USART1=y
ttyS1 CONFIG_STM32F7_USART2=y
ttyS2 CONFIG_STM32F7_USART3=y
ttyS3 CONFIG_STM32F7_UART4=y
ttyS4 CONFIG_STM32F7_USART6=y
ttyS5 CONFIG_STM32F7_UART7=y
ttyS6 CONFIG_STM32F7_UART8=y
```

To get the DEBUG console mapping we search the [defconfig file](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/nuttx-config/nsh/defconfig#L212) for `SERIAL_CONSOLE`.
Нижче ми бачимо, що консоль знаходиться на UART7:

```
CONFIG_UART7_SERIAL_CONSOLE=y
```

### board_config.h

For flight controllers that have an IO board, determine the PX4IO connection from **board_config.h** by searching for `PX4IO_SERIAL_DEVICE`.

For example, [/boards/px4/fmu-v5/src/board_config.h](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/src/board_config.h#L59):

```
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS6"
#define PX4IO_SERIAL_TX_GPIO           GPIO_UART8_TX
#define PX4IO_SERIAL_RX_GPIO           GPIO_UART8_RX
#define PX4IO_SERIAL_BASE              STM32_UART8_BASE
```

So the PX4IO is on `ttyS6` (we can also see that this maps to UART8, which we already knew from the preceding section).

### Підсумовуючи все це разом,

Остаточне відображення таке:

```
ttyS0 CONFIG_STM32F7_USART1=y GPS1
ttyS1 CONFIG_STM32F7_USART2=y TEL1
ttyS2 CONFIG_STM32F7_USART3=y TEL2
ttyS3 CONFIG_STM32F7_UART4=y TEL4
ttyS4 CONFIG_STM32F7_USART6=y
ttyS5 CONFIG_STM32F7_UART7=y DEBUG
ttyS6 CONFIG_STM32F7_UART8=y PX4IO
```

In the [flight controller docs](../flight_controller/pixhawk4.md#serial-port-mapping) the resulting table is:

| UART   | Пристрій   | Порт                                          |
| ------ | ---------- | --------------------------------------------- |
| UART1  | /dev/ttyS0 | GPS                                           |
| USART2 | /dev/ttyS1 | TELEM1 (керування потоком) |
| USART3 | /dev/ttyS2 | TELEM2 (керування потоком) |
| UART4  | /dev/ttyS3 | TELEM4                                        |
| USART6 | /dev/ttyS4 | RC SBUS                                       |
| UART7  | /dev/ttyS5 | Debug Console                                 |
| UART8  | /dev/ttyS6 | PX4IO                                         |

## Інші архітектури

:::info
Contributions welcome!
:::

## Дивіться також

- [Serial Port Configuration](../peripherals/serial_configuration.md)
- [MAVLink Telemetry (OSD/GCS)](../peripherals/mavlink_peripherals.md)
