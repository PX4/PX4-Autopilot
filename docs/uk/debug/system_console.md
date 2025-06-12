# Системна консоль PX4

The PX4 _System Console_ provides low-level access to the system, debug output and analysis of the system boot process.

:::tip
The console should be used for debugging if the system won't boot.
The [MAVLink Shell](../debug/mavlink_shell.md) may otherwise be more suitable, as it is much easier to set up and can be used for [many of the same tasks](../debug/consoles.md#console_vs_shell).
:::

## Підключення консолі

The console is made available through a (board-specific) UART that can be connected to a computer USB port using a [3.3V FTDI](https://www.digikey.com/en/products/detail/TTL-232R-3V3/768-1015-ND/1836393) cable.
Це дозволяє доступ до консолі за допомогою термінальної програми.

Pixhawk controller manufacturers are expected to expose the console UART and SWD (JTAG) debug interfaces through a dedicated _debug port_ that complies with the [Pixhawk Connector Standard](#pixhawk_debug_port).
На жаль, деякі дошки попередні цьому стандарту або не відповідають йому.

:::info
Developers targeting a number of different boards may wish to use a [debug adapter](../debug/swd_debug.md#debug-adapters) to simplify connecting boards to FTDI cables and [debug probes](../debug/swd_debug.md#debug-probes-for-px4-hardware).
:::

Розділи нижче наводять посилання на інформацію про проводку та консоль системи для багатьох загальних дошок.

### Проводка, специфічна для плати

The System Console UART pinouts/debug ports are typically documented in [autopilot overview pages](../flight_controller/index.md) (some are linked below):

- [3DR Pixhawk v1 Flight Controller](../flight_controller/pixhawk.md#console-port) (also applies to
  [mRo Pixhawk](../flight_controller/mro_pixhawk.md#debug-ports), [Holybro pix32](../flight_controller/holybro_pix32.md#debug-port))
- [Pixhawk 3](../flight_controller/pixhawk3_pro.md#debug-port)
- [Pixracer](../flight_controller/pixracer.md#debug-port)

<a id="pixhawk_debug_port"></a>

### Порти відладки Pixhawk

Pixhawk flight controllers usually come with a [Pixhawk Connector Standard Debug Port](../debug/swd_debug.md#pixhawk-connector-standard-debug-ports) which will be either the 10 pin [Pixhawk Debug Full](../debug/swd_debug.md#pixhawk-debug-full) or 6 pin [Pixhawk Debug Mini](../debug/swd_debug.md#pixhawk-debug-mini) port.

Ці порти мають контакти для консольного TX та RX, які можуть бути підключені до кабелю FTDI.
The mapping for the [Pixhawk Debug Mini](../debug/swd_debug.md#pixhawk-debug-mini) to FTDI is shown below.

| Порти відладки Pixhawk     | -                                   | FTDI | -                                                                   |
| -------------------------- | ----------------------------------- | ---- | ------------------------------------------------------------------- |
| 1 (red) | TARGET PROCESSOR VOLTAGE            |      | N/C (використовується для налагодження SWD/JTAG) |
| 2 (blk) | CONSOLE TX (OUT) | 5    | FTDI RX (yellow)                                 |
| 3 (blk) | CONSOLE RX (IN)  | 4    | FTDI TX (orange)                                 |
| 4 (blk) | SWDIO                               |      | N/C (використовується для налагодження SWD/JTAG) |
| 5 (blk) | SWCLK                               |      | N/C (використовується для налагодження SWD/JTAG) |
| 6 (blk) | GND                                 | 1    | FTDI GND (black)                                 |

The [SWD Debug Port](../debug/swd_debug.md) page and individual flight controller pages have more information about debug port pinouts.

## Відкриття консолі

Після підключення консолі використовуйте інструмент вибору або типові порти з'єднання, описані нижче:

### Linux / Mac OS: Screen

Встановіть screen на Ubuntu (Mac OS вже має його встановленим):

```sh
sudo apt-get install screen
```

- Серійний підключення: Pixhawk v1 / Pixracer використовує швидкість передачі 57600 бод

Connect screen at BAUDRATE baud, 8 data bits, 1 stop bit to the right serial port (use `ls /dev/tty*` and watch what changes when unplugging / replugging the USB device). Common names are `/dev/ttyUSB0` and `/dev/ttyACM0` for Linux and `/dev/tty.usbserial-ABCBD` for Mac OS.

```sh
screen /dev/ttyXXX BAUDRATE 8N1
```

### Windows: PuTTY

Download [PuTTY](http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html) and start it.

Потім виберіть 'серійне підключення' і встановіть параметри порту:

- 57600 baud
- 8 data bits
- 1 stop bit
