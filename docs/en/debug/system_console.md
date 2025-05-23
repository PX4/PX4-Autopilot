# PX4 System Console

The PX4 _System Console_ provides low-level access to the system, debug output and analysis of the system boot process.

:::tip
The console should be used for debugging if the system won't boot.
The [MAVLink Shell](../debug/mavlink_shell.md) may otherwise be more suitable, as it is much easier to set up and can be used for [many of the same tasks](../debug/consoles.md#console_vs_shell).
:::

## Wiring the Console

The console is made available through a (board-specific) UART that can be connected to a computer USB port using a [3.3V FTDI](https://www.digikey.com/en/products/detail/TTL-232R-3V3/768-1015-ND/1836393) cable.
This allows the console to be accessed using a terminal application.

Pixhawk controller manufacturers are expected to expose the console UART and SWD (JTAG) debug interfaces through a dedicated _debug port_ that complies with the [Pixhawk Connector Standard](#pixhawk_debug_port).
Unfortunately some boards predate this standard or a non-compliant.

::: info
Developers targeting a number of different boards may wish to use a [debug adapter](../debug/swd_debug.md#debug-adapters) to simplify connecting boards to FTDI cables and [debug probes](../debug/swd_debug.md#debug-probes-for-px4-hardware).
:::

The sections below outline/link to the wiring and system console information for many common boards.

### Board-Specific Wiring

The System Console UART pinouts/debug ports are typically documented in [autopilot overview pages](../flight_controller/index.md) (some are linked below):

- [3DR Pixhawk v1 Flight Controller](../flight_controller/pixhawk.md#console-port) (also applies to
  [mRo Pixhawk](../flight_controller/mro_pixhawk.md#debug-ports), [Holybro pix32](../flight_controller/holybro_pix32.md#debug-port))
- [Pixhawk 3](../flight_controller/pixhawk3_pro.md#debug-port)
- [Pixracer](../flight_controller/pixracer.md#debug-port)

<a id="pixhawk_debug_port"></a>

### Pixhawk Debug Port

Pixhawk flight controllers usually come with a [Pixhawk Connector Standard Debug Port](../debug/swd_debug.md#pixhawk-connector-standard-debug-ports) which will be either the 10 pin [Pixhawk Debug Full](../debug/swd_debug.md#pixhawk-debug-full) or 6 pin [Pixhawk Debug Mini](../debug/swd_debug.md#pixhawk-debug-mini) port.

These ports have pins for console TX and RX which can connect to an FTDI cable.
The mapping for the [Pixhawk Debug Mini](../debug/swd_debug.md#pixhawk-debug-mini) to FTDI is shown below.

| Pixhawk Debug Port | -                        | FTDI | -                                 |
| ------------------ | ------------------------ | ---- | --------------------------------- |
| 1 (red)            | TARGET PROCESSOR VOLTAGE |      | N/C (used for SWD/JTAG debugging) |
| 2 (blk)            | CONSOLE TX (OUT)         | 5    | FTDI RX (yellow)                  |
| 3 (blk)            | CONSOLE RX (IN)          | 4    | FTDI TX (orange)                  |
| 4 (blk)            | SWDIO                    |      | N/C (used for SWD/JTAG debugging) |
| 5 (blk)            | SWCLK                    |      | N/C (used for SWD/JTAG debugging) |
| 6 (blk)            | GND                      | 1    | FTDI GND (black)                  |

The [SWD Debug Port](../debug/swd_debug.md) page and individual flight controller pages have more information about debug port pinouts.

## Opening the Console

After the console connection is wired up, use the default serial port tool of your choice or the defaults described below:

### Linux / Mac OS: Screen

Install screen on Ubuntu (Mac OS already has it installed):

```sh
sudo apt-get install screen
```

- Serial: Pixhawk v1 / Pixracer use 57600 baud

Connect screen at BAUDRATE baud, 8 data bits, 1 stop bit to the right serial port (use `ls /dev/tty*` and watch what changes when unplugging / replugging the USB device). Common names are `/dev/ttyUSB0` and `/dev/ttyACM0` for Linux and `/dev/tty.usbserial-ABCBD` for Mac OS.

```sh
screen /dev/ttyXXX BAUDRATE 8N1
```

### Windows: PuTTY

Download [PuTTY](http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html) and start it.

Then select 'serial connection' and set the port parameters to:

- 57600 baud
- 8 data bits
- 1 stop bit
