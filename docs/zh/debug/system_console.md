# PX4 系统控制台

The PX4 _System Console_ provides low-level access to the system, debug output and analysis of the system boot process.

:::tip
The console should be used for debugging if the system won't boot.
The [MAVLink Shell](../debug/mavlink_shell.md) may otherwise be more suitable, as it is much easier to set up and can be used for [many of the same tasks](../debug/consoles.md#console_vs_shell).
:::

## System Console vs. Shells

The console is made available through a (board-specific) UART that can be connected to a computer USB port using a [3.3V FTDI](https://www.digikey.com/en/products/detail/TTL-232R-3V3/768-1015-ND/1836393) cable.
This allows the console to be accessed using a terminal application.

Pixhawk controller manufacturers are expected to expose the console UART and SWD (JTAG) debug interfaces through a dedicated _debug port_ that complies with the [Pixhawk Connector Standard](#pixhawk_debug_port).
Unfortunately some boards predate this standard or a non-compliant.

:::info
Developers targeting a number of different boards may wish to use a [debug adapter](../debug/swd_debug.md#debug-adapters) to simplify connecting boards to FTDI cables and [debug probes](../debug/swd_debug.md#debug-probes-for-px4-hardware).
:::

Connect the 6-pos JST SH 1:1 cable to the Dronecode probe or connect the individual pins of the cable to a FTDI cable like this:

### Connecting via Dronecode Probe

The System Console UART pinouts/debug ports are typically documented in [autopilot overview pages](../flight_controller/index.md) (some are linked below):

- [3DR Pixhawk v1 Flight Controller](../flight_controller/pixhawk.md#console-port) (also applies to
  [mRo Pixhawk](../flight_controller/mro_pixhawk.md#debug-ports), [Holybro pix32](../flight_controller/holybro_pix32.md#debug-port))
- [Pixhawk 3](../flight_controller/pixhawk3_pro.md#debug-port)
- [Pixracer](../flight_controller/pixracer.md#debug-port)

<a id="pixhawk_debug_port"></a>

### Connecting via FTDI 3.3V Cable

Pixhawk flight controllers usually come with a [Pixhawk Connector Standard Debug Port](../debug/swd_debug.md#pixhawk-connector-standard-debug-ports) which will be either the 10 pin [Pixhawk Debug Full](../debug/swd_debug.md#pixhawk-debug-full) or 6 pin [Pixhawk Debug Mini](../debug/swd_debug.md#pixhawk-debug-mini) port.

These ports have pins for console TX and RX which can connect to an FTDI cable.
The mapping for the [Pixhawk Debug Mini](../debug/swd_debug.md#pixhawk-debug-mini) to FTDI is shown below.

| Connecting via FTDI 3.3V Cable | -                            | FTDI | -                                |
| ---------------------------------------------- | ---------------------------- | ---- | -------------------------------- |
| 1（红）                                           | + 5v (红色) |      | N/C                              |
| 2                                              | UART7 Tx                     | 5    | FTDI RX （黄色）                     |
| 3                                              | UART7 Rx                     | 4    | FTDI TX （橙色）                     |
| 4（黑）                                           | SWDIO                        |      | N/C                              |
| 6                                              | SWCLK                        |      | N/C                              |
| 6                                              | GND                          | 1    | FTDI GND (黑色) |

The [SWD Debug Port](../debug/swd_debug.md) page and individual flight controller pages have more information about debug port pinouts.

## 打开控制台

After the console connection is wired up, use the default serial port tool of your choice or the defaults described below:

### Linux / Mac OS: Screen

下载 <a href="http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html">PuTTY</a> 并启动它。

```sh
sudo apt-get install screen
```

- 串口：pixhawk v1/pixracer 使用 57600 波特率

Connect screen at BAUDRATE baud, 8 data bits, 1 stop bit to the right serial port (use `ls /dev/tty*` and watch what changes when unplugging / replugging the USB device). Common names are `/dev/ttyUSB0` and `/dev/ttyACM0` for Linux and `/dev/tty.usbserial-ABCBD` for Mac OS.

```sh
screen /dev/ttyXXX BAUDRATE 8N1
```

### Windows: PuTTY

Download [PuTTY](http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html) and start it.

Then select 'serial connection' and set the port parameters to:

- 57600 波特率
- 8 数据位
- 1 个停止位
