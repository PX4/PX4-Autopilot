# PX4 시스템 콘솔

The PX4 _System Console_ provides low-level access to the system, debug output and analysis of the system boot process.

:::tip
The console should be used for debugging if the system won't boot.
The [MAVLink Shell](../debug/mavlink_shell.md) may otherwise be more suitable, as it is much easier to set up and can be used for [many of the same tasks](../debug/consoles.md#console_vs_shell).
:::

## 콘솔 배선

The console is made available through a (board-specific) UART that can be connected to a computer USB port using a [3.3V FTDI](https://www.digikey.com/en/products/detail/TTL-232R-3V3/768-1015-ND/1836393) cable.
이렇게 하면, 터미널 응용 프로그램을 사용하여 콘솔에 접근할 수 있습니다.

Pixhawk controller manufacturers are expected to expose the console UART and SWD (JTAG) debug interfaces through a dedicated _debug port_ that complies with the [Pixhawk Connector Standard](#pixhawk_debug_port).
불행히도, 일부 보드는 이 표준 이전이거나 비준수품입니다.

:::info
Developers targeting a number of different boards may wish to use a [debug adapter](../debug/swd_debug.md#debug-adapters) to simplify connecting boards to FTDI cables and [debug probes](../debug/swd_debug.md#debug-probes-for-px4-hardware).
:::

아래 섹션은 많은 공통 보드에 대한 배선 및 시스템 콘솔 정보에 대한 개요를 설명합니다.

### 보드별 연결 방법

The System Console UART pinouts/debug ports are typically documented in [autopilot overview pages](../flight_controller/index.md) (some are linked below):

- [3DR Pixhawk v1 Flight Controller](../flight_controller/pixhawk.md#console-port) (also applies to
  [mRo Pixhawk](../flight_controller/mro_pixhawk.md#debug-ports), [Holybro pix32](../flight_controller/holybro_pix32.md#debug-port))
- [Pixhawk 3](../flight_controller/pixhawk3_pro.md#debug-port)
- [Pixracer](../flight_controller/pixracer.md#debug-port)

<a id="pixhawk_debug_port"></a>

### Pixhawk 디버그 포트

Pixhawk flight controllers usually come with a [Pixhawk Connector Standard Debug Port](../debug/swd_debug.md#pixhawk-connector-standard-debug-ports) which will be either the 10 pin [Pixhawk Debug Full](../debug/swd_debug.md#pixhawk-debug-full) or 6 pin [Pixhawk Debug Mini](../debug/swd_debug.md#pixhawk-debug-mini) port.

These ports have pins for console TX and RX which can connect to an FTDI cable.
The mapping for the [Pixhawk Debug Mini](../debug/swd_debug.md#pixhawk-debug-mini) to FTDI is shown below.

| Pixhawk 디버그 포트            | -                                  | FTDI | -                                          |
| ------------------------- | ---------------------------------- | ---- | ------------------------------------------ |
| 1(red) | TARGET PROCESSOR VOLTAGE           |      | N/C (SWD/JTAG 디버깅에 사용됨) |
| 2 (흑)  | CONSOLE TX (출력) | 5    | FTDI RX (황)             |
| 3 (흑)  | CONSOLE RX (입력) | 4    | FTDI TX (적황)            |
| 4 (흑)  | SWDIO                              |      | N/C (SWD/JTAG 디버깅에 사용됨) |
| 5 (흑)  | SWCLK                              |      | N/C (SWD/JTAG 디버깅에 사용됨) |
| 6 (흑)  | GND                                | 1    | FTDI GND (흑)            |

The [SWD Debug Port](../debug/swd_debug.md) page and individual flight controller pages have more information about debug port pinouts.

## 콘솔 열기

콘솔 연결이 연결된 후, 선택한 기본 직렬 포트 도구 또는 아래에 설명된 기본값을 사용합니다.

### Linux / Mac 운영체제: Screen

Ubuntu에 screen 명령어를 설치합니다. Mac OS에 이미 설치되어 있습니다.

```sh
sudo apt-get install screen
```

- 시리얼: Pixhawk v1 / Pixracer는 57600 보드를 사용합니다.

Connect screen at BAUDRATE baud, 8 data bits, 1 stop bit to the right serial port (use `ls /dev/tty*` and watch what changes when unplugging / replugging the USB device). Common names are `/dev/ttyUSB0` and `/dev/ttyACM0` for Linux and `/dev/tty.usbserial-ABCBD` for Mac OS.

```sh
screen /dev/ttyXXX BAUDRATE 8N1
```

### 윈도우: PuTTY

Download [PuTTY](http://www.chiark.greenend.org.uk/~sgtatham/putty/download.html) and start it.

'직렬 연결'을 선택하고, 포트 매개변수를 다음과 같이 설정합니다.

- 초당 전송속도: 57600
- 8 data bits
- 1 stop bit
