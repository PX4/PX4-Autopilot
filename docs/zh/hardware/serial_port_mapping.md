# 串口映射

本主题说明如何确定USART/UART串行端口（下称串口）设备名称(例如“ttyS0”)与飞行控制器上对应端口（如`TELEM1`、`TELEM2`、`GPS1`、`RC SBUS`、`调试控制台(Debug console)`）之间的映射关系。

这份说明用于在飞行控制器文档中生成串行端口映射表。
例如： [Pixhawk 4 > Serial Port Mapping](../flight_controller/pixhawk4.md#serial-port-mapping)。

:::info
每个端口分配的功能不必与名称匹配（大多数情况下），并通过[串行端口配置](../peripherals/serial_configuration.md)。
通常情况下端口功能是与名称相匹配的，因此标记为`GPS1`的端口可直接连接GPS设备。
:::

## NuttX 在 STMxxyyy 上

<!-- instructions from DavidS here: https://github.com/PX4/PX4-user_guide/pull/672#issuecomment-598198434 -->

本节将展示如何通过检查板载配置文件，获取在 STMxxyyy 架构上构建 NuttX 所需的映射信息。
该说明使用 FMUv5，但同样可扩展至其他FMU版本/NuttX开发板。

###

**default.px4board** 文件列出了若干串行端口映射（搜索文本“SERIAL_PORTS”）。

来自 [/boards/px4/fmu-v5/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/default.px4board):

```
CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS0"
CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS1"
CONFIG_BOARD_SERIAL_TEL2="/dev/ttyS2"
CONFIG_BOARD_SERIAL_TEL4="/dev/ttyS3"
```

或者，您可以通过执行 `make px4_fmu-v5 boardconfig` 启动板配置工具，并进入串口菜单。

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

_nsh/defconfig_ 允许您确定哪些端口被定义，它们是 UART 还是 USART，以及 USART/UART 与设备之间的映射关系。
您还可以确定用于该功能的端口[串口/调试控制台](../debug/system_console.md)。

打开板载的 defconfig 配置文件，例如：[/boards/px4/fmu-v5/nuttx-config/nsh/defconfig](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/nuttx-config/nsh/defconfig#L215-L221)

搜索文本“ART”，直到找到类似以下格式的条目：`CONFIG_STM32xx_USARTn=y`（其中`xx`表示处理器类型，`n`表示端口号）。
例如：

```
CONFIG_STM32F7_UART4=y
CONFIG_STM32F7_UART7=y
CONFIG_STM32F7_UART8=y
CONFIG_STM32F7_USART1=y
CONFIG_STM32F7_USART2=y
CONFIG_STM32F7_USART3=y
CONFIG_STM32F7_USART6=y
```

这些条目会告知您哪些端口已被定义，以及它们属于UART还是USART。

复制上方段落，按“n”进行数字排序。
同时递增设备编号 _ttyS**n**_（从零开始计数），以获取设备到串行端口的映射关系。

```
ttyS0 CONFIG_STM32F7_USART1=y
ttyS1 CONFIG_STM32F7_USART2=y
ttyS2 CONFIG_STM32F7_USART3=y
ttyS3 CONFIG_STM32F7_UART4=y
ttyS4 CONFIG_STM32F7_USART6=y
ttyS5 CONFIG_STM32F7_UART7=y
ttyS6 CONFIG_STM32F7_UART8=y
```

要获取调试控制台映射，我们需在[defconfig file](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/nuttx-config/nsh/defconfig#L212) 搜索 `SERIAL_CONSOLE`。
下面我们看到控制台位于UART7：

```
CONFIG_UART7_SERIAL_CONSOLE=y
```

### board_config.h

对于配备IO板的飞行控制器，请通过在**board_config.h**文件中搜索`PX4IO_SERIAL_DEVICE`来确定PX4IO连接。

例如 [/boards/px4/fmu-v5/src/board_config.h](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v5/src/board_config.h#L59)：

```
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS6"
#define PX4IO_SERIAL_TX_GPIO           GPIO_UART8_TX
#define PX4IO_SERIAL_RX_GPIO           GPIO_UART8_RX
#define PX4IO_SERIAL_BASE              STM32_UART8_BASE
```

PX4IO 位于 `ttyS6` 上（我们还可以看到它映射到 UART8，这一点我们从前一节已经知道）。

### 整合所有内容

最终映射表如下：

```
ttyS0 CONFIG_STM32F7_USART1=y GPS1
ttyS1 CONFIG_STM32F7_USART2=y TEL1
ttyS2 CONFIG_STM32F7_USART3=y TEL2
ttyS3 CONFIG_STM32F7_UART4=y TEL4
ttyS4 CONFIG_STM32F7_USART6=y
ttyS5 CONFIG_STM32F7_UART7=y DEBUG
ttyS6 CONFIG_STM32F7_UART8=y PX4IO
```

在 [flight controller docs](../flight_controller/pixhawk4.md#serial-port-mapping) 最终生成的表格如下：

| UART   | 设备         | Port                           |
| ------ | ---------- | ------------------------------ |
| UART   | /dev/ttyS0 | GPS                            |
| USART2 | /dev/ttyS1 | TELEM1 (流控) |
| USART3 | /dev/ttyS2 | TELEM2 (流控) |
| UART4  | /dev/ttyS3 | TELEM4                         |
| USART6 | /dev/ttyS4 | RC SBUS                        |
| UART7  | /dev/ttyS5 | 调试控制台                          |
| UART8  | /dev/ttyS6 | PX4IO                          |

## Other Architectures

:::info
Contributions welcome!
:::

## 另见

- [Serial Port Configuration](../peripherals/serial_configuration.md)
- [MAVLink Telemetry (OSD/GCS)](../peripherals/mavlink_peripherals.md)
