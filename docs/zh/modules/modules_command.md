# 模块参考：命令（Command）

## actuator_test

Source: [systemcmds/actuator_test](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/actuator_test)

用于测试执行器的实用程序

警告：在使用此命令之前移除所有螺旋桨。

<a id="actuator_test_usage"></a>

### 用法

```
actuator_test <command> [arguments...]
 Commands:
   set           Set an actuator to a specific output value

 The actuator can be specified by motor, servo or function directly:
     [-m <val>]  Motor to test (1...8)
     [-s <val>]  Servo to test (1...8)
     [-f <val>]  Specify function directly
     -v <val>    value (-1...1)
     [-t <val>]  Timeout in seconds (run interactive if not set)
                 default: 0

   iterate-motors Iterate all motors starting and stopping one after the other

   iterate-servos Iterate all servos deflecting one after the other
```

## bl_update

Source: [systemcmds/bl_update](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/bl_update)

Utility to flash the bootloader from a file <a id="bl_update_usage"></a>

### 用法

```
bl_update [arguments...]
   setopt        Set option bits to unlock the FLASH (only needed if in locked
                 state)

   <file>        Bootloader bin file
```

## bsondump

Source: [systemcmds/bsondump](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/bsondump)

Utility to read BSON from a file and print or output document size. <a id="bsondump_usage"></a>

### 用法

```
bsondump [arguments...]
     <file>      The BSON file to decode and print.
```

## dumpfile

Source: [systemcmds/dumpfile](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/dumpfile)

转储文件实用程序。 以二进制模式（不要用 CR LF 替换 LF）将文件大小和内容打印到标准输出。 <a id="dumpfile_usage"></a>

### 用法

```
dumpfile [arguments...]
     <file>      File to dump
```

## dyn

Source: [systemcmds/dyn](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/dyn)

### 描述

加载并运行一个没有被编译进 PX4 的二进制文件的动态 PX4 模块

### Example

```
dyn ./hello.px4mod start
```

<a id="dyn_usage"></a>

### 用法

```
dyn [arguments...]
     <file>      File containing the module
     [arguments...] Arguments to the module
```

## failure

Source: [systemcmds/failure](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/failure)

### 描述

向系统中注入故障。

### 实现

此系统命令通过 uORB 发送一个机体命令来出发故障。

### 示例

通过停止GPS来测试GPS故障保护:

failure gps off

<a id="failure_usage"></a>

### 用法

```
failure [arguments...]
   help          Show this help text

   gps|...       Specify component

   ok|off|...    Specify failure type
     [-i <val>]  sensor instance (0=all)
                 default: 0
```

## gpio

Source: [systemcmds/gpio](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/gpio)

### 描述

此命令用于读写GPIO

```
gpio read <PORT><PIN>/<DEVICE> [PULLDOWN|PULLUP] [--force]
gpio write <PORT><PIN>/<DEVICE> <VALUE> [PUSHPULL|OPENDRAIN] [--force]
```

### 示例

读取配置为上拉的 PH4 引脚，它的值为高

```
gpio read H4 PULLUP
```

1 OK

设置 PE7 的输出值为高

```
gpio write E7 1 --force
```

Set the output value on device /dev/gpio1 to high

```
gpio write /dev/gpio1 1
```

<a id="gpio_usage"></a>

### 用法

```
gpio [arguments...]
   read
     <PORT><PIN>/<DEVICE> GPIO port and pin or device
     [PULLDOWN|PULLUP] Pulldown/Pullup
     [--force]   Force (ignore board gpio list)

   write
     <PORT> <PIN> GPIO port and pin
     <VALUE>     Value to write
     [PUSHPULL|OPENDRAIN] Pushpull/Opendrain
     [--force]   Force (ignore board gpio list)
```

## hardfault_log

Source: [systemcmds/hardfault_log](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/hardfault_log)

Hardfault 实用程序

在启动脚本中用于处理 hardfaults。

<a id="hardfault_log_usage"></a>

### 用法

```
hardfault_log <command> [arguments...]
 Commands:
   check         Check if there's an uncommitted hardfault

   rearm         Drop an uncommitted hardfault

   fault         Generate a hardfault (this command crashes the system :)
     [0|1|2|3]   Hardfault type: 0=divide by 0, 1=Assertion, 2=jump to 0x0,
                 3=write to 0x0 (default=0)

   commit        Write uncommitted hardfault to /fs/microsd/fault_%i.txt (and
                 rearm, but don't reset)

   count         Read the reboot counter, counts the number of reboots of an
                 uncommitted hardfault (returned as the exit code of the
                 program)

   reset         Reset the reboot counter
```

## hist

Source: [systemcmds/hist](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/hist)

Command-line tool to show the px4 message history. There are no arguments. <a id="hist_usage"></a>

### 用法

```
hist [arguments...]
```

## i2cdetect

Source: [systemcmds/i2cdetect](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/i2cdetect)

Utility to scan for I2C devices on a particular bus. <a id="i2cdetect_usage"></a>

### 用法

```
i2cdetect [arguments...]
     [-b <val>]  I2C bus
                 default: 1
```

## led_control

Source: [systemcmds/led_control](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/led_control)

### 描述

Command-line tool to control & test the (external) LED's.

要使用它，请确保有一个处理 led_control 的 uorb 主题的设备正在运行。

有不同的优先级，例如一个模块可以设置低优先级的颜色，另一个模块可以高优先级闪烁 N 次，闪烁后 LED 自动返回低优先级状态。 The `reset` command can also be used to return to a lower priority.

### 示例

第一个 LED 闪烁蓝光 5 次：

```
led_control blink -c blue -l 0 -n 5
```

<a id="led_control_usage"></a>

### 用法

```
led_control <command> [arguments...]
 Commands:
   test          Run a test pattern

   on            Turn LED on

   off           Turn LED off

   reset         Reset LED priority

   blink         Blink LED N times
     [-n <val>]  Number of blinks
                 default: 3
     [-s <val>]  Set blinking speed
                 values: fast|normal|slow, default: normal

   breathe       Continuously fade LED in & out

   flash         Two fast blinks and then off with frequency of 1Hz

 The following arguments apply to all of the above commands except for 'test':
     [-c <val>]  color
                 values: red|blue|green|yellow|purple|amber|cyan|white, default:
                 white
     [-l <val>]  Which LED to control: 0, 1, 2, ... (default=all)
     [-p <val>]  Priority
                 default: 2
```

## listener

Source: [systemcmds/topic_listener](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/topic_listener)

监听 uORB 主题并将数据打印到控制台的实用程序。

可以通过按 Ctrl+C、Esc 或 Q 随时退出侦听器。

<a id="listener_usage"></a>

### 用法

```
listener <command> [arguments...]
 Commands:
     <topic_name> uORB topic name
     [-i <val>]  Topic instance
                 default: 0
     [-n <val>]  Number of messages
                 default: 1
     [-r <val>]  Subscription rate (unlimited if 0)
                 default: 0
```

## mfd

Source: [systemcmds/mft](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/mft)

Utility interact with the manifest <a id="mfd_usage"></a>

### 用法

```
mfd <command> [arguments...]
 Commands:
   query         Returns true if not existed
```

## mft_cfg

Source: [systemcmds/mft_cfg](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/mft_cfg)

Tool to set and get manifest configuration <a id="mft_cfg_usage"></a>

### 用法

```
mft_cfg <command> [arguments...]
 Commands:
   get           get manifest configuration

   set           set manifest configuration

   reset         reset manifest configuration
     hwver|hwrev Select type: MTD_MTF_VER|MTD_MTF_REV
     -i <val>    argument to set extended hardware id (id == version for
                 <hwver>, id == revision for <hwrev> )
```

## mtd

Source: [systemcmds/mtd](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/mtd)

Utility to mount and test partitions (based on FRAM/EEPROM storage as defined by the board) <a id="mtd_usage"></a>

### 用法

```
mtd <command> [arguments...]
 Commands:
   status        Print status information

   readtest      Perform read test

   rwtest        Perform read-write test

   erase         Erase partition(s)

 The commands 'readtest' and 'rwtest' have an optional instance index:
     [-i <val>]  storage index (if the board has multiple storages)
                 default: 0

 The commands 'readtest', 'rwtest' and 'erase' have an optional parameter:
     [<partition_name1> [<partition_name2> ...]] Partition names (eg.
                 /fs/mtd_params), use system default if not provided
```

## nshterm

Source: [systemcmds/nshterm](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/nshterm)

在指定端口启动一个 NSH shell。

该命令此前被用于在 USB 串口端口开启一个 shell。
现在运行mavlink，并且可以在mavlink 上使用shell。

<a id="nshterm_usage"></a>

### 用法

```
nshterm [arguments...]
     <file:dev>  Device on which to start the shell (eg. /dev/ttyACM0)
```

## param

Source: [systemcmds/param](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/param)

### 描述

通过 shell 或脚本访问和操作参数的命令。

例如，这在启动脚本中用于设置特定于机身的参数。

Parameters are automatically saved when changed, eg. with `param set`. 它们通常存储在 FRAM 或 SD 卡中。 `param select` can be used to change the storage location for subsequent saves (this will
need to be (re-)configured on every boot).

If the FLASH-based backend is enabled (which is done at compile time, e.g. for the Intel Aero or Omnibus),
`param select` has no effect and the default is always the FLASH backend. However `param save/load <file>`
can still be used to write to/read from files.

每个参数有一个“已使用”的标志，此标志在启动过程参数被读取后被置位。 它只是用于向地面控制站显示相关参数。

### 示例

更改机身并确保已加载机身的默认参数：

```
param set SYS_AUTOSTART 4001
param set SYS_AUTOCONFIG 1
reboot
```

<a id="param_usage"></a>

### 用法

```
param <command> [arguments...]
 Commands:
   load          Load params from a file (overwrite all)
     [<file>]    File name (use default if not given)

   import        Import params from a file
     [<file>]    File name (use default if not given)

   save          Save params to a file
     [<file>]    File name (use default if not given)

   select        Select default file
     [<file>]    File name

   select-backup Select default file
     [<file>]    File name

   show          Show parameter values
     [-a]        Show all parameters (not just used)
     [-c]        Show only changed params (unused too)
     [-q]        quiet mode, print only param value (name needs to be exact)
     [<filter>]  Filter by param name (wildcard at end allowed, eg. sys_*)

   show-for-airframe Show changed params for airframe config

   status        Print status of parameter system

   set           Set parameter to a value
     <param_name> <value> Parameter name and value to set
     [fail]      If provided, let the command fail if param is not found

   set-default   Set parameter default to a value
     [-s]        If provided, silent errors if parameter doesn't exists
     <param_name> <value> Parameter name and value to set
     [fail]      If provided, let the command fail if param is not found

   compare       Compare a param with a value. Command will succeed if equal
     [-s]        If provided, silent errors if parameter doesn't exists
     <param_name> <value> Parameter name and value to compare

   greater       Compare a param with a value. Command will succeed if param is
                 greater than the value
     [-s]        If provided, silent errors if parameter doesn't exists
     <param_name> <value> Parameter name and value to compare
     <param_name> <value> Parameter name and value to compare

   touch         Mark a parameter as used
     [<param_name1> [<param_name2>]] Parameter name (one or more)

   reset         Reset only specified params to default
     [<param1> [<param2>]] Parameter names to reset (wildcard at end allowed)

   reset_all     Reset all params to default
     [<exclude1> [<exclude2>]] Do not reset matching params (wildcard at end
                 allowed)

   index         Show param for a given index
     <index>     Index: an integer >= 0

   index_used    Show used param for a given index
     <index>     Index: an integer >= 0

   find          Show index of a param
     <param>     param name
```

## payload_deliverer

Source: [modules/payload_deliverer](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/payload_deliverer)

### 描述

Handles payload delivery with either Gripper or a Winch with an appropriate timeout / feedback sensor setting,
and communicates back the delivery result as an acknowledgement internally

<a id="payload_deliverer_usage"></a>

### 用法

```
payload_deliverer <command> [arguments...]
 Commands:
   start

   gripper_test  Tests the Gripper's release & grabbing sequence

   gripper_open  Opens the gripper

   gripper_close Closes the gripper

   stop

   status        print status info
```

## perf

Source: [systemcmds/perf](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/perf)

Tool to print performance counters <a id="perf_usage"></a>

### 用法

```
perf [arguments...]
   reset         Reset all counters

   latency       Print HRT timer latency histogram

 Prints all performance counters if no arguments given
```

## reboot

Source: [systemcmds/reboot](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/reboot)

Reboot the system <a id="reboot_usage"></a>

### 用法

```
reboot [arguments...]
     [-b]        Reboot into bootloader
     [-i]        Reboot into ISP (1st stage bootloader)
     [lock|unlock] Take/release the shutdown lock (for testing)
```

## sd_bench

Source: [systemcmds/sd_bench](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/sd_bench)

Test the speed of an SD Card <a id="sd_bench_usage"></a>

### 用法

```
sd_bench [arguments...]
     [-b <val>]  Block size for each read/write
                 default: 4096
     [-r <val>]  Number of runs
                 default: 5
     [-d <val>]  Duration of a run in ms
                 default: 2000
     [-k]        Keep the test file
     [-s]        Call fsync after each block (default=at end of each run)
     [-u]        Test performance with unaligned data
     [-U]        Test performance with forced byte unaligned data
     [-v]        Verify data and block number
```

## sd_stress

Source: [systemcmds/sd_stress](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/sd_stress)

Test operations on an SD Card <a id="sd_stress_usage"></a>

### 用法

```
sd_stress [arguments...]
     [-r <val>]  Number of runs
                 default: 5
     [-b <val>]  Number of bytes
                 default: 100
```

## serial_passthru

Source: [systemcmds/serial_passthru](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/serial_passthru)

把数据从一个设备传输到另一个设备。

This can be used to use u-center connected to USB with a GPS on a serial port.

<a id="serial_passthru_usage"></a>

### 用法

```
serial_passthru [arguments...]
     -e <val>    External device path
                 values: <file:dev>
     -d <val>    Internal device path
                 values: <file:dev>
     [-b <val>]  Baudrate
                 default: 115200
     [-t]        Track the External devices baudrate on internal device
```

## 系统时间

Source: [systemcmds/system_time](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/system_time)

### 描述

Command-line tool to set and get system time.

### 示例

Set the system time and read it back

```
system_time set 1600775044
system_time get
```

<a id="system_time_usage"></a>

### 用法

```
system_time <command> [arguments...]
 Commands:
   set           Set the system time, provide time in unix epoch time format

   get           Get the system time
```

## top

Source: [systemcmds/top](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/top)

Monitor running processes and their CPU, stack usage, priority and state <a id="top_usage"></a>

### 用法

```
top [arguments...]
   once          print load only once
```

## usb_connected

Source: [systemcmds/usb_connected](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/usb_connected)

Utility to check if USB is connected. Was previously used in startup scripts.
A return value of 0 means USB is connected, 1 otherwise. <a id="usb_connected_usage"></a>

### 用法

```
usb_connected [arguments...]
```

## ver

Source: [systemcmds/ver](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/ver)

Tool to print various version information <a id="ver_usage"></a>

### 用法

```
ver <command> [arguments...]
 Commands:
   hw            Hardware architecture

   mcu           MCU info

   git           git version information

   bdate         Build date and time

   gcc           Compiler info

   bdate         Build date and time

   px4guid       PX4 GUID

   uri           Build URI

   all           Print all versions

   hwcmp         Compare hardware version (returns 0 on match)
     <hw> [<hw2>] Hardware to compare against (eg. PX4_FMU_V4). An OR comparison
                 is used if multiple are specified

   hwtypecmp     Compare hardware type (returns 0 on match)
     <hwtype> [<hwtype2>] Hardware type to compare against (eg. V2). An OR
                 comparison is used if multiple are specified

   hwbasecmp     Compare hardware base (returns 0 on match)
     <hwbase> [<hwbase2>] Hardware type to compare against (eg. V2). An OR
                 comparison is used if multiple are specified
```
