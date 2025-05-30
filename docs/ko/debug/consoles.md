# PX4 콘솔/쉘

PX4 enables terminal access to the system through the [MAVLink Shell](../debug/mavlink_shell.md) and the [System Console](../debug/system_console.md).

이 페이지에서는 콘솔과 쉘 사용 방법과 주요 차이점을 설명합니다.

<a id="console_vs_shell"></a>

## 시스템 콘솔과 셸의 차이점

The PX4 _System Console_ provides low-level access to the system, debug output and analysis of the system boot process.

There is just one _System Console_, which runs on one specific UART (the debug port, as configured in NuttX), and is commonly attached to a computer via an FTDI cable (or some other debug adapter like a [Dronecode probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation)).

- Used for _low-level debugging/development_: bootup, NuttX, startup scripts, board bringup, development on central parts of PX4 (e.g. uORB).
- 특히, 모든 부팅 출력(부팅 시 자동으로 시작되는 응용 프로그램에 대한 정보 포함)이 인쇄되는 유일한 장소입니다.

셸은 시스템에 대한 상위 수준의 접급을 제공합니다.

- 기본 모듈 테스트와 명령어를 실행할 수 있습니다.
- Only _directly_ display the output of modules you start.
- Cannot _directly_ display the output of tasks running on the work queue.
- 시스템이 시작되지 않으면(아직 실행되지 않기 때문에) 문제를 디버그할 수 없습니다.

:::info
The `dmesg` command is now available through the shell on some boards, enabling much lower level debugging than previously possible.
For example, with `dmesg -f &` you also see the output of background tasks.
:::

전용 UART에서 실행되거나, MAVLink로 실행되는 여러 셸이 있을 수 있습니다.
Since MAVLink provides more flexibility, currently only the [MAVLink Shell](../debug/mavlink_shell.md) is used.

The [System Console](../debug/system_console.md) is essential when the system does not boot (it displays the system boot log when power-cycling the board).
The [MAVLink Shell](../debug/mavlink_shell.md) is much easier to setup, and so is more generally recommended for most debugging.

<a id="using_the_console"></a>

## 콘솔/쉘 사용

The MAVLink shell/console and the [System Console](../debug/system_console.md) are used in much the same way.

For example, type `ls` to view the local file system, `free` to see the remaining free RAM, `dmesg` to look at boot output.

```sh
nsh> ls
nsh> free
nsh> dmesg
```

Below are a couple of commands which can be used in the [NuttShell](https://cwiki.apache.org/confluence/pages/viewpage.action?pageId=139629410) to get insights of the system.

This NSH command provides the remaining free memory:

```sh
free
```

The top command shows the stack usage per application:

```sh
top
```

Note that stack usage is calculated with stack coloring and is the maximum since the start of the task (not the current usage).

To see what is running in the work queues and at what rate, use:

```sh
work_queue status
```

To debug uORB topics:

```sh
uorb top
```

To inspect a specific uORB topic:

```sh
listener <topic_name>
```

Many other system commands and modules are listed in the [Modules and Command Reference](../modules/modules_main.md) (e.g. `top`, `listener`, etc.).

:::tip
Some commands may be disabled on some boards (i.e. the some modules are not included in firmware for boards with RAM or FLASH constraints).
In this case you will see the response: `command not found`
:::
