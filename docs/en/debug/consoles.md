# PX4 Consoles/Shells

PX4 enables terminal access to the system through the [MAVLink Shell](../debug/mavlink_shell.md) and the [System Console](../debug/system_console.md).

This page explains the main differences and how the console/shell are used.

<a id="console_vs_shell"></a>

## System Console vs. Shells

The PX4 _System Console_ provides low-level access to the system, debug output and analysis of the system boot process.

There is just one _System Console_, which runs on one specific UART (the debug port, as configured in NuttX), and is commonly attached to a computer via an FTDI cable (or some other debug adapter like a [Dronecode probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation)).

- Used for _low-level debugging/development_: bootup, NuttX, startup scripts, board bringup, development on central parts of PX4 (e.g. uORB).
- In particular, is the only place where all boot output (including information about applications auto-started on boot) is printed.

Shells provide higher-level access to the system:

- Used for basic module testing/running commands.
- Only _directly_ display the output of modules you start.
- Cannot _directly_ display the output of tasks running on the work queue.
- Can't debug problems when the system doesn't start (as it isn't running yet).

::: info
The `dmesg` command is now available through the shell on some boards, enabling much lower level debugging than previously possible.
For example, with `dmesg -f &` you also see the output of background tasks.
:::

There can be several shells, either running on a dedicated UART, or via MAVLink.
Since MAVLink provides more flexibility, currently only the [MAVLink Shell](../debug/mavlink_shell.md) is used.

The [System Console](../debug/system_console.md) is essential when the system does not boot (it displays the system boot log when power-cycling the board).
The [MAVLink Shell](../debug/mavlink_shell.md) is much easier to setup, and so is more generally recommended for most debugging.

<a id="using_the_console"></a>

## Using Consoles/Shells

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
