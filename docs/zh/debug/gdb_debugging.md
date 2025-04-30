# Debugging with GDB

The [GNU DeBugger (GDB)](https://sourceware.org/gdb/documentation/) comes installed with the compiler toolchain in the form of the `arm-none-eabi-gdb` binary.
调试器读取ELF文件内的调试符号，以了解PX4固件的静态和动态内存布局。
To access the PX4 autopilot microcontroller, it needs to connect to a [Remote Target](https://sourceware.org/gdb/current/onlinedocs/gdb.html/Connecting.html), which is provided by a [SWD debug probe](swd_debug.md).

信息流看起来像这样：

```sh
Developer <=> GDB <=> GDB Server <=> Debug Probe <=> SWD <=> PX4 Autopilot.
```

## 快速入门

要启动调试会话，您通常需要：

1. Need a specialized [SWD debug probe](../debug/swd_debug.md#debug-probes).
2. Find and connect to the [SWD debug port](../debug/swd_debug.md#autopilot-debug-ports).
   You may need a [debug adapter](swd_debug.md#debug-adapters).
3. 配置并启动调试探测来创建 GDB 服务。
4. 启动GDB并作为远程目标连接到 GDB 服务。
5. 以交互方式调试您的固件。

See the debug probe documentation for details on how to setup your debug connection:

- [SEGGER J-Link](probe_jlink.md): commercial probe, no built-in serial console, requires adapter.
- [Black Magic Probe](probe_bmp.md): integrated GDB server and serial console, requires adapter.
- [STLink](probe_stlink): best value, integrated serial console, adapter must be soldered.

We recommend using the J-Link with the Pixhawk Debug Adapter or the STLinkv3-MINIE with a soldered custom cable.

Once connected, you can use the usual GDB commands such as:

- `continue` to continue program execution
- `run` to start from the beginning
- `backtrace` to see the backtrace
- `break somewhere.cpp:123` to set a breakpoint
- `delete somewhere.cpp:123` to remove it again
- `info locals` to print local variables
- `info registers` to print the registers

Consult the [GDB documentation](https://sourceware.org/gdb/documentation/) for more details.

:::tip
To avoid having to type all commands to connect in GDB each time, you can write them into `~/.gdbinit`.
:::

## Gazebo dependencies

You've now connected the flight controller to an SWD debug probe!

The following topics explain how to start on-target debugging:

- [MCU Eclipse/J-Link Debugging for PX4](eclipse_jlink.md).
- [Visual Studio Code IDE (VSCode)](../dev_setup/vscode.md).

## Embedded Debug Tools

The [Embedded Debug Tools](https://pypi.org/project/emdbg/) connect several software and hardware debugging tools together in a user friendly Python package to more easily enable advanced use cases for ARM Cortex-M microcontrollers and related devices.

The library orchestrates the launch and configuration of hardware debug and trace probes, debuggers, logic analyzers, and waveform generators and provides analysis tools, converters, and plugins to provide significant insight into the software and hardware state during or after execution.

The `emdbg` library contains [many useful GDB plugins](https://github.com/Auterion/embedded-debug-tools/blob/main/src/emdbg/debug/gdb.md#user-commands) that make debugging PX4 easier.
It also provides tools for [profiling PX4 in real-time](https://github.com/Auterion/embedded-debug-tools/tree/main/ext/orbetto).
