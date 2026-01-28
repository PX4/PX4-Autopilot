# Debugging with GDB

The [GNU DeBugger (GDB)](https://sourceware.org/gdb/documentation/) comes installed with the compiler toolchain in the form of the `arm-none-eabi-gdb` binary.
The debugger reads the debug symbols inside an ELF file to understand the static and dynamic memory layout of the PX4 firmware.
To access the PX4 autopilot microcontroller, it needs to connect to a [Remote Target](https://sourceware.org/gdb/current/onlinedocs/gdb.html/Connecting.html), which is provided by a [SWD debug probe](swd_debug.md).

The flow of information looks like this:

```sh
Developer <=> GDB <=> GDB Server <=> Debug Probe <=> SWD <=> PX4 Autopilot.
```

## Quickstart

To start a debugging session you typically:

1. Need a specialized [SWD debug probe](../debug/swd_debug.md#debug-probes).
2. Find and connect to the [SWD debug port](../debug/swd_debug.md#autopilot-debug-ports).
   You may need a [debug adapter](swd_debug.md#debug-adapters).
3. Configure and start the debug probe to create a GDB server.
4. Launch GDB and connect to the GDB server as a remote target.
5. Debug your firmware interactively.

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

## Next Steps

You've now connected the flight controller to an SWD debug probe!

The following topics explain how to start on-target debugging:

- [MCU Eclipse/J-Link Debugging for PX4](eclipse_jlink.md).
- [Visual Studio Code IDE (VSCode)](../dev_setup/vscode.md).

## Video

The following video provides an overview of the tooling available for advanced debugging of PX4 via GDB.
It was presented at the PX4 Developer Conference 2023.

<lite-youtube videoid="1c4TqEn3MZ0" title="Debugging PX4 - Niklas Hauser, Auterion AG"/>

**Overview:** The inspection tools built into PX4 via Mavlink Shell (NSH) as well as interpretation of the PX4 uLog after a flight require PX4 to still be functioning. However, the most problematic bugs often manifest themselves in a (partially) hanging or crashed system. Therefore, we present the open-source Embedded Debug Tools project, which manages and configures probe, debugging and analysis tools for PX4 and NuttX:

- Debug interfaces (SWD) and the associated debug probes (J-Link, STLink) and libraries (JLinkGDBServer, OpenOCD).
- How to install and configure `arm-none-eabi-gdb(-py3)` for debugging your ELF.
- Commonly used GDB commands and scripts.
- Advanced GDB scripting via its Python API.
- Inspection of NuttX RTOS component internals: tasks, semaphores, scheduler.
- Inspecting peripheral state with CMSIS-SVD files and custom visualizations.
- Coredumping for post-mortem debugging via CrashDebug.
- Hardfault analysis in a live system and via the hardfault log.
- Remote GDB scripting via the Machine Interface.
- Automated HiL testing of PX4 via combined GDB and NSH scripting.
- ITM profiling over SWO pin using Orbuculum.
- Thread/IRQ/Workqueue/Heap visualization and latency analysis using perfetto.
- High-bandwidth ETM tracing over TRACE pins: J-Trace and ORBtrace mini.
- We conclude with an overview of interesting related project and an outlook on the future of PX4 debugging.

## Embedded Debug Tools

The [Embedded Debug Tools](https://pypi.org/project/emdbg/) connect several software and hardware debugging tools together in a user friendly Python package to more easily enable advanced use cases for ARM Cortex-M microcontrollers and related devices.

The library orchestrates the launch and configuration of hardware debug and trace probes, debuggers, logic analyzers, and waveform generators and provides analysis tools, converters, and plugins to provide significant insight into the software and hardware state during or after execution.

The `emdbg` library contains [many useful GDB plugins](https://github.com/Auterion/embedded-debug-tools/blob/main/src/emdbg/debug/gdb.md#user-commands) that make debugging PX4 easier.
It also provides tools for [profiling PX4 in real-time](https://github.com/Auterion/embedded-debug-tools/tree/main/ext/orbetto).
