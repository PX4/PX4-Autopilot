# Відлагодження з GDB

The [GNU DeBugger (GDB)](https://sourceware.org/gdb/documentation/) comes installed with the compiler toolchain in the form of the `arm-none-eabi-gdb` binary.
Налагоджувач читає символи відладки у файлі формату виконання ELF щоб зрозуміти статичну та динамічну структуру пам'яті прошивки PX4.
To access the PX4 autopilot microcontroller, it needs to connect to a [Remote Target](https://sourceware.org/gdb/current/onlinedocs/gdb.html/Connecting.html), which is provided by a [SWD debug probe](swd_debug.md).

Плин інформації виглядає таким чином:

```sh
Developer <=> GDB <=> GDB Server <=> Debug Probe <=> SWD <=> PX4 Autopilot.
```

## Швидкий старт

Для початку сеансу налагодження вам зазвичай потрібно:

1. Need a specialized [SWD debug probe](../debug/swd_debug.md#debug-probes).
2. Find and connect to the [SWD debug port](../debug/swd_debug.md#autopilot-debug-ports).
   You may need a [debug adapter](swd_debug.md#debug-adapters).
3. Налаштувати та запустити зонд налагодження для створення сервера GDB.
4. Запустити GDB та під'єднатись до сервера GDB як віддаленої цілі.
5. Налагоджувати прошивку інтерактивно.

Дивіться документацію зонда налагодження для додаткової інформації як налаштувати з'єднання для налагодження:

- [SEGGER J-Link](probe_jlink.md): commercial probe, no built-in serial console, requires adapter.
- [Black Magic Probe](probe_bmp.md): integrated GDB server and serial console, requires adapter.
- [STLink](probe_stlink): best value, integrated serial console, adapter must be soldered.

Рекомендуємо використовувати J-Link з адаптером налагодження Pixhawk або STLinkv3-MINIE зі спеціально спаяним кабелем.

Після підключення ви можете використовувати звичайні команди GDB, такі як:

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

## Наступні кроки

Ви під'єднали контролер польоту до налагоджувального зонда SWD!

Наступні розділи пояснюють, як розпочати налагодження на цільовій платформі:

- [MCU Eclipse/J-Link Debugging for PX4](eclipse_jlink.md).
- [Visual Studio Code IDE (VSCode)](../dev_setup/vscode.md).

## Відео

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

## Вбудовані інструменти налагодження

The [Embedded Debug Tools](https://pypi.org/project/emdbg/) connect several software and hardware debugging tools together in a user friendly Python package to more easily enable advanced use cases for ARM Cortex-M microcontrollers and related devices.

Ця бібліотека організовує запуск та налаштування апаратних зондів налагодження та зондів трасування, налагоджувачів, логічних аналізаторів, генераторів сигналу та надає інструменти аналізу, перетворювачі та плагіни для отримання суттєвого розуміння стану програмного та апаратного забезпечення під час або після виконання.

The `emdbg` library contains [many useful GDB plugins](https://github.com/Auterion/embedded-debug-tools/blob/main/src/emdbg/debug/gdb.md#user-commands) that make debugging PX4 easier.
It also provides tools for [profiling PX4 in real-time](https://github.com/Auterion/embedded-debug-tools/tree/main/ext/orbetto).
