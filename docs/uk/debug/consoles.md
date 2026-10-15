# Консолі/Оболонки PX4

PX4 enables terminal access to the system through the [MAVLink Shell](../debug/mavlink_shell.md) and the [System Console](../debug/system_console.md).
DroneCAN peripheral nodes running PX4 firmware can also be accessed through the [DroneCAN Shell](../debug/dronecan_shell.md) (it can't be used to access the flight controller).

Ця сторінка пояснює основні відмінності та як використовується консоль/оболонка.

<a id="console_vs_shell"></a>

## Системна консоль у порівнянні з оболонкою

The PX4 _System Console_ provides low-level access to the system, debug output and analysis of the system boot process.

There is just one _System Console_, which runs on one specific UART (the debug port, as configured in NuttX), and is commonly attached to a computer via an FTDI cable (or some other debug adapter like a [Zubax BugFace BF1](https://github.com/Zubax/bugface_bf1)).

- Used for _low-level debugging/development_: bootup, NuttX, startup scripts, board bringup, development on central parts of PX4 (e.g. uORB).
- Зокрема, це єдине місце, де виводиться весь вивід завантаження (включаючи інформацію про програми, які автоматично запускаються при завантаженні).

Оболонки надають високорівневий доступ до системи:

- Використовується для базового тестування модулів/виконання команд.
- Only _directly_ display the output of modules you start.
- Cannot _directly_ display the output of tasks running on the work queue.
- Не може налагоджувати проблеми, коли система не запускається (оскільки вона ще не працює).

:::info
The `dmesg` command is now available through the shell on some boards, enabling much lower level debugging than previously possible.
For example, with `dmesg -f &` you also see the output of background tasks.
:::

There can be several shells, either running on a dedicated UART, or tunnelled over a link such as MAVLink or DroneCAN.
Most boards use the [MAVLink Shell](../debug/mavlink_shell.md); peripheral nodes with only a CAN connection can use the [DroneCAN Shell](../debug/dronecan_shell.md) (requires firmware built with `CONFIG_UAVCANNODE_COMMAND_SHELL`).

The [System Console](../debug/system_console.md) is essential when the system does not boot (it displays the system boot log when power-cycling the board).
The [MAVLink Shell](../debug/mavlink_shell.md) is much easier to setup, and so is more generally recommended for most debugging.

<a id="using_the_console"></a>

## Використання Консолі/Оболонки

The MAVLink shell, [DroneCAN Shell](../debug/dronecan_shell.md), and [System Console](../debug/system_console.md) are all used in much the same way.

For example, type `ls` to view the local file system, `free` to see the remaining free RAM, `dmesg` to look at boot output.

```sh
nsh> ls
nsh> free
nsh> dmesg
```

Below are a couple of commands which can be used in the [NuttShell](https://cwiki.apache.org/confluence/pages/viewpage.action?pageId=139629410) to get insights of the system.

Ця команда NSH надає доступну вільну пам'ять:

```sh
free
```

Команда top показує використання стеку для кожного додатку:

```sh
top
```

Зверніть увагу, що використання стеку обчислюється за допомогою алгоритму забарвлення стеку та є максимумом з моменту початку завдання (а не поточним використанням).

Щоб побачити, що виконується у робочих чергах і з якою швидкістю, використовуйте:

```sh
work_queue status
```

Для налагодження тем uORB:

```sh
uorb top
```

Для перевірки певної рубрики uORB:

```sh
listener <topic_name>
```

Many other system commands and modules are listed in the [Modules and Command Reference](../modules/modules_main.md) (e.g. `top`, `listener`, etc.).

:::tip
Some commands may be disabled on some boards (i.e. the some modules are not included in firmware for boards with RAM or FLASH constraints).
In this case you will see the response: `command not found`
:::
