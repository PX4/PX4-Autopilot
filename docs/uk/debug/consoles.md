# Консолі/Оболонки PX4

PX4 enables terminal access to the system through the [MAVLink Shell](../debug/mavlink_shell.md) and the [System Console](../debug/system_console.md).

Ця сторінка пояснює основні відмінності та як використовується консоль/оболонка.

<a id="console_vs_shell"></a>

## Системна консоль у порівнянні з оболонкою

The PX4 _System Console_ provides low-level access to the system, debug output and analysis of the system boot process.

There is just one _System Console_, which runs on one specific UART (the debug port, as configured in NuttX), and is commonly attached to a computer via an FTDI cable (or some other debug adapter like a [Dronecode probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation)).

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

Може бути кілька оболонок, які працюють на відведеному UART або через MAVLink.
Since MAVLink provides more flexibility, currently only the [MAVLink Shell](../debug/mavlink_shell.md) is used.

The [System Console](../debug/system_console.md) is essential when the system does not boot (it displays the system boot log when power-cycling the board).
The [MAVLink Shell](../debug/mavlink_shell.md) is much easier to setup, and so is more generally recommended for most debugging.

<a id="using_the_console"></a>

## Використання Консолі/Оболонки

The MAVLink shell/console and the [System Console](../debug/system_console.md) are used in much the same way.

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
