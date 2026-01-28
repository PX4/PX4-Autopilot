# Консоль MAVLink

The MAVLink Shell is an _NSH console_ that can be accessed via MAVLink over serial (USB/Telemetry) or WiFi (UDP/TCP) links (in particular, on NuttX-based systems like: Pixhawk, Pixracer, etc.).

Консоль можна використовувати для виконання команд та модулів та відображення їх виводу.
While the shell cannot _directly_ display the output of modules that it does not start, it can do so indirectly using the `dmesg` command (`dmesg -f &` can be used to display the output of other modules and tasks running on the work queue).

:::tip
The [QGroundControl MAVLink Console](#qgroundcontrol) is the easiest way to access the console.
If the system does not start properly you should instead use the [System Console](../debug/system_console.md).
:::

## Відкриття консолі

<a id="qgroundcontrol"></a>

### QGroundControl MAVLink Console

The easiest way to access shell is to use the [QGroundControl MAVLink Console](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_console.html) (see **Analyze View > Mavlink Console**).

### mavlink_shell.py

You can also access the shell in a terminal using the **mavlink_shell.py** script:

1. Shut down _QGroundControl_.

2. Встановіть залежності:

   ```sh
   pip3 install --user pymavlink pyserial
   ```

3. Відкрийте термінал (у каталозі PX4-Autopilot) та запустіть оболонку:

   ```sh
   # For serial port
   ./Tools/mavlink_shell.py /dev/ttyACM0
   ```

   ```sh
   # For Wifi connection
   ./Tools/mavlink_shell.py 0.0.0.0:14550
   ```

Use `mavlink_shell.py -h` to get a description of all available arguments.

## Використання консолі MAVLink

For information see: [PX4 Consoles/Shells > Using Consoles/Shells](../debug/consoles.md#using_the_console).
