# Poor Man's Sampling Profiler

This section describes how you can use the [Poor Man's Sampling Profiler](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/nuttx/Debug/poor-mans-profiler.sh) (PMSP) shell script to assess the performance of PX4.
This is an implementation of a known method originally invented by [Mark Callaghan and Domas Mituzas](https://poormansprofiler.org/).

## Підхід

PMSP - це оболонковий сценарій, який працює шляхом переривання виконання прошивки періодично для збору поточного стеку викликів.
Відмічені трасування стеку додаються в текстовий файл.
Once sampling is finished (which normally takes about an hour or more), the collected stack traces are _folded_.
The result of _folding_ is another text file that contains the same stack traces, except that all similar stack traces (i.e. those that were obtained at the same point in the program) are joined together, and the number of their occurrences is recorded.
The folded stacks are then fed into the visualization script, for which purpose we employ [FlameGraph - an open source stack trace visualizer](http://www.brendangregg.com/flamegraphs.html).

## Основне використання

### Вимоги

Профілер покладається на GDB для запуску PX4 на вбудованій цілі.
Так що перед профілюванням цілі, вам потрібно мати обладнання, яке ви хочете профілювати, і вам потрібно скомпілювати та завантажити прошивку на це обладнання.
You will then need a [debug probe](../debug/swd_debug.md#debug-probes) (such as the DroneCode Probe), to run the GDB server and interact with the board.

### Визначення пристрою відладки

The `poor-mans-profiler.sh` automatically detects and uses the correct USB device if you use it with a [DroneCode Probe](../debug/probe_bmp.md#dronecode-probe).
If you use a different kind of probe you may need to pass in the specific _device_ on which the debugger is located.
You can use the bash command `ls -alh /dev/serial/by-id/` to enumerate the possible devices on Ubuntu.
Наприклад, наступні пристрої перераховані з підключеними через USB Pixhawk 4 та DroneCode Probe:

```sh
user@ubuntu:~/PX4-Autopilot$ ls -alh /dev/serial/by-id/
total 0
drwxr-xr-x 2 root root 100 Apr 23 18:57 .
drwxr-xr-x 4 root root  80 Apr 23 18:48 ..
lrwxrwxrwx 1 root root  13 Apr 23 18:48 usb-3D_Robotics_PX4_FMU_v5.x_0-if00 -> ../../ttyACM0
lrwxrwxrwx 1 root root  13 Apr 23 18:57 usb-Black_Sphere_Technologies_Black_Magic_Probe_BFCCB401-if00 -> ../../ttyACM1
lrwxrwxrwx 1 root root  13 Apr 23 18:57 usb-Black_Sphere_Technologies_Black_Magic_Probe_BFCCB401-if02 -> ../../ttyACM2
```

In this case, the script would automatically pick up the device named `*Black_Magic_Probe*-if00`.
Але якщо ви використовуєте інший пристрій, ви зможете знайти відповідний ідентифікатор у вищезазначеному переліку.

Then pass in the appropriate device using the `--gdbdev` argument like this:

```sh
./poor-mans-profiler.sh --elf=build/px4_fmu-v4_default/px4_fmu-v4_default.elf --nsamples=30000 --gdbdev=/dev/ttyACM2
```

### Запуск

Основне використання профілера доступне через систему збірки.
For example, the following command builds and profiles px4_fmu-v4pro target with 10000 samples (fetching _FlameGraph_ and adding it to the path as needed).

```sh
make px4_fmu-v4pro_default profile
```

For more control over the build process, including setting the number of samples, see the [Implementation](#implementation).

## Розуміння виводу

Знизу наведено знімок екрану прикладового виводу (зверніть увагу, що тут він не є інтерактивним):

![FlameGraph Example](../../assets/debug/flamegraph-example.png)

На графіку пламені, горизонтальні рівні представляють рамки стеку, тоді як ширина кожної рамки пропорційна кількості разів, коли вона була вибрана.
Зі свого боку, кількість разів, коли функцію виявилися вибраною, пропорційна тривалості множеній на частоту її виконання.

## Можливі проблеми

Сценарій був розроблений як тимчасове рішення, тому він має деякі проблеми.
Будь ласка, будьте обережні з ними під час використання:

- Якщо GDB працює некоректно, скрипт може не виявити це і продовжити виконання.
  У цьому випадку, очевидно, не буде вироблено жодних придатних стеків.
  In order to avoid that, the user should periodically check the file `/tmp/pmpn-gdberr.log`, which contains the stderr output of the most recent invocation of GDB.
  У майбутньому сценарій слід змінити так, щоб він викликав GDB у тихому режимі, де він буде вказувати проблеми через свій код виходу.

- Іноді GDB просто залишається назавжди, поки відбувається вибіркове збереження стеку.
  Під час цієї несправності ціль буде припинена на невизначений термін.
  The solution is to manually abort the script and re-launch it again with the `--append` option.
  У майбутньому сценарій слід змінити, щоб накладати тайм-аут на кожне викликання GDB.

- Багатопотокові середовища не підтримуються.
  Це не впливає на однопроцесорні вбудовані цілі, оскільки вони завжди виконуються в одному потоці, але це обмеження робить профілер несумісним з багатьма іншими програмами.
  У майбутньому папку стеку слід модифікувати для підтримки кількох стеків викликів на один зразок.

## Імплементація

The script is located at [/platforms/nuttx/Debug/poor-mans-profiler.sh](https://github.com/PX4/PX4-Autopilot/blob/main/platforms/nuttx/Debug/poor-mans-profiler.sh)
Once launched, it will perform the specified number of samples with the specified time interval.
Collected samples will be stored in a text file in the system temp directory (typically `/tmp`).
Після завершення вибіркового відбору сценарій автоматично викличе папку стеку, вихід якої буде збережено в сусідньому файлі в тимчасовому каталозі.
If the stacks were folded successfully, the script will invoke the _FlameGraph_ script and store the result in an interactive SVG file.
Зверніть увагу, що не всі переглядачі зображень підтримують інтерактивні зображення;
рекомендується відкрити отриманий SVG у веб-переглядачі.

The FlameGraph script must reside in the `PATH`, otherwise PMSP will refuse to launch.

PMSP використовує GDB для збору стек-відстежень.
Currently it uses `arm-none-eabi-gdb`, other toolchains may be added in the future.

Для того щоб мати можливість відображати місця розташування пам'яті на символи, сценарій повинен посилатися на виконуваний файл, який в даний момент працює на цільовому пристрої.
This is done with the help of the option `--elf=<file>`, which expects a path (relative to the root of the repository) pointing to the location of the currently executing ELF.

Приклад використання:

```sh
./poor-mans-profiler.sh --elf=build/px4_fmu-v4_default/px4_fmu-v4_default.elf --nsamples=30000
```

Зверніть увагу, що кожен запуск скрипта перезапише старі стеки.
Should you want to append to the old stacks rather than overwrite them, use the option `--append`:

```sh
./poor-mans-profiler.sh --elf=build/px4_fmu-v4_default/px4_fmu-v4_default.elf --nsamples=30000 --append
```

As one might suspect, `--append` with `--nsamples=0` will instruct the script to only regenerate the SVG without accessing the target at all.

Будь ласка, прочитайте сценарій для більш глибокого розуміння того, як це працює.
