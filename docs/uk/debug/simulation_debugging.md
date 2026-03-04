# Відлагодження симуляції

Поскільки симуляція виконується на машині-хості, всі інструменти розробки робочого стола доступні.

## CLANG Address Sanitizer (Mac OS, Linux)

Засіб очищення адрес Clang може допомогти знайти помилки вирівнювання (шини) та інші помилки пам’яті, такі як помилки сегментації. Команда нижче встановлює правильні параметри компіляції.

```sh
make clean # only required on first address sanitizer run after a normal build
PX4_ASAN=1 make px4_sitl jmavsim
```

## Valgrind

```sh
brew install valgrind
```

або

```sh
sudo apt-get install valgrind
```

Для використання valgrind під час симуляції SITL:

```sh
make px4_sitl_default jmavsim___valgrind
```

## Запустіть Gazebo Classic SITL без відлагоджувача

За замовчуванням SITL запускається без приєднаного відлагоджувача при використанні будь-якого симулятора:

```sh
make px4_sitl_default gz
make px4_sitl_default gazebo-classic
make px4_sitl_default jmavsim
```

Для Gazebo Classic (тільки) ви також можете запустити симулятор з прикріпленим відладчиком.
Зверніть увагу, що ви повинні вказати тип транспортного засобу в цільовому симуляторі, як показано нижче:

```sh
make px4_sitl_default gazebo-classic_iris_gdb
make px4_sitl_default gazebo-classic_iris_lldb
```

Це запустить відлагоджувач та запустить додаток SITL з Gazebo та симулятором Iris.
In order to break into the debugger shell and halt the execution, hit `CTRL-C`:

```sh
Process 16529 stopped
* thread #1: tid = 0x114e6d, 0x00007fff90f4430a libsystem_kernel.dylib`__read_nocancel + 10, name = 'px4', queue = 'com.apple.main-thread', stop reason = signal SIGSTOP
    frame #0: 0x00007fff90f4430a libsystem_kernel.dylib`__read_nocancel + 10
libsystem_kernel.dylib`__read_nocancel:
->  0x7fff90f4430a <+10>: jae    0x7fff90f44314            ; <+20>
    0x7fff90f4430c <+12>: movq   %rax, %rdi
    0x7fff90f4430f <+15>: jmp    0x7fff90f3fc53            ; cerror_nocancel
    0x7fff90f44314 <+20>: retq
(lldb)
```

In order to not have the DriverFrameworks scheduling interfere with the debugging session `SIGCONT` should be masked in LLDB and GDB:

```sh
(lldb) process handle SIGCONT -n false -p false -s false
```

Або у випадку GDB:

```sh
(gdb) обробка SIGCONT noprint nostop
```

Після цього оболонки lldb або gdb працюють як звичайні сеанси, будь ласка, зверніться до документації LLDB / GDB.

Останній параметр, триплет &lt;viewer_model_debugger&gt;, фактично передається make у каталозі збірки, отже

```sh
make px4_sitl_default gazebo-classic_iris_gdb
```

еквівалентний з

```sh
make px4_sitl_default	# Configure with cmake
make -C build/px4_sitl_default classic_iris_gdb
```

Повний список доступних цілей make в каталозі збірки можна отримати за допомогою:

```sh
make help
```

## Прикріплення GDB до запущеного SITL

You can also start your simulation, and _then_ attach `gdb`:

1. У одному термінальному вікні введіть команду для запуску вашої симуляції:

   ```sh
   make px4_sitl_default gazebo-classic
   ```

   As the script runs, note the **SITL COMMAND:** output text located right above the large "PX4" text.
   Він перерахує місце розташування вашого файлу px4 bin для подальшого використання.

   ```sh
   SITL COMMAND: "<px4 bin file>" "<build dir>"/etc

   ______  __   __    ___
   | ___ \ \ \ / /   /   |
   | |_/ /  \ V /   / /| |
   |  __/   /   \  / /_| |
   | |     / /^\ \ \___  |
   \_|     \/   \/     |_/

   px4 starting.

   INFO  [px4] startup script: /bin/sh etc/init.d-posix/rcS 0
   INFO  [init] found model autostart file as SYS_AUTOSTART=10015
   ```

2. Відкрийте ще один термінал та введіть:

   ```sh
   ps -a
   ```

   Вам слід зафіксувати PID процесу з назвою "PX4"

   (У цьому прикладі це 14149)

   ```sh
   atlas:~/px4/main/PX4-Autopilot$ ps -a
       PID TTY          TIME CMD
   1796 tty2     00:01:59 Xorg
   1836 tty2     00:00:00 gnome-session-b
   14027 pts/1    00:00:00 make
   14077 pts/1    00:00:00 sh
   14078 pts/1    00:00:00 cmake
   14079 pts/1    00:00:00 ninja
   14090 pts/1    00:00:00 sh
   14091 pts/1    00:00:00 bash
   14095 pts/1    00:01:23 gzserver
   14149 pts/1    00:02:48 px4
   14808 pts/2    00:00:00 ps
   ```

3. Потім введіть у тому ж вікні

   ```sh
   sudo gdb [px4 bin file path (from step 1) here]
   ```

   Наприклад,

   ```sh
   sudo gdb /home/atlas/px4/base/PX4-Autopilot/build/px4_sitl_default/bin/px4
   ```

   Тепер ви можете прикріпитися до екземпляра PX4, введіть PID, вказаний у кроці 2.

   ```sh
   attach [PID on px4]
   ```

   Тепер у вас повинен бути інтерфейс GDB для налагодження.

## Оптимізація компілятора

It is possible to suppress compiler optimization for given executables and/or modules (as added by cmake with `add_executable` or `add_library`) when configuring
for `posix_sitl_*`.
Це може бути зручним, коли необхідно пройтися по коду з відлагоджувачем або роздрукувати змінні, які інакше були б оптимізовані.

To do so, set the environment variable `PX4_NO_OPTIMIZATION` to be a semi-colon separated list of regular expressions that match the targets that need to be compiled without optimization.
This environment variable is ignored when the configuration isn&#39;t `posix_sitl_*`.

Наприклад,

```sh
export PX4_NO_OPTIMIZATION='px4;^modules__uORB;^modules__systemlib$'
```

would suppress optimization of the targets: platforms\_\_posix\_\_px4_layer, modules\_\_systemlib, modules\_\_uORB, examples\_\_px4_simple_app, modules\_\_uORB\_\_uORB_tests and px4.

Цілі, які можуть відповідати цим регулярним виразам, можна надрукувати за допомогою команди:

```sh
make -C build/posix_sitl_* list_cmake_targets
```
