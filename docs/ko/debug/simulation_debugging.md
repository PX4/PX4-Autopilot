# 시뮬레이션 디버깅

호스트 시스템에서 시뮬레이션이 실행 중이므로, 데스크탑 개발 도구를 사용할 수 있습니다.

## CLANG 주소 새니타이저(Mac OS, Linux)

Clang 주소 새니타이저는 정렬(버스) 오류 및 분할 오류와 같은 기타 메모리 오류를 찾는 데 도움이 됩니다. 아래 명령은 올바른 컴파일 옵션을 설정합니다.

```sh
make clean # only required on first address sanitizer run after a normal build
PX4_ASAN=1 make px4_sitl jmavsim
```

## Valgrind

```sh
brew install valgrind
```

또는

```sh
sudo apt-get install valgrind
```

SITL 시뮬레이션 중에 valgrind를 사용하려면:

```sh
make px4_sitl_default jmavsim___valgrind
```

## Launch Gazebo Classic SITL Without Debugger

By default SITL is launched without a debugger attached when using any simulator backend:

```sh
make px4_sitl_default gz
make px4_sitl_default gazebo-classic
make px4_sitl_default jmavsim
```

For Gazebo Classic (only) you can also start the simulator with a debugger attached.
Note however, that you must provide the vehicle type in the simulator target, as shown below:

```sh
make px4_sitl_default gazebo-classic_iris_gdb
make px4_sitl_default gazebo-classic_iris_lldb
```

This will start the debugger and launch the SITL application with Gazebo and the Iris simulator.
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

또는 GDB의 경우:

```sh
(gdb) handle SIGCONT noprint nostop
```

그 후 lldb 또는 gdb 셸은 일반 세션처럼 작동합니다. LLDB/GDB 문서를 참고하십시오.

The last parameter, the &lt;viewer_model_debugger&gt; triplet, is actually passed to make in the build directory, so

```sh
make px4_sitl_default gazebo-classic_iris_gdb
```

명령은 다음 명령과 같습니다.

```sh
make px4_sitl_default	# Configure with cmake
make -C build/px4_sitl_default classic_iris_gdb
```

A full list of the available make targets in the build directory can be obtained with:

```sh
make help
```

## Attaching GDB to running SITL

You can also start your simulation, and _then_ attach `gdb`:

1. In one terminal screen enter the command to start your simulation:

   ```sh
   make px4_sitl_default gazebo-classic
   ```

   As the script runs, note the **SITL COMMAND:** output text located right above the large "PX4" text.
   It will list the location of your px4 bin file for later use.

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

2. Open another terminal and type:

   ```sh
   ps -a
   ```

   You will want to note the PID of the process named "PX4"

   (In this example it is 14149)

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

3. Then type in the same window

   ```sh
   sudo gdb [px4 bin file path (from step 1) here]
   ```

   예를 들어,

   ```sh
   sudo gdb /home/atlas/px4/base/PX4-Autopilot/build/px4_sitl_default/bin/px4
   ```

   Now, you can attach to the PX4 instance by entering the PID noted in step 2.

   ```sh
   attach [PID on px4]
   ```

   You should now have a GDB interface to debug with.

## 컴파일러 최적화

It is possible to suppress compiler optimization for given executables and/or modules (as added by cmake with `add_executable` or `add_library`) when configuring
for `posix_sitl_*`.
This can be handy when it is necessary to step through code with a debugger or print variables that would otherwise be optimized out.

To do so, set the environment variable `PX4_NO_OPTIMIZATION` to be a semi-colon separated list of regular expressions that match the targets that need to be compiled without optimization.
This environment variable is ignored when the configuration isn&#39;t `posix_sitl_*`.

예를 들어,

```sh
export PX4_NO_OPTIMIZATION='px4;^modules__uORB;^modules__systemlib$'
```

would suppress optimization of the targets: platforms\_\_posix\_\_px4_layer, modules\_\_systemlib, modules\_\_uORB, examples\_\_px4_simple_app, modules\_\_uORB\_\_uORB_tests and px4.

The targets that can be matched with these regular expressions can be printed with the command:

```sh
make -C build/posix_sitl_* list_cmake_targets
```
