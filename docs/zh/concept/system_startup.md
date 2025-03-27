# 系统启动

PX4 系统的启动由 shell 脚本文件控制。
On NuttX they reside in the [ROMFS/px4fmu_common/init.d](https://github.com/PX4/PX4-Autopilot/tree/main/ROMFS/px4fmu_common/init.d) folder - some of these are also used on Posix (Linux/MacOS).
The scripts that are only used on Posix are located in [ROMFS/px4fmu_common/init.d-posix](https://github.com/PX4/PX4-Autopilot/tree/main/ROMFS/px4fmu_common/init.d-posix).

All files starting with a number and underscore (e.g. `10000_airplane`) are predefined airframe configurations.
They are exported at build-time into an `airframes.xml` file which is parsed by [QGroundControl](http://qgroundcontrol.com) for the airframe selection UI.
Adding a new configuration is covered [here](../dev_airframes/adding_a_new_frame.md).

其它的文件则是系统常规启动逻辑的一部分。
The first executed file is the [init.d/rcS](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d/rcS) script (or [init.d-posix/rcS](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/rcS) on Posix), which calls all other scripts.

根据 PX4 运行的操作系统将本文后续内容分成了如下各小节。

## Posix (Linux/MacOS)

在 Posix 操作系统上，系统的 shell 将会作为脚本文件的解释器（例如， 在 Ubuntu 中 /bin/sh 与 Dash 建立了符号链接）。
为了使 PX4 可以在 Posix 中正常运行，需要做到以下几点：

- PX4 的各个模块需要看起来像系统的单个可执行文件。
  这一点可以通过创建符号链接做到。
  For each module a symbolic link `px4-<module> -> px4` is created in the `bin` directory of the build folder.
  When executed, the binary path is checked (`argv[0]`), and if it is a module (starts with `px4-`), it sends the command to the main px4 instance (see below).

  :::tip
  The `px4-` prefix is used to avoid conflicts with system commands (e.g. `shutdown`), and it also allows for simple tab completion by typing `px4-<TAB>`.

:::

- Shell 需要知道在那里可以找到上述符号链接。
  For that the `bin` directory with the symbolic links is added to the `PATH` variable right before executing the startup scripts.

- Shell 将每个模块作为一个新的 (客户端) 进程进行启动，
  每个客户端进程都需要与 PX4 主实例（服务器）进行通讯，实际的模块以线程的形式运行。
  This is done through a [UNIX socket](http://man7.org/linux/man-pages/man7/unix.7.html).
  服务器侦听一个 socket，然后客户端将连接该 socket 并通过它发送指令。
  服务器收到客户端的指令后将指令运行的输出结果及返回代码重新发送给客户端。

- The startup scripts call the module directly, e.g. `commander start`, rather than using the `px4-` prefix.
  This works via aliases: for each module an alias in the form of `alias <module>=px4-<module>` is created in the file `bin/px4-alias.sh`.

- The `rcS` script is executed from the main px4 instance.
  It does not start any modules, but first updates the `PATH` variable and then simply runs a shell with the `rcS` file as argument.

- 除此之外，在进行多飞行器仿真时还可以启动多个服务器实例。
  A client selects the instance via `--instance`.
  The instance is available in the script via `$px4_instance` variable.

当 PX4 在操作系统上处于运行状态时可以从任意终端直接运行各个模块。
例如：

```sh
cd <PX4-Autopilot>/build/px4_sitl_default/bin
./px4-commander takeoff
./px4-listener sensor_accel
```

### Dynamic Modules

通常，所有模块都被编入一个 PX4 可执行程序。
However, on Posix, there's the option of compiling a module into a separate file, which can be loaded into PX4 using the `dyn` command.

```sh
dyn ./test.px4mod
```

## NuttX

NuttX has an integrated shell interpreter ([NuttShell (NSH)](https://cwiki.apache.org/confluence/pages/viewpage.action?pageId=139629410)), and thus scripts can be executed directly.

### 替换系统的启动文件

软件组件的失效不会中止 PX4 系统的启动，
This is controlled via `set +e` in the startup script.

The boot sequence can be debugged by connecting the [system console](../debug/system_console.md) and power-cycling the board.
由此生成的启动引导日志文件中包含了引导序列的详细信息，同时也应包含了解释启动中止的线索。

#### 启动失败的常见原因

- For custom applications: The system was out of RAM.
  Run the `free` command to see the amount of free RAM.
- A software fault or assertion resulting in a stack trace

### 自定义系统的启动文件

The whole boot can be replaced by creating a file `/etc/rc.txt` on the microSD card with a new configuration (nothing in the old configuration will be auto-started, and if the file is empty, nothing at all will be started).

根据默认启动程序来进行定制化是一个比较好地开始。
文档如下。

### 自定义系统的启动文件

The best way to customize the system startup is to introduce a [new frame configuration](../dev_airframes/adding_a_new_frame.md).
机架配置文件可以在固件中，也可以在SD卡上。

If you only need to "tweak" the existing configuration, such as starting one more application or setting the value of a few parameters, you can specify these by creating two files in the `/etc/` directory of the SD Card:

- [/etc/config.txt](#customizing-the-configuration-config-txt): modify parameter values
- [/etc/extras.txt](#starting-additional-applications-extras-txt): start applications

文件具体信息在后面介绍。

:::warning
The system boot files are UNIX FILES which require UNIX LINE ENDINGS.
如果在Windows上编辑，需要使用合适的编辑器。
:::

:::info
These files are referenced in PX4 code as `/fs/microsd/etc/config.txt` and `/fs/microsd/etc/extras.txt`, where the root folder of the microsd card is identified by the path `/fs/microsd`.
:::

#### 自定义配置（config.txt）

The `config.txt` file can be used to modify parameters.
It is loaded after the main system has been configured and _before_ it is booted.

For example, you could create a file on the SD card, `etc/config.txt` with that sets parameter values as shown:

```sh
param set-default PWM_MAIN_DIS3 1000
param set-default PWM_MAIN_MIN3 1120
```

#### 启动附加应用程序 (extras.txt)

The `extras.txt` can be used to start additional applications after the main system boot.
通常，额外启动的将是有效载荷控制器或类似的可选自定义组件。

:::warning
Calling an unknown command in system boot files may result in boot failure.
通常情况下系统在启动失败后不会发送 mavlink 消息，在这种情况下请检查系统控制台上输出的的错误消息。
:::

下面的示例演示了如何启动自定义应用程序:

- Create a file on the SD card `etc/extras.txt` with this content:

  ```sh
  custom_app start
  ```

- A command can be made optional by gating it with the `set +e` and `set -e` commands:

  ```sh
  set +e
  optional_app start      # Will not result in boot failure if optional_app is unknown or fails
  set -e

  mandatory_app start     # Will abort boot if mandatory_app is unknown or fails
  ```
