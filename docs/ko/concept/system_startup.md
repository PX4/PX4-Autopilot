# 시스템 시작

PX4 시작은 쉘 스크립트에 의해 제어됩니다.
On NuttX they reside in the [ROMFS/px4fmu_common/init.d](https://github.com/PX4/PX4-Autopilot/tree/main/ROMFS/px4fmu_common/init.d) folder - some of these are also used on Posix (Linux/MacOS).
The scripts that are only used on Posix are located in [ROMFS/px4fmu_common/init.d-posix](https://github.com/PX4/PX4-Autopilot/tree/main/ROMFS/px4fmu_common/init.d-posix).

All files starting with a number and underscore (e.g. `10000_airplane`) are predefined airframe configurations.
They are exported at build-time into an `airframes.xml` file which is parsed by [QGroundControl](http://qgroundcontrol.com) for the airframe selection UI.
Adding a new configuration is covered [here](../dev_airframes/adding_a_new_frame.md).

나머지 파일은 공통 시작 로직의 일부입니다.
The first executed file is the [init.d/rcS](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d/rcS) script (or [init.d-posix/rcS](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/rcS) on Posix), which calls all other scripts.

다음 섹션은 PX4가 실행되는 운영 체제에 따라 달라집니다.

## POSIX (Linux/MacOS)

Posix에서 시스템 셸은 스크립트 인터프리터로 사용됩니다(예: /bin/sh, Ubuntu에서 dash에 심볼릭 링크됨).
동작하기 위한 몇가지 조건이 있습니다.

- PX4 모듈은 시스템에서 개별적으로 실행할 수 있어야합니다.
  이 동작은 심볼릭 링크로 처리합니다.
  For each module a symbolic link `px4-<module> -> px4` is created in the `bin` directory of the build folder.
  When executed, the binary path is checked (`argv[0]`), and if it is a module (starts with `px4-`), it sends the command to the main px4 instance (see below).

  :::tip
  The `px4-` prefix is used to avoid conflicts with system commands (e.g. `shutdown`), and it also allows for simple tab completion by typing `px4-<TAB>`.

:::

- 쉘은 심볼릭 링크를 찾을 위치를 알고 있어야 합니다.
  For that the `bin` directory with the symbolic links is added to the `PATH` variable right before executing the startup scripts.

- 쉘은 각 모듈을 새로운(클라이언트) 프로세스로 시작합니다.
  각 클라이언트 프로세스는 실제 모듈이 스레드로 실행되는 px4(서버)의 기본 인스턴스와 통신합니다.
  This is done through a [UNIX socket](http://man7.org/linux/man-pages/man7/unix.7.html).
  서버는 클라이언트가 연결하고 명령을 보낼 수 있는 소켓으로 수신 대기합니다.
  그런 다음 서버는 출력과 반환 코드를 다시 클라이언트로 전송합니다.

- The startup scripts call the module directly, e.g. `commander start`, rather than using the `px4-` prefix.
  This works via aliases: for each module an alias in the form of `alias <module>=px4-<module>` is created in the file `bin/px4-alias.sh`.

- The `rcS` script is executed from the main px4 instance.
  It does not start any modules, but first updates the `PATH` variable and then simply runs a shell with the `rcS` file as argument.

- 그 외에도, 다중 기체 시뮬레이션을 위하여 여러 서버 인스턴스를 시작할 수 있습니다.
  A client selects the instance via `--instance`.
  The instance is available in the script via `$px4_instance` variable.

모듈은 PX4가 시스템에서 실행 중이면, 터미널에서 실행할 수 있습니다.
예:

```sh
cd <PX4-Autopilot>/build/px4_sitl_default/bin
./px4-commander takeoff
./px4-listener sensor_accel
```

### Dynamic Modules

일반적으로 모든 모듈은 단일 PX4 실행 파일로 컴파일됩니다.
However, on Posix, there's the option of compiling a module into a separate file, which can be loaded into PX4 using the `dyn` command.

```sh
dyn ./test.px4mod
```

## NuttX

NuttX has an integrated shell interpreter ([NuttShell (NSH)](https://cwiki.apache.org/confluence/pages/viewpage.action?pageId=139629410)), and thus scripts can be executed directly.

### 시스템 부팅 디버깅

소프트웨어 구성 요소의 드라이버 오류로 인하여 부팅이 중단되지는 않습니다.
This is controlled via `set +e` in the startup script.

The boot sequence can be debugged by connecting the [system console](../debug/system_console.md) and power-cycling the board.
부팅 로그에는 부팅 순서에 대한 자세한 정보가 포함되어 있으며, 부팅 중단 이유에 대한 힌트가 포함되어 있습니다.

#### 일반적인 부팅 실패 사례

- For custom applications: The system was out of RAM.
  Run the `free` command to see the amount of free RAM.
- A software fault or assertion resulting in a stack trace

### 시스템 시작 변경

The whole boot can be replaced by creating a file `/etc/rc.txt` on the microSD card with a new configuration (nothing in the old configuration will be auto-started, and if the file is empty, nothing at all will be started).

Customizing the default boot is almost always a better approach.
This is documented below.

### 시스템 시작 사용자 정의

The best way to customize the system startup is to introduce a [new frame configuration](../dev_airframes/adding_a_new_frame.md).
The frame configuration file can be included in the firmware or on an SD Card.

If you only need to "tweak" the existing configuration, such as starting one more application or setting the value of a few parameters, you can specify these by creating two files in the `/etc/` directory of the SD Card:

- [/etc/config.txt](#customizing-the-configuration-config-txt): modify parameter values
- [/etc/extras.txt](#starting-additional-applications-extras-txt): start applications

The files are described below.

:::warning
The system boot files are UNIX FILES which require UNIX LINE ENDINGS.
Windows에서 편집하는 경우 적절한 편집기를 사용하여야 합니다.
:::

:::info
These files are referenced in PX4 code as `/fs/microsd/etc/config.txt` and `/fs/microsd/etc/extras.txt`, where the root folder of the microsd card is identified by the path `/fs/microsd`.
:::

#### 구성 사용자 정의(config.txt)

The `config.txt` file can be used to modify parameters.
It is loaded after the main system has been configured and _before_ it is booted.

For example, you could create a file on the SD card, `etc/config.txt` with that sets parameter values as shown:

```sh
param set-default PWM_MAIN_DIS3 1000
param set-default PWM_MAIN_MIN3 1120
```

#### Starting Additional Applications (extras.txt)

The `extras.txt` can be used to start additional applications after the main system boot.
일반적으로, 페이로드 콘트롤러나 유사한 선택적 사용자 지정 구성 요소들입니다.

:::warning
Calling an unknown command in system boot files may result in boot failure.
일반적으로 시스템은 부팅 실패 후 mavlink 메시지를 스트리밍하지 않습니다. 이 경우 시스템 콘솔에 인쇄된 오류 메시지를 확인하여야 합니다.
:::

다음 예는 사용자 정의 애플리케이션 시작 방법을 설명합니다.

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
