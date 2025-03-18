# Середовище розробки Windows (засноване на Cygwin)

:::warning
This development environment is [community supported and maintained](../advanced/community_supported_dev_env.md).
Це може працювати або не працювати з поточними версіями PX4.

Цей інструментарій був рекомендований раніше, але наразі не працює з PX4 v1.12 і новіше через проблеми з пакетами.
The [Windows WSL2-Based Development Environment](../dev_setup/dev_env_windows_wsl.md) should be used by preference.

Дивіться [Встановлення інструментарію](../dev_setup/dev_env.md) для інформації про середовища та інструменти, що підтримуються основною командою розробників.
:::

Наступні інструкції пояснюють як налаштувати (на базі Cygwin) середовище розробки на Windows 10 для PX4.
Це середовище може бути використане для збірки PX4 для:

- Pixhawk та іншого апаратного забезпечення на основі NuttX
- [jMAVSim Simulation](../sim_jmavsim/index.md)

<a id="installation"></a>

## Інструкції з установки

1. Download the latest version of the ready-to-use MSI installer from: [Github releases](https://github.com/PX4/windows-toolchain/releases) or [Amazon S3](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.9.msi) (fast download).

2. Запустіть, оберіть потрібне місце установки, дочекайтесь встановлення:

   ![jMAVSimOnWindows](../../assets/toolchain/cygwin_toolchain_installer.png)

3. Tick the box at the end of the installation to _clone the PX4 repository, build and run simulation with jMAVSim_ (this simplifies the process to get you started).

   ::: info
   If you missed this step you will need to [clone the PX4-Autopilot repository manually](#getting-started).

:::

:::warning
At time of writing the installer is missing some dependencies (and cannot yet be rebuilt to add them - see [PX4-windows-toolchain#31](https://github.com/PX4/PX4-windows-toolchain/issues/31)).

Щоб додати їх самостійно:

1. Перейдіть в директорію встановлення інструментів (за замовчуванням **C:\\PX4\\**)
2. Run **run-console.bat** (double click) to start the linux-like Cygwin bash console
3. Введіть в консолі наступну команду:

   ```sh
   pip3 install --user kconfiglib jsonschema future
   ```

:::

## Початок роботи

The toolchain uses a specially configured console window (started by running the **run-console.bat** script) from which you can call the normal PX4 build commands:

1. Перейдіть в директорію встановлення інструментів (за замовчуванням **C:\\PX4\\**)

2. Run **run-console.bat** (double click) to start the linux-like Cygwin bash console (you must use this console to build PX4).

3. Клонуйте репозиторій PX4 PX4-Autopilot з цієї консолі:

   ::: info
   Skip this step if you ticked the installer option to _clone the PX4 repository, build and run simulation with jMAVSim_.
   Клонування потрібно зробити тільки один раз!

:::

   ```sh
   # Clone the PX4-Autopilot repository into the home folder & loads submodules in parallel
   git clone --recursive -j8 https://github.com/PX4/PX4-Autopilot.git
   ```

   Тепер можна використовувати консоль/PX4-Autopilot репозиторій для збірки PX4.

4. Наприклад, для запуску JMAVSim:

   ```sh
   # Navigate to PX4-Autopilot repo
   cd Firmware
   # Build and runs SITL simulation with jMAVSim to test the setup
   make px4_sitl jmavsim
   ```

   Після цього консоль покаже:

   ![jMAVSimOnWindows](../../assets/simulation/jmavsim/jmavsim_windows_cygwin.png)

## Наступні кроки

Після того, як ви закінчите налаштування інструментів командного рядка:

- Install the [QGroundControl Daily Build](../dev_setup/qgc_daily_build.md)
- Continue to the [build instructions](../dev_setup/building_px4.md).

## Усунення проблем

### Інструменти моніторингу файлів проти швидкості інструментарію

Антивірус та інші інструменти моніторингу файлів у фоні можуть суттєво сповільнити встановлення інструментів та збірки PX4.

Можливо знадобиться тимчасово зупинити їх під час збірки (на власний розсуд).

### Windows & Git Special Cases

#### Символи закінчення рядків Windows CR+LF проти Unix LF

Ми рекомендуємо примусити використання закінчення рядків Unix LF для кожного репозиторію, з яким ви працюєте з цим інструментарієм (також використовуйте редактор, який залишає  їх незмінними під час збереження змін, тобто Eclipse або VSCode).
Compilation of source files also works with CR+LF endings checked out locally, but there are cases in Cygwin (e.g. execution of shell scripts) that require Unix line endings (otherwise you get errors like `$'\r': Command not found.`).
На щастя, git може це зробити для вас, якщо виконаєте дві команди в кореневому каталозі вашого репозиторію:

```sh
git config core.autocrlf false
git config core.eol lf
```

Якщо ви працюєте з цим інструментарієм в декількох репозиторіях, ви також можете встановити ці дві конфігурації глобально для вашого комп'ютера:

```sh
git config --global ...
```

Це не рекомендується, оскільки це може вплинути на будь-які інші (непов'язані) випадки використання git на вашій Windows машині.

#### Біт виконання прав доступу Unix

Під Unix є прапорець в правах доступу кожного файлу, який вказує на те, чи дозволено файлу виконуватись.
_git_ under Cygwin supports and cares about that bit (even though the Windows NTFS file system does not use it).
This often results in _git_ finding "false-positive" differences in permissions.
Вивід diff в результаті може виглядати так:

```sh
diff --git ...
old mode 100644
new mode 100755
```

Ми рекомендуємо відключити глобально перевірку прав доступу у Windows, щоб уникнути проблем:

```sh
# глобально відключити перевірку біту виконання для комп'ютера
git config --global core.fileMode false
```

Для наявних репозиторіїв, що мають цю проблему викликану локальною конфігурацією, додатково виконайте:

```sh
# прибрати локальну конфігурацію, щоб застосувати глобальну
git config --unset core.filemode

# прибрати локальну конфігурацію для всіх підмодулів
git submodule foreach --recursive git config --unset core.filemode
```

<!--
Instructions for building/updating this toolchain are covered in [Windows Cygwin Development Environment (Maintenance Instructions)](../dev_setup/dev_env_windows_cygwin_packager_setup.md)
-->
