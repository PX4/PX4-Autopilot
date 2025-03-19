# Налагодження з Eclipse та J-Link

This topic explains how to setup and use [MCU Eclipse](https://gnu-mcu-eclipse.github.io/) with a _Segger Jlink adapter_ to debug PX4 running on NuttX (e.g. Pixhawk series boards).

## Необхідне обладнання

- [J-Link EDU Mini](https://www.segger.com/products/debug-probes/j-link/models/j-link-edu-mini/)
- Adapter to connect Segger JLink to Flight Controller [SWD Debug Port](../debug/swd_debug.md) (debug port).
- Мікро USB кабель

## Встановлення

### PX4

Налаштуйте PX4, дотримуючись звичайних вказівок:

- [Setup the PX4 Developer Environment/Toolchain](../dev_setup/dev_env.md) for your platform (e.g. for Linux see: [Development Environment on Ubuntu LTS / Debian Linux](../dev_setup/dev_env_linux_ubuntu.md)).
- [Download PX4](../dev_setup/building_px4.md) and optionally build it on the command line.

### Eclipse

To install _Eclipse_:

1. Download [Eclipse CDT for C/C++ Developers](https://github.com/gnu-mcu-eclipse/org.eclipse.epp.packages/releases/) (MCU GitHub).
2. Розпакуйте папку Eclipse та скопіюйте її куди завгодно (немає потреби запускати будь-які сценарії установки).
3. Run _Eclipse_ and choose a location for your initial workbench.

### Інструменти Segger Jlink

To install the _Segger Jlink_ tools:

1. Download and run the [J-Link Software and Documentation Pack](https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack) for your OS (Windows and Linux packages available).
   - On Linux the tools are installed in **/usr/bin**.

For more information, see: [https://gnu-mcu-eclipse.github.io/debug/jlink/install/](https://gnu-mcu-eclipse.github.io/debug/jlink/install/).

## Перше використання

1. Connect the _Segger JLink_ to the host computer and the [flight controller debug port](../debug/swd_debug.md) (via an adapter).

2. Увімкніть модульний політний контролер.

3. Run _Eclipse_.

4. Add a source by choosing **File > Import > C/C++ > Existing Code as Makefile Project** and click **Next**.

5. Point it to the **PX4-Autopilot** folder and give it a name, then select _ARM Cross GCC_ in the _Toolchain for Indexer Settings_ and click **Finish**.
   Імпорт триває деякий час. Дочекайтеся його завершення.

6. Set the MCU settings: right-click on the top-level project in the Project Explorer, select _Properties_ then under MCU choose _SEGGER J-Link Path_.
   Встановіть його, як показано на знімку екрану нижче.
   ![Eclipse: Segger J-Link Path](../../assets/debug/eclipse_segger_jlink_path.png)

7. Пакети з оновленнями:

   - Click the small icon on the top right called _Open Perspective_ and open the _Packs_ perspective.
      ![Eclipse: Workspace](../../assets/debug/eclipse_workspace_perspective.png)

   - Click the **update all** button.

      :::tip
      This takes a VERY LONG TIME (10 minutes).
      Ігноруйте всі помилки про відсутні пакети.

:::

      ![Eclipse: Workspace Packs Perspective](../../assets/debug/eclipse_packs_perspective.jpg)

   - The STM32Fxx devices are found in the Keil folder, install by right-clicking and then selecting **install** on the according device for F4 and F7.

8. Налаштування конфігурації налагодження для цілі:

   - Right click project and open the _Settings_ (menu: **C/C++ Build > Settings**)
   - Choose the _Devices_ Tab, _Devices_ section (Not _Boards_).
   - Знайдіть FMU чіп, який ви хочете налагодити.

   ![Eclipse: Select FMU in settings](../../assets/debug/eclipse_settings_devices_fmu.png)

9. Select debug configurations with the small drop-down next to the bug symbol:
   ![Eclipse: Debug config](../../assets/debug/eclipse_settings_debug_config.png)

10. Then select _GDB SEGGER J-Link Debugging_ and then the **New config** button on the top left.
   ![Eclipse: GDB Segger Debug config](../../assets/debug/eclipse_settings_debug_config_gdb_segger.png)

11. Налаштування конфігурації збірки:

   - Give it a name and set the _C/C++ Application_ to the corresponding **.elf** file.
   - Choose _Disable Auto build_

      ::: info
      Remember that you must build the target from the command line before starting a debug session.

:::

   ![Eclipse: GDB Segger Debug config](../../assets/debug/eclipse_settings_debug_config_gdb_segger_build_config.png)

12. The _Debugger_ and _Startup_ tabs shouldn’t need any modifications (just verify your settings with the screenshots below)

   ![Eclipse: GDB Segger Debug config: debugger tab](../../assets/debug/eclipse_settings_debug_config_gdb_segger_build_config_debugger_tab.png)
   ![Eclipse: GDB Segger Debug config: startup tab](../../assets/debug/eclipse_settings_debug_config_gdb_segger_build_config_startup_tab.png)

## Відлагодження з урахуванням завдань SEGGER

Task-aware debugging (also known as [thread-aware debugging](https://www.segger.com/products/debug-probes/j-link/tools/j-link-gdb-server/thread-aware-debugging/)) allows you to show the context of all running threads/tasks instead of just the stack current task.
Це досить корисно, оскільки PX4 має тенденцію запускати виконання багато різних завдань.

Для активації цієї функції в Eclipse:

1. You first need to enable `CONFIG_DEBUG_TCBINFO` in the NuttX configuration for your build (to expose the TCB offsets).

   - Відкрийте термінал у кореневій теці вихідного коду PX4-Autopilot

   - In the terminal, open `menuconfig` using the appropriate make target for the build.
      Це виглядатиме приблизно так:

      ```sh
      make px4_fmu-v5_default boardguiconfig
      ```

      (See [PX4 Menuconfig Setup](../hardware/porting_guide_config.md#px4-menuconfig-setup) for more information) on using the config tools).

   - Ensure that the _Enable TCBinfo struct for debug_ is selected as shown:
      ![NuttX: Menuconfig: CONFIG_DEBUG_TCBINFO](../../assets/debug/nuttx_tcb_task_aware.png)

2. Compile the **jlink-nuttx.so** library in the terminal by running the following command in the terminal: `make jlink-nuttx`

3. Змініть Eclipse, щоб використовувати цю бібліотеку.
   In the _J-Link GDB Server Setup_ configuration, update **Other options** to include `-rtos /home/<PX4 path>/Tools/jlink-nuttx.so`, as shown in the image below.

   ![Eclipse: GDB Segger Debug config RTOS aware: debugger tab](../../assets/debug/eclipse_settings_debug_config_gdb_segger_task_aware.png)

4. Під час запуску налагоджувача ви повинні побачити зараз декілька потоків замість одного:

   ![Eclipse: GDB Segger Debug config RTOS aware: debug session](../../assets/debug/eclipse_settings_debug_config_gdb_segger_task_aware_tasks.png)

## Усунення проблем

### Цільовий процесор відсутній в Package Manager

Якщо цільовий ЦП не відображається в package manager, вам може знадобитися ці крок для запуску відображення реєстру.

:::tip
This should not generally happen (but anecdotally has been reported when connecting to an STM F7 controller).
:::

Adding missing SVD files for the _Peripheral View_:

1. Find out where MCU Eclipse stores its packages (**Preferences > C/C++ > MCU Packages**):

   ![Eclipse: MCU Packages](../../assets/debug/eclipse_mcu_packages.png)

2. Завантажте відсутні пакети з: http://www.keil.com/dd2/Pack/

3. Open downloaded pack with a decompression tool, and extract the **.SVD** files from: **/CMSIS/SVD**.

4. Select desired **.SVD** file in: **Debug Options > GDB SEGGER JLink Debugging > SVD Path**

   ![Eclipse: SVD File path](../../assets/debug/eclipse_svd_file_path.png)
