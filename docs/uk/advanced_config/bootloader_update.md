# Оновлення завантажувача

_PX4 Bootloader_ використовується для завантаження прошивки для [Pixhawk boards](../flight_controller/pixhawk_series.md) (PX4FMU, PX4IO).

Зазвичай контролери Pixhawk поставляються з попередньо встановленою відповідною версією завантажувача.
Однак у деяких випадках його може бути відсутній, або може бути присутня старіша версія, яку потрібно оновити, або плата може бути відключена і потребує стирання та перевстановлення завантажувача.

Ця тема пояснює, як побудувати завантажувач PX4 та кілька методів для його прошивки на плату.

::: info

- You can use [QGC Bootloader Update](#qgc-bootloader-update-sys-bl-update) with firmware that includes the [`bl-update` module](../modules/modules_command.md#bl-update).
  This is the easiest way to update the bootloader, provided the board is able to load firmware.
- You can also use the [Debug Probe](#debug-probe-bootloader-update) to update the bootloader.
  This is useful for updating/fixing the bootloader when the board is bricked.
- На [FMUv6X-RT](../flight_controller/pixhawk6x-rt.md) ви можете [встановлювати завантажувач/відновлювати плати через USB](bootloader_update_v6xrt.md).
  Це корисно, якщо у вас немає тесту налагодження.

:::

## QGC Bootloader Update (`SYS_BL_UPDATE`)

The easiest way to update the bootloader is to first use _QGroundControl_ to install firmware that contains the desired/latest bootloader.
Ви можете ініціювати оновлення завантажувача при наступному перезавантаженні, встановивши параметр: [SYS_BL_UPDATE](../advanced_config/parameter_reference.md#SYS_BL_UPDATE).

This approach can be used if the [`bl-update` module](../modules/modules_command.md#bl-update) is present in the firmware.
The easiest way to check this is just to see if the [SYS_BL_UPDATE](../advanced_config/parameter_reference.md#SYS_BL_UPDATE) parameter is [found in QGroundControl](../advanced_config/parameters.md#finding-a-parameter).

:::warning
Boards that include the module will have the line `CONFIG_SYSTEMCMDS_BL_UPDATE=y` in their `default.px4board` file (for examples [see this search](https://github.com/search?q=repo%3APX4%2FPX4-Autopilot+path%3A**%2Fdefault.px4board+CONFIG_SYSTEMCMDS_BL_UPDATE%3Dy&type=code)).
You can enable this key in your own custom firmware if needed.
:::

Кроки наступні:

1. Вставте SD-карту (це дозволяє реєструвати журнали завантаження для відлагодження будь-яких проблем).

2. [Оновіть прошивку](../config/firmware.md#custom) з образом, що містить новий/потрібний завантажувач.

   ::: info
   The updated bootloader might be included the default firmware for your board or supplied in custom firmware.

:::

3. Зачекайте, доки транспортний засіб перезавантажиться.

4. [Знайдіть](../advanced_config/parameters.md) та увімкніть параметр [SYS_BL_UPDATE](../advanced_config/parameter_reference.md#SYS_BL_UPDATE).

5. Перезавантажте (відключіть / підключіть плату).
   Оновлення завантажувача займе лише кілька секунд.

Зазвичай на цьому етапі ви можливо захочете [оновити прошивку](../config/firmware.md) ще раз, використовуючи правильно/ново встановлений загрузчик.

An specific example of this process for updating the [FMUv2 bootloader](#fmuv2-bootloader-update) is given below.

## Створення завантажувача PX4

### PX4 Bootloader FMUv6X та новіші

Плати, що починаються з FMUv6X (STM32H7), використовують вбудований завантажувач PX4.

Це можна побудувати з каталогу [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot), використовуючи команду `make` та конкретну для плати ціль з суфіксом `_bootloader`.

Для FMUv6X команда наступна:

```sh
make px4_fmu-v6x_bootloader
```

Це збудує бінарний файл завантажувача як `build/px4_fmu-v6x_bootloader/px4_fmu-v6x_bootloader.elf`, який можна прошити через SWD або DFU.
Якщо ви збираєте завантажувач, вам вже повинні бути знайомі з одним із цих варіантів.

Якщо вам потрібний файл у форматі HEX замість ELF файлу, використовуйте objcopy:

```sh
arm-none-eabi-objcopy -O ihex build/px4_fmu-v6x_bootloader/px4_fmu-v6x_bootloader.elf px4_fmu-v6x_bootloader.hex
```

### PX4 Bootloader FMUv5X та раніші версії

PX4 boards up to FMUv5X (before STM32H7) used the [PX4 bootloader](https://github.com/PX4/PX4-Bootloader) repository.

Інструкції в README репозиторію пояснюють, як його використовувати.

## Оновлення завантажувача Debug Probe

Наступні кроки пояснюють, як ви можете "вручну" оновити завантажувач за допомогою сумісного [Відладного пристрою](../debug/swd_debug.md#debug-probes-for-px4-hardware):

1. Отримайте бінарний файл, який містить завантажувальник (або від команди розробників, або [зіберіть його самостійно](#building-the-px4-bootloader)).

2. Get a [Debug Probe](../debug/swd_debug.md#debug-probes-for-px4-hardware).
   Підключіть зонд до комп'ютера за допомогою USB та налаштуйте `gdbserver`.

3. Перейдіть до каталогу, що містить бінарний файл, і запустіть команду для обраного вами завантажувача в терміналі:

   - FMUv6X

     ```sh
     arm-none-eabi-gdb px4_fmu-v6x_bootloader.elf
     ```

   - FMUv6X-RT

     ```sh
     arm-none-eabi-gdb px4_fmu-v6xrt_bootloader.elf
     ```

   - FMUv5

     ```sh
     arm-none-eabi-gdb px4fmuv5_bl.elf
     ```

   H7 Завантажувачі з [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) мають назву за шаблоном `*._bootloader.elf`.
   Завантажувачі з [PX4/PX4-Bootloader](https://github.com/PX4/PX4-Bootloader) мають назву за шаблоном `*_bl.elf`.

:::

4. The _gdb terminal_ appears and it should display (something like) the following output:

   ```sh
   GNU gdb (GNU Tools for Arm Embedded Processors 7-2017-q4-major) 8.0.50.20171128-git
   Copyright (C) 2017 Free Software Foundation, Inc.
   License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
   This is free software: you are free to change and redistribute it.
   There is NO WARRANTY, to the extent permitted by law.
   Type "show copying"    and "show warranty" for details.
   This GDB was configured as "--host=x86_64-linux-gnu --target=arm-none-eabi".
   Type "show configuration" for configuration details.
   For bug reporting instructions, please see:
   <https://www.sourceware.org/gdb/bugs/>.
   Find the GDB manual and other documentation resources online at:
   <https://www.sourceware.org/gdb/documentation/>.
   For help, type "help".
   Type "apropos word" to search for commands related to "word"...
   Reading symbols from px4fmuv5_bl.elf...done.
   ```

5. Find your `<dronecode-probe-id>` by running an `ls` command in the **/dev/serial/by-id** directory.

6. Тепер підключіться до debug probe з наступною командою:

   ```sh
   tar ext /dev/serial/by-id/<dronecode-probe-id>
   ```

7. Увімкніть Pixhawk за допомогою іншого USB-кабелю та під’єднайте зонд до порту `FMU-DEBUG`.

   ::: info
   If using a Zubax BugFace BF1 you may need to remove the case in order to connect to the `FMU-DEBUG` port (e.g. on Pixhawk 4 you would do this using a T6 Torx screwdriver).

:::

8. Використовуйте таку команду, щоб знайти SWD Pixhawk і підключитися до нього:

   ```sh
   (gdb) mon swdp_scan
   (gdb) attach 1
   ```

9. Завантажте двійковий файл в Pixhawk:

   ```sh
   (gdb) load
   ```

Після оновлення завантажувача ви можете [завантажити прошивку PX4](../config/firmware.md) за допомогою _QGroundControl_.

## Оновлення завантажувача FMUv2

Якщо _QGroundControl_ встановлює ціль FMUv2 (див. консоль під час встановлення), і у вас є новіша плата, вам може знадобитися оновити завантажувальник, щоб мати доступ до всієї пам'яті на вашому контролері польоту.
This example explains how you can use [QGC Bootloader Update](qgc-bootloader-update-sys-bl-update) to update the bootloader.

:::info
Ранні контролери польоту FMUv2 [Pixhawk-series](../flight_controller/pixhawk_series.md#fmu_versions) мали [апаратну проблему](../flight_controller/silicon_errata.md#fmuv2-pixhawk-silicon-errata), яка обмежувала їх використання 1 Мб флеш-пам’яті.
Проблема виправлена на новіших платах, але вам може знадобитися оновити заводський завантажувальник, щоб встановити прошивку FMUv3 та мати доступ до всієї доступної пам'яті у 2 МБ.
:::

Щоб оновити завантажувач:

1. Вставте SD-карту (це дозволяє реєструвати журнали завантаження для відлагодження будь-яких проблем).

2. [Оновіть програмне забезпечення](../config/firmware.md) до версії PX4 _master_ (під час оновлення програмного забезпечення перевірте **Розширені налаштування** і виберіть **Розробницьку збірку (master)** із випадаючого списку).
   _QGroundControl_ автоматично виявить, що апаратне забезпечення підтримує FMUv2 і встановить відповідне програмне забезпечення.

   ![FMUv2 update](../../assets/qgc/setup/firmware/bootloader_update.jpg)

   Зачекайте, доки транспортний засіб перезавантажиться.

3. [Знайдіть](../advanced_config/parameters.md) та увімкніть параметр [SYS_BL_UPDATE](../advanced_config/parameter_reference.md#SYS_BL_UPDATE).

4. Перезавантажте (відключіть / підключіть плату).
   Оновлення завантажувача займе лише кілька секунд.

5. Тоді знову [Оновити програмне забезпечення](../config/firmware.md).
   На цей раз _QGroundControl_ повинен автоматично визначити обладнання як FMUv3 і відповідним чином оновити програмне забезпечення.

   ![FMUv3 update](../../assets/qgc/setup/firmware/bootloader_fmu_v3_update.jpg)

   ::: info
   Якщо апаратне забезпечення має [Помилки в кремнієвій мікросхемі](../flight_controller/silicon_errata.md#fmuv2-pixhawk-silicon-errata), воно все одно буде виявлене як FMUv2, і ви побачите, що FMUv2 було знову встановлено (у консолі).
   У цьому випадку ви не зможете встановити апаратне забезпечення FMUv3.

:::

## Інші плати (не Pixhawk)

Плати, які не є частиною серії [Pixhawk](../flight_controller/pixhawk_series.md), матимуть власні механізми оновлення завантажувача.

Для плат, які передвстановлені за допомогою Betaflight, дивіться [Flash пусковика на системи Betaflight](bootloader_update_from_betaflight.md).
