# Початковий завантажувач STM32

The code for the PX4 bootloader is available from the Github [Bootloader](https://github.com/px4/bootloader) repository.

## Підтримувані плати

- FMUv2 (Pixhawk 1, STM32F4)
- FMUv3 (Pixhawk 2, STM32F4)
- FMUv4 (Pixracer 3 і Pixhawk 3 Pro, STM32F4)
- FMUv5 (Pixhawk 4, STM32F7)
- TAPv1 (TBA, STM32F4)
- ASCv1 (TBA, STM32F4)

## Збираємо початковий завантажувач

```sh
git clone https://github.com/PX4/Bootloader.git
cd Bootloader
git submodule init
git submodule update
make
```

Після цього кроку, розмаїття elf-файлів для усіх підтримуваних плат будуть присутні в директорії Bootloader.

## Прошиваємо початковий завантажувач

:::warning
The right power sequence is critical for some boards to allow JTAG / SWD access. Виконайте ці кроки точно так, як описано.
:::

Інструкції нижче дійсні для Blackmagic / Dronecode адаптерів.
Для інших JTAG адаптерів будуть потрібні інші, але подібні кроки.
Розробники, які намагаються прошити завантажувач повинні мати необхідні знання.
Якщо ви не знаєте, як це зробити, то ймовірно, слід переглянути, чи дійсно вам потрібно щось змінювати у початковому завантажувачі.

Послідовність наступна:

1. Від'єднати кабель JTAG
2. Під'єднайте USB-кабель живлення
3. Під'єднати кабель JTAG

### Black Magic / Dronecode адаптери

#### Використовуємо правильний послідовний порт

- On LINUX: `/dev/serial/by-id/usb-Black_Sphere_XXX-if00`
- On MAC OS: Make sure to use the cu.xxx port, not the tty.xxx port: `tar ext /dev/tty.usbmodemDDEasdf`

```sh
arm-none-eabi-gdb
  (gdb) tar ext /dev/serial/by-id/usb-Black_Sphere_XXX-if00
  (gdb) mon swdp_scan
  (gdb) attach 1
  (gdb) mon option erase
  (gdb) mon erase_mass
  (gdb) load tapv1_bl.elf
        ...
        Transfer rate: 17 KB/sec, 828 bytes/write.
  (gdb) kill
```

###

These instructions are for the [J-Link GDB server](https://www.segger.com/jlink-gdb-server.html).

#### Вимоги

[Download the J-Link software](https://www.segger.com/downloads/jlink) from the Segger website and install it according to their instructions.

#### Запустити JLink GDB сервер

Команда використовується для запуску сервера для польотних контролерів що використовують STM32F427VI SoC:

```sh
JLinkGDBServer -select USB=0 -device STM32F427VI -if SWD-DP -speed 20000
```

The `--device`/SoC for common targets is:

- **FMUv2, FMUv3, FMUv4, aerofc-v1, mindpx-v2:** STM32F427VI
- **px4_fmu-v4pro:** STM32F469II
- **px4_fmu-v5:** STM32F765II
- **crazyflie:** STM32F405RG

#### Під'єднайтесь до GDB

```sh
arm-none-eabi-gdb
  (gdb) tar ext :2331
  (gdb) load aerofcv1_bl.elf
```

### Усунення проблем

Якщо будь-який з наведених нижче команд не знайдено, ви або не використовуєте Blackmagic probe або його програмне забезпечення застаріло.
Спершу оновіть прошивку адаптера.

Якщо Ви отримуєте наступне повідомлення про помилку:

```
Error erasing flash with vFlashErase packet
```

Відключити плату (лишивши JTAG підключеним) та запустіть

```sh
mon tpwr disable
swdp_scan
attach 1
load tapv1_bl.elf
```

Це вимкне живлення пристрою й уможливить наступний цикл прошивки.
