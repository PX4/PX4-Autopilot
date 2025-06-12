# Black Magic / Dronecode адаптери

The [Black Magic Probe](https://black-magic.org) is an easy to use, mostly plug-and-play, JTAG/SWD debugger for embedded microcontrollers.
Оскільки Black Magic Probe є загальним зондом для налагодження, вам знадобиться адаптер для підключення до керуючих пристроїв польоту Pixhawk, який можна придбати тут:

- [Drone Code Debug Adapter](https://1bitsquared.com/products/drone-code-debug-adapter) (1 BIT SQUARED).

## Dronecode Probe

The [Dronecode Probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation) is a specialization of the Black Magic Probe for debugging PX4 autopilots.

The probe's USB interface exposes two separate virtual serial port interfaces: one for connecting to the [System Console](system_console.md) (UART) and the other for an embedded GDB server (SWD interface).

The probe provides a DCD-M connector cable for attaching to the [Pixhawk Debug Mini](swd_debug.md#pixhawk-debug-mini).

:::info
The _6-pos DF13_ connector that comes with the probe cannot be used for SWD debugging (it is for using the System Console).
:::

## Використання зонда

:::info
To debug STM32F7 or later (FMUv5 and newer) the Dronecode probe / Blackmagic probe likely requires a firmware update.
You can find how to update the [blackmagic probe here](https://github.com/blacksphere/blackmagic/wiki/Upgrading-Firmware).
:::

Щоб використовувати зонд Dronecode з GDB, запустіть GDB з точним ELF-файлом, який в даний момент прошивається на автопілоті:

```sh
arm-none-eabi-gdb build/px4_fmu-v5_default/px4_fmu-v5_default.elf
```

Потім вам потрібно вибрати інтерфейс зонду Dronecode, на Linux це, наприклад.:

```sh
target ext /dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe_f9414d5_7DB85DAC-if00
```

Потім проскануйте ціль:

```sh
monitor swdp_scan
```

І ви маєте побачити щось на зразок:

```sh
Target voltage: 3.3V
Available Targets:
No. Att Driver
 1      STM32F76x M7
```

Зверніть увагу, що для деяких автопілотів він показує 0.0V, але наступні кроки все одно працюють.

Тепер ви можете підключитися до цієї цілі:

```sh
attach 1
```

Тепер ви повинні бути підключені.
