# MCU-Link адаптер для налагодження

[MCU-Link Debug Probe](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/mcu-link-debug-probe:MCU-LINK) - це дешевий, швидкий і дуже потужний засіб відлагодження, який може використовуватися як самостійний засіб відлагодження та комунікатор консолі під час роботи з платами Pixhawk.

Основні функції:

- Тільки єдине USB-C з'єднання для Reset, SWD, SWO та послідовного входу у дуже компактному виконанні!
- До 9,6 Мбіт/с підключення SWO.
  До 5 MBaud послідовний. 1.2V до 5V цільової напруги.
  З'єднання по високошвидкісному протоколу USB2 до 480 Мбіт/c.
- Приводимий в дію програмним забезпеченням NXP LinkServer або pyOCD з широкою підтримкою пристроїв.
- Набагато дешевше (<15€) ніж адаптер відладки Pixhawk (~20€) з JLink EDU mini (~55€) або JLink BASE (~400€) при кращих технічних характеристиках апаратного забезпечення.

[Адаптер відлагодження Pixhawk](https://holybro.com/products/pixhawk-debug-adapter) надає простий спосіб підключення Pixhawk до MCU-Link (зонд не постачається з адаптером для роботи з контролерами польоту Pixhawk).

:::info
Ці інструкції було перевірено на: FMUv6X-RT, FMUv6X, FMUv6c, FMUv5X.
:::

## Налагодження конфігурації за допомогою NXP LinkServer

MCU-Link забезпечує для чіпів NXP (FMUv6X-RT) сервер GDB [LinkServer](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/linkserver-for-microcontrollers:LINKERSERVER):

[Завантажте](https://www.nxp.com/design/design-center/software/development-software/mcuxpresso-software-and-tools-/linkserver-for-microcontrollers:LINKERSERVER#downloads) Linkserver для вашої операційної системи та дотримуйтесь інструкцій з установки.

На Windows LinkServer встановлюється в `C:\NXP\LinkServer_x.x.x`
На Linux LinkServer встановлюється `/usr/local/LinkServer/LinkServer`

Для прошивки ви можете використовувати команду `LinkServer flash` з метою `MIMXRT1176xxxxx:MIMXRT1170-EVK-CM7-ONLY` для FMUv6X-RT

```sh
/usr/local/LinkServer/LinkServer flash MIMXRT1176xxxxx:MIMXRT1170-EVK-CM7-ONLY load build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.elf
```

Ви можете запустити сервер GDB в новому терміналі:

```sh
/usr/local/LinkServer/LinkServer gdbserver MIMXRT1176xxxxx:MIMXRT1170-EVK-CM7-ONLY
```

Потім підключіться до порту 3333 через GDB:

```sh
arm-none-eabi-gdb build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.elf -ex "target extended-remote :3333"
```

Використовуйте GDB, щоб завантажити двійковий файл в Pixhawk:

```sh
(gdb) load
```

## Налаштування відлагодження за допомогою pyOCD

MCU-Link надає [GDB сервер через pyOCD](https://pyocd.io/):

```sh
python3 -m pip install -U pyocd
```

Ви можете запустити сервер GDB в новому терміналі:

```sh
pyocd gdb -t mimxrt1170_cm7
```

Потрібно, щоб ціль була однією з:

- FMUv6X-RT: `mimxrt1170_cm7`
- FMUv6X: `stm32h743xx`
- FMUv6C: `stm32h743xx`
- FMUv5X: `stm32f767zi`

Потім можна під'єднатися до порту 3333 через GDB:

```sh
arm-none-eabi-gdb build/px4_fmu-v6xrt_default/px4_fmu-v6xrt_default.elf -ex "target extended-remote :3333"
```

Використовуйте GDB, щоб завантажити двійковий файл в Pixhawk:

```sh
(gdb) load
```
