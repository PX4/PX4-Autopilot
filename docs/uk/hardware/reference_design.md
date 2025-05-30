# Проектування контролера польоту PX4

The PX4 reference design is the [Pixhawk series](../flight_controller/pixhawk_series.md) of flight controllers. First released in 2011, this design is now in its 5th [generation](#reference_design_generations) (with the 6th generation board design in progress).

## Сумісність з бінарним кодом

Всі плати, виготовлені за певним дизайном, мають бути сумісними з двійковим кодом (тобто можуть запускати ту ж саму прошивку). З 2018 року ми будемо надавати набір тестів для перевірки бінарної сумісності, який дозволить нам перевірити та сертифікувати цю сумісність.

Покоління FMU 1-3 були розроблені як відкрите обладнання, тоді як покоління FMU 4 та 5 надали лише специфікації роз'ємів та живлення (схеми були створені окремими виробниками). Для кращої сумісності, починаючи з FMUv6 і пізніше, буде використано повний модельний референтний дизайн.

<a id="reference_design_generations"></a>

## Reference Design Generations

- FMUv1: Development board \(STM32F407, 128 KB RAM, 1MB flash, [schematics](https://github.com/PX4/Hardware/tree/master/FMUv1)\) (no longer supported by PX4)
- FMUv2: Pixhawk \(STM32F427, 168 MHz, 192 KB RAM, 1MB flash, [schematics](https://github.com/PX4/Hardware/tree/master/FMUv2)\)
- FMUv3: Pixhawk variants with 2MB flash \(3DR Pixhawk 2 \(Solo\), Hex Pixhawk 2.1, Holybro Pixfalcon, 3DR Pixhawk Mini, STM32F427, 168 MHz, 256 KB RAM, 2 MB flash, [schematics](https://github.com/PX4/Hardware/tree/master/FMUv3_REV_D)\)
- FMUv4: Pixracer \(STM32F427, 168 MHz, 256 KB RAM, 2 MB flash, [pinout](https://docs.google.com/spreadsheets/d/1raRRouNsveQz8cj-EneWG6iW0dqGfRAifI91I2Sr5E0/edit#gid=1585075739)\)
- FMUv4 PRO: Drotek Pixhawk 3 PRO \(STM32F469, 180 MHz, 384 KB RAM, 2 MB flash, [pinout](https://docs.google.com/spreadsheets/d/1raRRouNsveQz8cj-EneWG6iW0dqGfRAifI91I2Sr5E0/edit#gid=1585075739)\)
- FMUv5: Holybro Pixhawk 4 \(STM32F765, 216 MHz, 512 KB RAM, 2 MB flash, [pinout](https://docs.google.com/spreadsheets/d/1-n0__BYDedQrc_2NHqBenG1DNepAgnHpSGglke-QQwY/edit#gid=912976165)\)
- FMUv5X: (Multiple Products) \(STM32F765, 400 MHz, 512KB RAM, 2 MB flash\)  ([standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-011%20Pixhawk%20Autopilot%20v5X%20Standard.pdf))
- FMUv6X: (Multiple Products) \(STM32H753, 480 MHz, 1 MB RAM, 2 MB flash\) and variant 6i \(i.MX RT1050, 600 MHz, 512 KB RAM, external flash\) ([standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-012%20Pixhawk%20Autopilot%20v6X%20Standard.pdf))
- FMUv6C: (Multiple Products) \(STM32H743V, 480 MHz, 1 MB RAM, 2 MB flash\) ([standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-018%20Pixhawk%20Autopilot%20v6C%20Standard.pdf))
- FMUv6U: (Multiple Products) \(STM32H753, 400 MHz, 1 MB RAM, 2 MB flash\) ([standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-016%20Pixhawk%20Autopilot%20v6U%20Standard.pdf))
- FMUv6X-RT: (Multiple Products) \(NXP i.MX RT1176, 32 Bit Arm® Cortex®-M7, 1GHz 32 Bit Arm® Cortex®-M4, 400MHz secondary core, 2 MB RAM, 64 MB flash\) and variant 6i \(i.MX RT1050, 600 MHz, 512 KB RAM, external flash\) ([standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-020%20Pixhawk%20Autopilot%20v6X-RT%20Standard.pdf))

Starting with FMUv5X all new standards are published on GitHub under [Pixhawk/Pixhawk-Standards](https://github.com/pixhawk/Pixhawk-Standards). See [Pixhawk.org](https://pixhawk.org) for more info.

## Розподіл функцій основної / ввід-вивідної функції

Діаграма нижче показує розподіл обов'язків автобуса та функціональних відповідальностей між платами FMU та I/O в контролері польоту серії Pixhawk (плати вбудовані в один фізичний модуль).

![PX4 Main/IO Functional Breakdown](../../assets/diagrams/px4_fmu_io_functions.png)

<!-- Draw.io version of file can be found here: https://drive.google.com/file/d/1H0nK7Ufo979BE9EBjJ_ccVx3fcsilPS3/view?usp=sharing -->

Деякі контролери серії Pixhawk будуються без плати введення-виведення для зменшення розміру або складності, або для кращого вирішення певних сценаріїв використання плати.
У цьому випадку І/O драйвер не запущений.

:::info
Manufacturer flight controller variants without an I/O board are often named as a "diminutive" of a version that includes the I/O board: e.g. _Pixhawk 4_ **Mini**_, \_CUAV v5 **nano**_.
:::

Build targets that must run on flight controllers with an I/O board map the FMU outputs to `AUX` and the I/0 outputs to `MAIN` (see diagram above).
Якщо ціль запускається на обладнанні, де відсутня або вимкнена плата вводу-виводу, вихідні сигнали головних ШШ не будуть присутні.
You might see this, for example, by running `px4_fmu-v5_default` on [Pixhawk 4](../flight_controller/pixhawk4.md) (with IO) and [Pixhawk 4 Mini](../flight_controller/pixhawk4_mini.md) (without I/O).

:::warning
On [Pixhawk 4 Mini](../flight_controller/pixhawk4_mini.md) this results in a mismatch between the `MAIN` label screenprinted on the flight controller and the `AUX` bus shown during [Actuator Configuration](../config/actuators.md).
::: info that if a build target is only ever intended to run on a flight controller that does not have an I/0 board, then the FMU outputs are mapped to `MAIN` (for example, the `px4_fmu-v4_default` target for [Pixracer](../flight_controller/pixracer.md)).

PX4 PWM outputs are mapped to either `MAIN` or `AUX` ports in [Actuator Configuration](../config/actuators.md).
