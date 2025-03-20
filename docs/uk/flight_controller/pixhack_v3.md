# Pixhack V3

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://store.cuav.net/) for hardware support or compliance issues.
:::

The CUAV _Pixhack V3_ flight controller board is a flexible autopilot intended primarily for manufacturers of commercial systems.

The board is a variant of the SOLO Pixhawk<sup>&reg;</sup> 2 (PH2) flight controller, which is in turn based on the [Pixhawk-project](https://pixhawk.org/) **FMUv3** open hardware design.
It runs PX4 on the [NuttX](https://nuttx.apache.org/) OS, and is fully compatible with both PX4 or ArduPilot<sup>&reg;</sup> (APM) firmware.

_Pixhack V3_ has significant improvements with respect to the original design, including better interface layout and the addition of vibration damping and a thermostat system.

![Pixhack v3](../../assets/flight_controller/pixhack_v3/pixhack_v3_157_large_default.jpg)

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Короткий опис

- Мікропроцесор:
  - STM32F427
  - STM32F100 (відмовостійкий співпроцесор)
- Датчики:
  - Акселерометри (3): LS303D, MPU6000, MPU9250/hmc5983
  - Гіроскопи (3): L3GD20, MPU6000, MPU9250
  - Компаси (2): LS303D, MPU9250
  - Барометри (2): MS5611 X2
- Інтерфейси:
  - MAVLink UART (2)
  - GPS UART (2)
  - DEBUG UART (1)
  - RC IN (для PPM, SBUS, DSM/DSM2)
  - RSSI IN: PWM OR 3.3ADC
  - I2C (2)
  - CAN BUS (1)
  - ADC IN: 3.3В X1 , 6.6В X1
  - PWM OUT: 8 PWM IO + 4 IO
- Система живлення:
  - PM POWER IN: 4.5 ~ 5.5 В
  - USB POWER IN: 5.0 В +- 0.25В
- Вага та розміри:
  - Вага: 63г
  - Ширина: 68мм
  - Товщина: 17мм
  - Довжина: 44мм
- Інші характеристики:
  - Температура роботи: -20 ~ 60°C

## Доступність

Плату можна придбати з:

- [store.cuav.net](http://store.cuav.net/index.php?id_product=8&id_product_attribute=0&rewrite=pixhack-v3-autopilot&controller=product&id_lang=3)
- [leixun.aliexpress.com/store](https://leixun.aliexpress.com/store)

## Збірка прошивки

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make px4_fmu-v3_default
```

## Розпіновки та схеми

- [Documentation/wiring guides](http://doc.cuav.net/flight-controller/pixhack/en/pixhack-v3.html)

## Налаштування послідовного порту

| UART   | Пристрій   | Порт                                          |
| ------ | ---------- | --------------------------------------------- |
| UART1  | /dev/ttyS0 | IO debug                                      |
| USART2 | /dev/ttyS1 | TELEM1 (керування потоком) |
| USART3 | /dev/ttyS2 | TELEM2 (керування потоком) |
| UART4  |            |                                               |
| UART7  | CONSOLE    |                                               |
| UART8  | SERIAL4    |                                               |
