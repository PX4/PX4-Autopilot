# ARK Електроніка ARKV6X

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://arkelectron.com/contact-us/) for hardware support or compliance issues.
:::

The USA-built [ARKV6X](\(https://arkelectron.gitbook.io/ark-documentation/flight-controllers/arkv6x\)) flight controller is based on the [FMUV6X and Pixhawk Autopilot Bus open source standards](https://github.com/pixhawk/Pixhawk-Standards).

Завдяки потрійній синхронізації IMU можливе узагальнення даних, голосування та фільтрація.
The Pixhawk Autopilot Bus (PAB) form factor enables the ARKV6X to be used on any [PAB-compatible carrier board](../flight_controller/pixhawk_autopilot_bus.md), such as the [ARK Pixhawk Autopilot Bus Carrier](../flight_controller/ark_pab.md).

![ARKV6X Main Photo](../../assets/flight_controller/arkv6x/ark_v6x_front.jpg)

:::info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Де купити

Order From [Ark Electronics](https://arkelectron.com/product/arkv6x/) (US)

## Датчики

- [Dual Invensense ICM-42688-P IMUs](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)
- [Invensense IIM-42652 Industrial IMU](https://invensense.tdk.com/products/smartindustrial/iim-42652/)
- [Bosch BMP390 Barometer](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp390/)
- [Bosch BMM150 Magnetometer](https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm150/)

## Мікропроцесор

- [STM32H743IIK6 MCU](https://www.st.com/en/microcontrollers-microprocessors/stm32h743ii.html)
  - 480MHz
  - 2MB Flash
  - 1MB Flash

## Інші характеристики

- FRAM
- [Pixhawk Autopilot Bus (PAB) Form Factor](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf)
- LED індикатори
- Слот MicroSD
- USA Built
- Розроблений з нагрівачем потужністю 1 Вт. Підтримує датчики в теплі в екстремальних умовах

## Вимоги до живлення

- 5V
- 500 мА
  - 300 мА для основної системи
  - 200 мА для нагрівача

## Додаткова інформація

- Вага: 5.0 g
- Розміри: 3,6 x 2,9 x 0,5 см

## Схема розташування виводів

For pinout of the ARKV6X see the [DS-10 Pixhawk Autopilot Bus Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-010%20Pixhawk%20Autopilot%20Bus%20Standard.pdf)

## Налаштування послідовного порту

| UART   | Пристрій   | Порт                            |
| ------ | ---------- | ------------------------------- |
| USART1 | /dev/ttyS0 | GPS                             |
| USART2 | /dev/ttyS1 | TELEM3                          |
| USART3 | /dev/ttyS2 | Debug Console                   |
| UART4  | /dev/ttyS3 | UART4 & I2C |
| UART5  | /dev/ttyS4 | TELEM2                          |
| USART6 | /dev/ttyS5 | PX4IO/RC                        |
| UART7  | /dev/ttyS6 | TELEM1                          |
| UART8  | /dev/ttyS7 | GPS2                            |

## Збірка прошивки

```sh
make ark_fmu-v6x_default
```

## Дивіться також

- [ARK Electronics ARKV6X](https://arkelectron.gitbook.io/ark-documentation/flight-controllers/arkv6x) (ARK Docs)
