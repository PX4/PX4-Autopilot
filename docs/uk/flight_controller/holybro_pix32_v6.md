# Holybro Pix32 v5

:::warning
PX4 не розробляє цей (або будь-який інший) автопілот.
Contact the [manufacturer](https://holybro.com/) for hardware support or compliance issues.
:::

_Pix32 v6_<sup>&reg;</sup> is the latest update to the pix32 v5 flight controllers. Це варіант Pixhawk 6C з модульним дизайном та спільною ціллю з FMUv6C. It is comprised of a separate flight controller and carrier board which are connected by a [100 pin connector](https://docs.holybro.com/autopilot/pix32-v6/download). Це призначено для тих пілотів, які потребують потужної, гнучкої та настроюваної системи керування польотами.

Він оснащений високопродуктивним процесором H7, резервуванням IMU, платою IMU з контролем температури та економічно вигідним дизайном, що забезпечує неймовірну продуктивність і надійність. It complies with the [Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf).

<img src="../../assets/flight_controller/pix32v6/pix32v6_fc_only.png" width="550px" title="pix32v6 Upright Image" />

<!--
:::tip
This autopilot is [supported](../flight_controller/autopilot_pixhawk_standard.md) by the PX4 maintenance and test teams.
:::
-->

## Введення

Inside the Pix32 v6, you can find an STMicroelectronics® based STM32H743, paired with sensor technology from Bosch® & InvenSense®, giving you flexibility and reliability for controlling any autonomous vehicle, suitable for both academic and commercial applications.

Pix32 v6’s H7 MCU містить ядро Arm® Cortex®-M7 до 480 MHz, має 2MB flash пам’яті та 1MB RAM. Завдяки оновленій потужності обробки розробники можуть бути більш продуктивними та ефективними у своїй роботі з розробкою, що дозволяє використовувати складні алгоритми та моделі. Це включає високопродуктивні, низькошумні IMU на платі, розроблені, щоб бути ефективними з точки зору вартості, при цьому мати резерв IMU. Система віброізоляції для фільтрації високочастотної вібрації та зменшення шуму для забезпечення точних показань, що дозволяє транспортним засобам досягати кращих загальних характеристик польоту.

Цей контролер польоту ідеально підходить для людей, які шукають доступний та модульний контролер польоту, який може використовувати настроювану базову плату. We have made the [pix32 v6 base board schematics public](https://docs.holybro.com/autopilot/pix32-v6/download), you can either make a custom carrier board yourself or just let us help you with it. За допомогою налаштованої базової плати ви можете впевнитися, що фізичний розмір, виводи і вимоги до розподілу живлення відповідають вашому безпілотнику належним чином, забезпечуючи, що у вас є всі необхідні з'єднання і ніяких витрат і масштабу з'єднувачів, яких вам не потрібно.

**Key Design Points**

- High performance STM32H743 Processor with more computing power & RAM
- Новий економічний дизайн із низькопрофільним форм-фактором
- Інтегрована система ізоляції вібрацій для фільтрації високочастотних вібрацій та зменшення шуму для забезпечення точних вимірювань
- IMU температурно контролюються за допомогою вбудованих нагрівальних резисторів, що дозволяє досягти оптимальної робочої температури IMU

# Технічні характеристики

### **Processors & Sensors**

- FMU Processor: STM32H743&#x20;
  - 32 Bit Arm® Cortex®-M7, 480MHz, 2MB memory, 1MB SRAM&#x20;
- IO Processor: STM32F103
  - &#x20;32 Bit Arm® Cortex®-M3, 72MHz, 64KB SRAM&#x20;
- On-board sensors&#x20;
  - &#x20;Accel/Gyro: ICM-42688-P&#x20;
  - Accel/Gyro: BMI055&#x20;
  - Mag: IST8310&#x20;
  - Барометр: MS5611

### **Electrical data**

- Номінальна напруга:
  - Максимальна вхідна напруга: 6 В
  - Вхід USB Power: 4.75~5.25V
  - Вхід Servo Rail: 0\~36V
- Номінальний струм:
  - TELEM1 Обмежувач максимального вихідного струму: 1.5A
  - Комбінований обмежувач вихідного струму всіх інших портів: 1.5A

### **Mechanical data**

- Розміри модуля FC: 44.8 x 44.8 x 13.5
- Вага модуля FC: 36г

### **Interfaces**

- 16- PWM серво виводів (8 з IO, 8 з FMU)

- 3 загальних послідовних портів
  - `TELEM1` - Full flow control, separate 1.5A current limit
  - `TELEM2` - Full flow control
  - `TELEM3`

- 2 порти GPS
  - `GPS1` - Full GPS port (GPS plus safety switch)
  - `GPS2` - Basic GPS port

- 1 I2C порт
  - Підтримує виділене I2C калібрування EEPROM, розташоване на модулі сенсорів

- 2 CAN шини
  - CAN шина має individual silent controls або ESC RX-MUX control

- 2 порти відладки:
  - FMU Debug
  - I/O Debug

- Виділений R/C вхід для Spektrum / DSM та S.BUS, CPPM, аналоговий / PWM RSSI

- Виділений S.BUS вивід

- 2 порти Power input (аналогові)

- Інші характеристики:
  - Operating & storage temperature: -40 ~ 85°c

## Де купити

Order from [Holybro](https://holybro.com/collections/autopilot-flight-controllers/products/pix32-v6).

## Схема розташування виводів

- [Holybro Pix32 v6 Baseboard Ports Pinout](https://docs.holybro.com/autopilot/pix32-v6/pix32-v6-baseboard-ports)
- [Holybro Pix32 v6 Baseboard Ports Pinout](https://docs.holybro.com/autopilot/pix32-v6/pix32-v6-mini-base-ports)

## Налаштування послідовного порту

| UART   | Пристрій   | Порт          |
| ------ | ---------- | ------------- |
| USART1 | /dev/ttyS0 | GPS1          |
| USART2 | /dev/ttyS1 | TELEM3        |
| USART3 | /dev/ttyS2 | Debug Console |
| UART5  | /dev/ttyS3 | TELEM2        |
| USART6 | /dev/ttyS4 | PX4IO         |
| UART7  | /dev/ttyS5 | TELEM1        |
| UART8  | /dev/ttyS6 | GPS2          |

## Розміри

- [Pix32v6 Dimensions](https://docs.holybro.com/autopilot/pix32-v6/dimensions)

## Номінальна напруга

_Pix32 v6_ can be triple-redundant on the power supply if three power sources are supplied. The three power rails are: **USB**, **POWER1**, **POWER2** (N/A on Pix32 v6 Mini-Baseboard) .

**Normal Operation Maximum Ratings**

За таких умов всі джерела живлення будуть використовуватися в цьому порядку для живлення системи:

1. **POWER1** and **POWER2** inputs (4.9V to 5.5V)
2. **USB** input (4.75V to 5.25V)

**Absolute Maximum Ratings**

За таких умов система не буде витрачати жодної потужності (не буде працювати), але залишиться неушкодженою.

1. **POWER1** and **POWER2** inputs (operational range 4.1V to 5.7V, 0V to 10V undamaged)
2. **USB** input (operational range 4.1V to 5.7V, 0V to 6V undamaged)
3. Servo input: VDD_SERVO pin of **FMU PWM OUT** and **I/O PWM OUT** (0V to 42V undamaged)

**Voltage monitoring**

Pix32 v6 використовує аналогові модулі живлення.

Holybro makes various analog [power modules](../power_module/index.md) for different need.

- [PM02 Power Module](../power_module/holybro_pm02.md)
- [PM06 Power Module](../power_module/holybro_pm06_pixhawk4mini_power_module.md)
- [PM07 Power Module](../power_module/holybro_pm07_pixhawk4_power_module.md)

## Збірка прошивки

:::tip
Most users will not need to build this firmware!
It is pre-built and automatically installed by _QGroundControl_ when appropriate hardware is connected.
:::

To [build PX4](../dev_setup/building_px4.md) for this target:

```
make px4_fmu-v6c_default
```

<a id="debug_port"></a>

## Відладочний порт

The [PX4 System Console](../debug/system_console.md) and [SWD interface](../debug/swd_debug.md) run on the **FMU Debug** port.

The pinouts and connector comply with the [Pixhawk Debug Full](../debug/swd_debug.md#pixhawk-debug-full) interface defined in the [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf) interface (JST SM10B connector).

| Pin                            | Сигнал                              | Вольтаж               |
| ------------------------------ | ----------------------------------- | --------------------- |
| 1 (red)     | `Vtref`                             | +3.3V |
| 2 (blk)     | Console TX (OUT) | +3.3V |
| 3 (blk)     | Console RX (IN)  | +3.3V |
| 4 (blk)     | `SWDIO`                             | +3.3V |
| 5 (blk)     | `SWCLK`                             | +3.3V |
| 6 (blk)     | `SWO`                               | +3.3V |
| 7 (blk)     | NFC GPIO                            | +3.3V |
| 8 (чорний)  | PH11                                | +3.3V |
| 9 (чорний)  | nRST                                | +3.3V |
| 10 (чорний) | `GND`                               | GND                   |

Інформацію про використання цього порту див:

- [SWD Debug Port](../debug/swd_debug.md)
- [PX4 System Console](../debug/system_console.md) (Note, the FMU console maps to USART3).

## Периферійні пристрої

- [Digital Airspeed Sensor](https://holybro.com/products/digital-air-speed-sensor)
- [Telemetry Radio Modules](https://holybro.com/collections/telemetry-radios?orderby=date)
- [Rangefinders/Distance sensors](../sensor/rangefinders.md)

## Підтримувані платформи / Конструкції

Будь-який мультикоптер / літак / наземна платформа / човен, який може керуватися звичайними RC сервоприводами або сервоприводами Futaba S-Bus.
The complete set of supported configurations can be seen in the [Airframes Reference](../airframes/airframe_reference.md).

## Подальша інформація

- [Holybro Docs](https://docs.holybro.com/) (Holybro)
- [Reference: Pixhawk 6C Wiring QuickStart](../assembly/quick_start_pixhawk6c.md)
- [PM02 Power Module](../power_module/holybro_pm02.md)
- [PM06 Power Module](../power_module/holybro_pm06_pixhawk4mini_power_module.md)
- [PM07 Power Module](../power_module/holybro_pm07_pixhawk4_power_module.md)
- FMUv6C reference design pinout.
- [Pixhawk Connector Standard](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf).
