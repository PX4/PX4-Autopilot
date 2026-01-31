# Sky-Drones SmartAP GPS

[SmartAP GPS](https://sky-drones.com/navigation/smartap-gnss.html) is a GNSS navigation module with integrated antenna, UBlox Neo-M8N chipset, 3x 3-axis magnetometer (compass), 1x MS-5611 pressure sensor and RGB LED driver.
SmartAP GPS підтримує одночасний прийом до 3 GNSS (GPS, Galileo, GLONASS, BeiDou).

![SmartAP GPS](../../assets/hardware/gps/gps_smartap_gps.jpg)

Основні функції включають:

- Одночасний прийом до 3 GNSS (GPS, Galileo, GLONASS, BeiDou)
- 3х вбудовані магнітометри: HMC5983, IST8310 та LIS3MDL
- 1x вбудований барометр: MS5611
- Драйвер RGB-світлодіодів та індикатори стану

## Де купити

- [Sky-Drones Store](https://sky-drones.com/navigation/smartap-gnss.html)

## Вміст набору

Набір SmartAP GPS включає в себе:

- 1x GPS модуль
- 1x 30cm кабель

## Налаштування

For the aircraft, you should set the parameter [SER_GPS1_BAUD](../advanced_config/parameter_reference.md#SER_GPS1_BAUD) to 115200 8N1 to ensure that PX4 uses the correct baudrate.

## Підключення та з'єднання

SmartAP GPS має 10-контактний роз'єм JST-GH, який можна підключити до польотного контролера Pixhawk (сумісний зі стандартом роз'єму Pixhawk).

| Номер виводу | Назва виводу                    |
| ------------ | ------------------------------- |
| 1            | 5V                              |
| 2            | USART1_RX  |
| 3            | USART1_TX  |
| 4            | I2C1_SCL   |
| 5            | I2C1_SDA   |
| 6            | SAFETY_BTN |
| 7            | SAFETY_LED |
| 8            | +3V3                            |
| 9            | BUZZER                          |
| 10           | GND                             |

## Специфікація

- Приймач GPS u-blox M8N
- Магнітометр IST8310
- Магнітометр HMC5983
- Магнітометр LIS3MDL
- Датчик тиску MS5611
- Світлодіоди RGB для показу статусу
  - NCP5623 I2C Driver
- Діаметр: 75 мм
- Вага: 34g

## Подальша інформація

- [Buy SmartAP GPS](https://sky-drones.com/navigation/smartap-gnss.html)
- [Documentation](https://docs.sky-drones.com/avionics/smartap-gnss)
- [CAD Models](https://docs.sky-drones.com/avionics/smartap-gnss/cad-model)
