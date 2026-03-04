# CUAV NEO 3 GPS

<Badge type="tip" text="PX4 v1.13" />

Цей NEO 3 GPS виготовлений CUAV.
Вона інтегрує Ublox M9N, IST8310, світлодіодні лампи трьох кольорів та безпечні перемикачі, і сумісна з контролерами стандарту CUAV та Pixhawk.

![Hero image of Neo3 GPS](../../assets/hardware/gps/cuav_gps_neo3/neo_3.jpg)

## Технічні характеристики

| Апаратне забезпечення(Hardware) | Тип                                                                                                                        |
| :------------------------------------------------- | :------------------------------------------------------------------------------------------------------------------------- |
| Компас                                             | IST8310                                                                                                                    |
| Приймач GNSS                                       | UBLOX NEO M9N                                                                                                              |
| RGB привід                                         | NC5623C                                                                                                                    |
| Зумер                                              | Пасивний зумер                                                                                                             |
| Запобіжний перемикач                               | Фізична кнопка                                                                                                             |
| GNSS                                               | Beidou, Galileo, GLONASS, GPS                                                                                              |
| Система розширення GNSS                            | SBAS:WAAS,EGNOS,MSAS<br>QZSS:L1s(SAIF)<br>other：RTCM3.3 |
| Кількість одночасних GNSS                          | 4                                                                                                                          |
| Смуга частот                                       | GPS:L1C/A<br>GLONASS:L10F<br>Beidou:B1I<br>Galileo:E1B/C   |
| Горизонтальна точність                             | 2.0M                                                                                                       |
| Точність швидкості                                 | 0.05M/S                                                                                                    |
| Частота оновлення навігації                        | 25Hz(Max)                                                                                               |
| Отримання даних                                    | Холодний старт: 24 с<br>Гарячий старт: 2 с<br>Допоміжний старт: 2 с        |
| Кількість супутників(MAX)       | 32+                                                                                                                        |
| Чутливість                                         | Відстеження та нав-167dBm<br>Холодний старт Гарячий старт-148dBm<br>Повторний збір -160dBm                                 |
| Протокол                                           | UART+IO+I2C                                                                                                                |
| Тип порту                                          | GHR-10V-S                                                                                                                  |
| Підтримувані автопілоти включають                  | серія CUAV,<br>серія Pixahwk                                                                                               |
| Фільтрування хвиль                                 | SAW+LNA+SAW                                                                                                                |
| Протиелектромагнітний/радіочастотний перешкід      | EMI+RFI                                                                                                                    |
| Оновлення прошивки                                 | підтримка                                                                                                                  |
| Вхідна напруга                                     | 5V                                                                                                                         |
| Робоча температура                                 | -10~70℃                                                                                                    |
| Розмір                                             | 60\*60\*16MM                                                                                                               |
| Вага                                               | 33g                                                                                                                        |

## Розміри

![Neo 3 Size](../../assets/hardware/gps/cuav_gps_neo3/neo_3_size.png)

## Схема розташування виводів

![Neo 3 Pinouts](../../assets/hardware/gps/cuav_gps_neo3/neo_3_pinouts.png)

## Де купити

- [CUAV](https://cuav.en.alibaba.com/product/1600217379204-820872629/CUAV_NEO_3_M9N_GPS_Module_for_Pixhawk_Compass_gps_tracker_navigation_gps.html?spm=a2700.shop_oth.74.1.636e28725EvVHb)

## Підключення та з'єднання

Схема підключення та з'єднання Neo3

![Neo3 wiring and connection diagram](../../assets/hardware/gps/cuav_gps_neo3/neo_3_connect.png)

## Додаткова інформація

- [CUAV docs](https://doc.cuav.net/gps/neo-series-gnss/zh-hans/neo-3.html)
