# CUAV C-RTK

The [CUAV C-RTK GPS receiver](https://www.cuav.net/en/c_rtk_9ps/) is an [RTK GPS module](../gps_compass/rtk_gps.md) for the mass market.
Повна система RTK складається щонайменше з двох модулів C-RTK (один для базової станції, а інший для літака). Використовуючи RTK, PX4 може визначати своє місцезнаходження з точністю до сантиметра, що набагато точніше, ніж може забезпечити звичайний GPS.

<img src="../../assets/hardware/gps/rtk_c-rtk.jpg" width="500px" title="C-RTK" />

## Де купити

- [cuav taobao](https://item.taobao.com/item.htm?id=565380634341&spm=2014.21600712.0.0)
- [cuav aliexpress](https://www.aliexpress.com/store/product/CUAV-NEW-Flight-Controller-GPS-C-RTK-differential-positioning-navigation-module-GPS-for-PIX4-Pixhawk-pixhack/3257035_32853894248.html?spm=2114.12010608.0.0.75592fadQKPPEn)

## Налаштування

RTK setup and use on PX4 via _QGroundControl_ is largely plug and play \(see [RTK GPS](../gps_compass/rtk_gps.md) for more information\).

## Підключення та з'єднання

C-RTK GPS comes with a cable that terminates in a 6-pin connector and 4-pin connector that are compatible with [Pixhack v3](https://doc.cuav.net/flight-controller/pixhack/en/quick-start-pixhack-v3x.html#gps--compass).
6-контактний роз'єм забезпечує інтерфейс для RTK GPS і повинен бути підключений до GPS-порту польотного контролера.
4-контактний роз'єм - це GPS-інтерфейс m8n (стандартний), який призначений для (додаткового) використання в якості другого GPS.

:::tip
At time of writing PX4 does not yet fully support a second GPS. 4-контактний порт не потрібно підключати.
:::

<img src="../../assets/hardware/gps/rtk_cuav_c-rtk_to_6pin_connector.jpg" width="500px" title="C-RTK_6PIN" />

Може знадобитися модифікація кабелів/роз'ємів для підключення до інших плат польотних контролерів. The pin mappings for _Pixhawk 3 Pro_ and _Pixracer_ are shown below.

### Схема розташування виводів

Нижче наведено розводку GPS-приймача C-RTK. Це може бути використано для модифікації роз'єму для інших плат автопілота.

| pin | C-RTK GPS 6P                | pin | Pixhawk 3 Pro GPS           | C-RTK GPS 4P                |
| --- | --------------------------- | --- | --------------------------- | --------------------------- |
| 1   | SDA                         | 1   | VCC                         |                             |
| 2   | SCL                         | 2   | GPS_TX |                             |
| 3   | GPS_RX | 3   | GPS_RX | GPS_RX |
| 4   | GPS_TX | 4   | SCL                         | GPS_TX |
| 5   | VCC_5V | 5   | SDA                         | VCC_5v |
| 6   | GND                         | 6   | GND                         | GND                         |
