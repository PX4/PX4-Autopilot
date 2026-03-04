# Holybro H-RTK M8P GNSS (Знято з виробництва)

:::warning
This GNSS has been discontinued, and is no longer commercially available.
:::

The [Holybro H-RTK M8P GNSS](https://holybro.com/collections/standard-h-rtk-series/products/h-rtk-m8p-gnss-series) is an [RTK GNSS module](../gps_compass/rtk_gps.md) series for the mass market.
This family is similar to the [H-RTK M9P](../gps_compass/rtk_gps_holybro_h-rtk-f9p.md) series but uses the smaller, lighter, and less expensive M8P u-blox RTK GNSS module (which still provides far superior position resolution than previous generations\_.

Є три моделі Holybro H-RTK M8P на вибір, кожна з різним дизайном антени, щоб задовольнити різні потреби.
Refer to the [Specification and Model Comparison section](#specification-and-model-comparison) for more details.

Використання RTK дозволяє PX4 отримати своє положення з точністю до сантиметрів, що набагато точніше, ніж може забезпечити звичайний GPS.

![h-rtk_rover](../../assets/hardware/gps/rtk_holybro_h-rtk-m8p_all_label.jpg)

## Де купити

- [H-RTK M8P (GPS RTK Mounts)](https://holybro.com/products/vertical-mount-for-h-rtk-helical)

## Налаштування

RTK setup and use on PX4 via _QGroundControl_ is largely plug and play \(see [RTK GPS](../gps_compass/rtk_gps.md) for more information\).

## Підключення та з'єднання

All H-RTK GNSS models come with a GH 10-pin connector/cable that is compatible with [Pixhawk 4](../flight_controller/pixhawk4.md).

:::info
The cables/connectors may need to be modified in order to connect to other flight controller boards (see [pin map](#pin_map)below).
:::

<a id="pin_map"></a>

## Карта виводів

![h-rtk_rover_pinmap](../../assets/hardware/gps/rtk_holybro_h-rtk-m8p_pinmap.jpg)

## Специфікація та порівняння моделей

![h-rtk_spec](../../assets/hardware/gps/rtk_holybro_h-rtk-m8p_spec.png)

## Аксесуари до GPS

[GPS Accessories (Holybro Website)](https://holybro.com/collections/gps-accessories)

![h-rtk](../../assets/hardware/gps/rtk_holybro_h-rtk_mount_3.png)
