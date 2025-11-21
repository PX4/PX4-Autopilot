# Holybro H-RTK F9P GNSS

:::tip
[Holybro H-RTK ZED-F9P Rover](../dronecan/holybro_h_rtk_zed_f9p_gps.md) is an upgraded version of this module.
:::

The [Holybro H-RTK F9P GNSS](https://holybro.com/products/h-rtk-f9p-gnss-series) is an multi-band high-precision [RTK GNSS System](../gps_compass/rtk_gps.md) series launched by Holybro.
This family is similar to the [H-RTK M8P](../gps_compass/rtk_gps_holybro_h-rtk-m8p.md) series, but uses multi-band RTK with faster convergence times and reliable performance, concurrent reception of GPS, GLONASS, Galileo and BeiDou, and faster update rate for highly dynamic and high volume applications with centimeter-accuracy.
It uses a u-blox F9P module, a IST8310 compass, and a tri-colored LED indicator.
It also has an integrated safety switch for a simple and convenient operation.

There are three models of Holybro H-RTK F9P to choose from, each with different antenna design to meet different needs.
Refer to [Specification and Model Comparison section](#specification-and-model-comparison) for more details.

Using RTK allows PX4 to get its position with centimetre-level accuracy, which is much more accurate than can be provided by a normal GPS.

![h-rtk](../../assets/hardware/gps/rtk_holybro_h-rtk-f9p_all_label.jpg)

## 购买渠道

- [H-RTK F9P (Holybro Website)](https://holybro.com/products/h-rtk-f9p-gnss-series)
- [H-RTK Accessories (Holybro Website)](https://holybro.com/collections/h-rtk-gps)

## 配置

RTK setup and use on PX4 via _QGroundControl_ is largely plug and play \(see [RTK GPS](../gps_compass/rtk_gps.md) for more information\).

## 接线和连接

H-RTK Helical models come with both GH 10-pin & 6-pin cables that are compatible with the GPS1 & GPS2 ports on flight controllers that use the Pixhawk Connector Standard, such as [Pixhawk 4](../flight_controller/pixhawk4.md) and [Pixhawk 5x](../flight_controller/pixhawk5x.md).

The H-RTK Rover Lite comes in two version.
The standard version comes with 10 pin connector for the `GPS1` port.
The "2nd GPS" version comes with 6 pin connector for the `GPS2` port.
This is used as a secondary GPS for [Dual GPS Systems](../gps_compass/index.md#dual_gps).

:::info
The cables/connectors may need to be modified in order to connect to other flight controller boards (see [Pin Map](#pin-map) below).
:::

## Pin Map

![h-rtk-f9p_rover_pinmap](../../assets/hardware/gps/rtk_holybro_h-rtk_helical_pinmap.jpg)

![h-rtk-f9p_helical_pinmap](../../assets/hardware/gps/rtk_holybro_h-rtk_rover_lite_pinmap.jpg)

## Specification and Model Comparison

![h-rtk-f9p_spec](../../assets/hardware/gps/rtk_holybro_h-rtk-f9p_spec.png)

## GPS Accessories

[H-RTK Mount (Holybro Website)](https://holybro.com/products/vertical-mount-for-h-rtk-helical)

![h-rtk](../../assets/hardware/gps/rtk_holybro_h-rtk_mount_3.png)
