# 地理围栏

围栏是一个虚拟边界，用于定义机体可以在哪些地方行驶。
围栏可用于防止机体飞出遥控器的范围，或进入不安全或受限制的空域。

PX4提供了两个独立的机制来指定围栏：

- 一个是基本的“故障保护”地理围栏，定义了一个简单的圆柱体。
- More complicated geometries can be defined using a Geofence Plan (_QGroundControl_).

:::info
GeoFences apply in all modes, including both missions and manual flight.
:::

## 故障保护地理围栏

The [Geofence Failsafe](../config/safety.md#geofence-failsafe) defines a cylinder centered on the home position, with a specified maximum radius and altitude.

设置中还包括越界时的故障保护动作。
This may simply be a warning notification, but more commonly a vehicle will immediately [Return](../flight_modes/return.md) to a safe location.

For more information see: [Safety > Geofence Failsafe](../config/safety.md#geofence-failsafe).

## 地理围栏规划

PX4 支持由多个圆形和多边形区域组成的复杂地理围栏边界，这些区域可以定义为包含（禁出）或排除（禁入）区域。

The Geofence is planned in _QGroundControl_ alongside the mission and rally points.

![Geofence Plan](../../assets/qgc/plan_geofence/geofence_overview.jpg)

Geofence planning is fully documented in [Plan View > GeoFence](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/plan_geofence.html) (QGroundControl User Guide).

总结：

1. Open _QGroundControl > Plan View_.
2. Select the _Plan Type_ radio button: **Fence**.
   This will display the _GeoFence Editor_.
   ![Geofence Plan](../../assets/qgc/plan_geofence/geofence_editor.jpg)
3. Select the **Polygon Fence** or **Circular Fence** button to add a _basic_ fence of the desired type to the map.
   这也在编辑器中增加了此类围栏的条目。
4. 在地图上调整围栏的形状和位置。
   - 围栏中心的圆点可以用来调整围栏的位置。
   - 边界上的圆点可以用来调整半径。
   - 角（顶点）上的圆点可以用来改变多边形的形状。
      点击线段中间可以在两个顶点中添加新的顶点。
5. Use the _Geofence Editor_ to set a fence as an inclusion or exclusion, and to select a fence to edit (**Edit** radio button) or Delete (**Del** button).
6. 可添加任意数量的围栏
7. Once finished, click on the **Upload** button (top right) to send the fence (along with rally points and mission) to the vehicle.
8. Set the breach action in the [Geofence Failsafe](../config/safety.md#geofence-failsafe).

:::info
Any geofence that does not include the Home position will be rejected by the flight controller and not uploaded.
If the vehicle is flying, also any geofence that would immediately get breached after the upload is rejected.
:::

:::info
PX4 implements the MAVLink [Mission microservice](https://mavlink.io/en/services/mission.html), which includes support for GeoFences.
:::
