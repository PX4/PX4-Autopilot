# GeoFence

GeoFence는 기체의 비행 영역을 정의하는 가상 경계입니다.
GeoFence는 기체가 RC 무선 조종기의 영향권을 벗어나서 안전하지 않거나 제한된 공역을 비행하는 것을 방지합니다.

두 가지 방법으로 GeoFence를 설정할 수 있습니다.

- 간단한 실린더 형태를 이용한  "사고방지" 지오펜스를 정의할 수 있습니다.
- More complicated geometries can be defined using a Geofence Plan (_QGroundControl_).

:::info
GeoFences apply in all modes, including both missions and manual flight.
:::

## 사고 방지 GeoFence

The [Geofence Failsafe](../config/safety.md#geofence-failsafe) defines a cylinder centered on the home position, with a specified maximum radius and altitude.

또한 펜스가 위반되는 경우 "사고 방지 액션" 설정이 포함됩니다.
This may simply be a warning notification, but more commonly a vehicle will immediately [Return](../flight_modes/return.md) to a safe location.

For more information see: [Safety > Geofence Failsafe](../config/safety.md#geofence-failsafe).

## GeoFence 계획

PX4는 포함 (내부 비행) 또는 제외 (외부 비행) 영역으로 정의되는 여러개의 원형 및 다각형으로 구성된 복잡한 GeoFence 경계를 지원합니다.

The Geofence is planned in _QGroundControl_ alongside the mission and rally points.

![Geofence Plan](../../assets/qgc/plan_geofence/geofence_overview.jpg)

Geofence planning is fully documented in [Plan View > GeoFence](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/plan_geofence.html) (QGroundControl User Guide).

요약

1. Open _QGroundControl > Plan View_.
2. Select the _Plan Type_ radio button: **Fence**.
   This will display the _GeoFence Editor_.
   ![Geofence Plan](../../assets/qgc/plan_geofence/geofence_editor.jpg)
3. Select the **Polygon Fence** or **Circular Fence** button to add a _basic_ fence of the desired type to the map.
   편집기에서 울타리 유형을 추가합니다.
4. 지도를 사용하여 울타리의 모양과 위치를 구성하십시오.
   - 펜스 중앙 마커를 사용하여 펜스를 올바른 위치로 이동할 수 있습니다.
   - 원형 울타리 테두리의 마커를 사용하여 반경을 변경할 수 있습니다.
   - 모서리 (정점)의 마커를 사용하여 다각형의 형상을 변경할 수 있습니다.
      기존 마커 사이의 선을 따라 중간을 클릭하면 추가 정점이 생성됩니다.
5. Use the _Geofence Editor_ to set a fence as an inclusion or exclusion, and to select a fence to edit (**Edit** radio button) or Delete (**Del** button).
6. 필요한 만큼 울타리를 추가하십시오.
7. Once finished, click on the **Upload** button (top right) to send the fence (along with rally points and mission) to the vehicle.
8. Set the breach action in the [Geofence Failsafe](../config/safety.md#geofence-failsafe).

:::info
Any geofence that does not include the Home position will be rejected by the flight controller and not uploaded.
If the vehicle is flying, also any geofence that would immediately get breached after the upload is rejected.
:::

:::info
PX4 implements the MAVLink [Mission microservice](https://mavlink.io/en/services/mission.html), which includes support for GeoFences.
:::
