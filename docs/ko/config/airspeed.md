# 항속센서 보정

:::info
[Airspeed sensors](../sensor/airspeed.md) are highly recommended for Fixed-wing and VTOL vehicles.
:::

:::warning
Unlike most other sensor drivers, the airspeed sensor drivers are not automatically started.
Before calibration they must be [enabled via the corresponding parameter](../advanced_config/parameters.md):

- Sensirion SDP3X ([SENS_EN_SDP3X](../advanced_config/parameter_reference.md#SENS_EN_SDP3X))
- TE MS4525 ([SENS_EN_MS4525DO](../advanced_config/parameter_reference.md#SENS_EN_MS4525DO))
- TE MS5525 ([SENS_EN_MS5525DS](../advanced_config/parameter_reference.md#SENS_EN_MS5525DS))
- Eagle Tree airspeed sensor ([SENS_EN_ETSASPD](../advanced_config/parameter_reference.md#SENS_EN_ETSASPD))

:::

## 보정 절차

항속 센서 보정 절차

1. Start _QGroundControl_ and connect the vehicle.

2. Enable the airspeed sensors if not already done (as in _warning_ above).

3. Select **"Q" icon > Vehicle Setup > Sensors** (sidebar) to open _Sensor Setup_.

4. Click the **Airspeed** sensor button.

  ![Airspeed calibration](../../assets/qgc/setup/sensor/sensor_airspeed.jpg)

5. 센서로 부는 바람을 막으십시오 (예: 손을 컵 모양으로 감쌀 수 있습니다).
  피톳 튜브의 구멍을 막지 않도록 주의하십시오.

6. Click **OK** to start the calibration.

7. 피톳 튜브의 끝에 입으로 바람을 불어 보정 완료 신호를 보냅니다.

  :::tip
  Blowing into the tube is also a basic check that the dynamic and static ports are installed correctly.
  교체한  센서는 튜브에 바람을 불어 넣을 때 큰 음의 차압을 판독하고 보정이 오류와 함께 중단됩니다.

:::

8. _QGroundControl_ then tells you if the calibration was successful or not.

## 추가 정보

- [QGroundControl User Guide > Sensors](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/sensors_px4.html#airspeed)
