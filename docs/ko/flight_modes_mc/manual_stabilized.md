# Stabilized Mode (Multicopter)

<img src="../../assets/site/difficulty_medium.png" title="Medium difficulty to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />&nbsp;

The _Stabilized_ manual mode stabilizes and levels the multicopter when the RC control sticks are centred.
To move/fly the vehicle you move the sticks outside of the centre.

:::info
This mode is also enabled if you set the flight mode to _Manual_.
:::

When sticks are outside the centre, the roll and pitch sticks control the _angle_ of the vehicle (attitude) around the respective axes, the yaw stick controls the rate of rotation above the horizontal plane, and the throttle controls altitude/speed.

조종 스틱을 놓으면 중앙 데드 존으로 돌아갑니다.
롤 포크와 피치 스틱이 중앙에 오면 멀티 피터가 수평을 유지하고 정지합니다.
The vehicle will then hover in place/maintain altitude - provided it is properly balanced, throttle is set appropriately (see [below](#params)), and no external forces are applied (e.g. wind).
기체는는 바람 방향으로 표류하게 되며, 고도를 유지하기 위해서는 스로틀을 제어하여야 합니다.

![MC Manual Flight](../../assets/flight_modes/stabilized_mc.png)

## Technical Description

RC mode where centered sticks level vehicle.

:::info
[Altitude mode](../flight_modes_mc/altitude.md) additionally stabilizes the vehicle altitude when sticks are centred, and [Position mode](../flight_modes_mc/position.md) stabilizes both altitude and position over ground.
:::

조종사의 입력은 롤 및 피치 각 명령과 요 율 명령으로 전달됩니다.
Throttle is rescaled (see [below](#params)) and passed directly to control allocation.
자동 조종 장치는 자세를 제어합니다. 즉, RC 스틱이 컨트롤러 데드 존 내부에 집중 될 때 롤과 피치 각을 제로로 조절합니다 (결과적으로 태도가 수평이 됨).
자동 조종 장치는 바람 (또는 다른 원인)으로 인한 드리프트를 보상하지 않습니다.

- 중앙 스틱 (데드밴드 내부) :
  - Roll/Pitch sticks level vehicle.
- Outside center:
  - Roll/Pitch sticks control tilt angle in those orientations, resulting in corresponding left-right and forward-back movement.
  - Throttle stick controls up/down speed (and movement speed in other axes).
  - 요 스틱은 수평면 위의  회전 각속도를 제어합니다.
- Manual control input is required (such as RC control, joystick).
  - Roll, Pitch: Assistance from autopilot to stabilize the attitude.
    Position of RC stick maps to the orientation of vehicle.
  - Throttle: Manual control via RC sticks. RC input is sent directly to control allocation.
  - Yaw: Assistance from autopilot to stabilize the attitude rate.
    Position of RC stick maps to the rate of rotation of vehicle in that orientation.

<a id="params"></a>

## 매개변수

| 매개변수                                                                                                                                         | 설명                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| -------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="MPC_THR_HOVER"></a>[MPC_THR_HOVER](../advanced_config/parameter_reference.md#MPC_THR_HOVER) | Hover throttle that is output when the throttle stick is centered and `MPC_THR_CURVE` is set to default.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| <a id="MPC_THR_CURVE"></a>[MPC_THR_CURVE](../advanced_config/parameter_reference.md#MPC_THR_CURVE) | 스로틀 스케일링을 정의합니다. By default this is set to **Rescale to hover thrust**, which means that when the throttle stick is centered the configured hover throttle is output (`MPC_THR_HOVER`) and the stick input is linearly rescaled below and above that (allowing for a smooth transition between Stabilized and Altitude/Position control). <br>강력한 기체의 경우 호버 스로틀이 매우 낮아 (예 : 20 % 미만) 스로틀 입력이 왜곡 될 수 있습니다. 즉, 여기서 추력의 80 %는 스틱 입력의 상단 절반으로, 하단은 20 %로 제어됩니다. If needed `MPC_THR_CURVE` can be set to **No Rescale** so that there is no rescaling (stick input to throttle mapping is independent of `MPC_THR_HOVER`). |
