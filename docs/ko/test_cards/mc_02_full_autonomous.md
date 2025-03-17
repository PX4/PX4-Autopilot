# 시험 MC_02 - 완전 자동화

## Objective

To test the auto modes such as Mission, RTL, etc...

## Preflight

Plan a mission on the ground. Ensure the mission has

- Takeoff as first waypoint
- Changes in Altitude throughout the mission
- Last waypoint is an RTL
- Duration of 5 to 6 minutes

## Flight Tests

❏ Mission

&nbsp;&nbsp;&nbsp;&nbsp;❏ Auto take-off

&nbsp;&nbsp;&nbsp;&nbsp;❏ Changes in Altitude throughout the mission

&nbsp;&nbsp;&nbsp;&nbsp;❏ First waypoint set to Takeoff

&nbsp;&nbsp;&nbsp;&nbsp;❏ Enable Mission End RTL

&nbsp;&nbsp;&nbsp;&nbsp;❏ Duration of 5 to 6 minutes

&nbsp;&nbsp;&nbsp;&nbsp;❏ Auto Disarm on land

❏ Mission + Manual arm

&nbsp;&nbsp;&nbsp;&nbsp;❏ Arm in any manual mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Engage Auto (Mission Mode) to trigger take-off

&nbsp;&nbsp;&nbsp;&nbsp;❏ Observe tracking, cornering and proper RTL performance

&nbsp;&nbsp;&nbsp;&nbsp;❏ Upon touching ground, copter should disarm automatically within 2 seconds (disarm time set by parameter: [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND))

❏ RTL

&nbsp;&nbsp;&nbsp;&nbsp;❏ Arm and takeoff in position mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Fly out ~10m from start point

&nbsp;&nbsp;&nbsp;&nbsp;❏ Engage RTL Mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Observe tracking, cornering and proper RTL performance

## 예상 결과

- 추력을 올릴 때 서서히 이륙한다
- 첫 시도시 임무 내용이 올라가야 함
- 자동 모드 인가시 기체는 자동으로 이륙해야 함
- Vehicle shoud adjust height to RTL altitude before returning home
- 지면에 착륙시, 콥터가 지면에서 튀면 안됨

<!--
MC_002 - Full autonomous

-	Make sure the auto-disarm is enabled
-	QGC open test1_mission.plan and sync to the vehicle
-	Takeoff from QGC start mission slider
-	Check the vehicle completes the mission
-	Let the vehicle to auto land, take manual control if needed and explain the reason in log description.
-	Check the vehicle disarms by itself.
-->
