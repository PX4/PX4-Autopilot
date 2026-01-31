# 测试 MC_02-完全自主

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

## 预期成果

- 当油门升高时，起飞应该是平稳的
- 任务应该在第一次尝试时上传
- 使用 Auto 时飞机应自动起飞
- Vehicle shoud adjust height to RTL altitude before returning home
- 着陆时，直升机不应在地面上反弹

<!--
MC_002 - Full autonomous

-	Make sure the auto-disarm is enabled
-	QGC open test1_mission.plan and sync to the vehicle
-	Takeoff from QGC start mission slider
-	Check the vehicle completes the mission
-	Let the vehicle to auto land, take manual control if needed and explain the reason in log description.
-	Check the vehicle disarms by itself.
-->
