# 测试 MC_05-室内飞行（手动模式）

## 何时使用此测试卡

- 新建初次飞行
- 当需要在受限区域复现问题时
- 可能存在稳定性问题的实验性构建
- 测试已更换和/或修改的硬件

## 解锁并起飞

❏ 设置飞行模式以稳定和布防

❏ 升高油门起飞

## 飞行

❏ 自稳

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response 1:1

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response 1:1

❏ 高度

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response 1:1

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to Climbs/Descend rate

## 降落

❏ 以稳定或高度模式着陆，油门低于 40％

❏ Upon touching ground, copter should disarm automatically within 2 seconds (disarm time set by parameter: [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND))

## 预期成果

- 当油门升高时，起飞应该是平稳的
- 在上述任何飞行模式中都不应出现振荡
- 着陆时，直升机不应在地面上反弹
