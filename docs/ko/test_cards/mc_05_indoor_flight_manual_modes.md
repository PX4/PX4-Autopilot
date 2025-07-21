# 시험 MC_05 - 실내 비행 (수동 모드)

## 이 시험 카드를 사용하는 경우

- 새 처녀비행 수립
- 지정 영역에서 나타나는 문제를 재현하고자할 때
- 안정성 문제를 내포할 수 있는 시험작
- 교체하거나 수정한 하드웨어의 시험

## 시동 및 이륙

❏  Stabilize로 비행 모드를 설정하고 시동

❏ 추진력을 올려 이륙

## 비행

❏ 안정화 상태

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response 1:1

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response 1:1

❏ 고도

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response 1:1

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to Climbs/Descend rate

## 착륙

❏ 안정화 상태 또는 추진력을 40% 이하로 둔 상태의 고도 제어 모드에서 착륙

❏ Upon touching ground, copter should disarm automatically within 2 seconds (disarm time set by parameter: [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND))

## 예상 결과

- 추력을 올릴 때 서서히 이륙한다
- 위에 언급한 어떤 비행 모드에서도 떨림이 나타나서는 안됨
- 지면에 착륙시, 콥터가 지면에서 튀면 안됨
