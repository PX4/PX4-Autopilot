# 스마트 배터리

스마트 배터리는 자동조종장치가 "둔감한" 배터리에 대해 추정할 수 있는 것보다 배터리 상태에 대한 더 정확한(그리고 종종 더 자세한) 정보를 제공합니다.
이를 통하여 보다 안정적인 비행 계획 실패 조건 알림이 가능합니다.
정보에는 남은 충전량, 비우기 시간 (예상), 셀 전압 (최대/최소 정격, 현재 전압 등), 온도, 전류, 오류 정보, 배터리 공급 업체, 화학 물질 등이 포함될 수 있습니다.

PX4는 (최소한) 다음과 같은 스마트 배터리를 지원합니다.

- [Rotoye Batmon](../smart_batteries/rotoye_batmon.md)

### 추가 정보

- [Mavlink Battery Protocol](https://mavlink.io/en/services/battery.html)
- [batt_smbus](../modules/modules_driver.md) - PX4 SMBus Battery Driver docs
- [Safety > Low Battery Failsafe](../config/safety.md#battery-level-failsafe).
