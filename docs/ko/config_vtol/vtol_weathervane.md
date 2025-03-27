# VTOL 풍향계 특징

The _weather vane_ feature automatically turns a VTOL vehicle to face its nose into the relative wind during hover flight.
이 기능은 안정성을 향상시킵니다 (측면에서 바람이 기체를 뒤집을 가능성을 줄임).

The feature is [enabled by default](#configuration) on VTOL hybrid vehicles flying in multicopter mode.

:::info
Weather vane functionality is not supported on pure multirotors.
:::

## 수동 모드 동작 방식

The weather vane feature will only take effect in [Position mode](../flight_modes_mc/position.md) (not other manual MC modes).

사용자는 풍향계 컨트롤러가 기체의 기수를 바람으로 변경중에도, 요 스틱을 사용하여 요 속도를 제어할 수 있습니다.
목표 요율은 풍향계 요율과 사용자가 명령한 요율의 합입니다.

## 임무 모드 동작 방식

In [Mission mode](../flight_modes_vtol/mission.md) the weather vane feature will always be active when the parameter is enabled.
임무에서 명령된 모든 요 각도는 무시됩니다.

<a id="configuration"></a>

## 설정

This functionality is configured using the [WV\_\* parameters](../advanced_config/parameter_reference.md#WV_EN).

| 매개변수                                                                                                             | 설명                                              |
| ---------------------------------------------------------------------------------------------------------------- | ----------------------------------------------- |
| [WV_EN](../advanced_config/parameter_reference.md#WV_EN)                                    | 풍향계 활성화                                         |
| [WV_ROLL_MIN](../advanced_config/parameter_reference.md#WV_ROLL_MIN)   | 요율을 요구하는 풍향계 컨트롤러의 최소 롤 각도 설정점. |
| [WV_YRATE_MAX](../advanced_config/parameter_reference.md#WV_YRATE_MAX) | 풍향계 컨트롤러가 요구할 수 있는 최대 요 레이트.    |

## 어떻게 작동합니까?

호버 비행 중에 기체는 위치를 유지하기 위하여 바람에 의해 가해지는 항력을 극복하여야 합니다.
이를 달성하는 유일한 방법은 추력 벡터를 상대적인 바람으로 기울이는 것입니다 (말 그대로 바람에 '기댄다').
추력 벡터를 추적하여 풍향을 추정할 수 있습니다.
풍향계 컨트롤러는 기체 기수를 예상 풍향으로 바꾸는 요 레이트를 명령하는 데 사용됩니다.
