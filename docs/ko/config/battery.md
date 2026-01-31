# 배터리 파워 모듈 설정

전원 설정 방법에 대하여 설명합니다.

:::info
These instructions require that the vehicle has a [Power Module (PM)](../power_module/index.md), or other hardware that can measure the battery voltage and (optionally) the current.
:::

:::tip
This tuning is not needed for [Smart/MAVLink Batteries](../smart_batteries/index.md): batteries that can supply a reliable indication of remaining charge.
:::

## 개요

Battery Estimation Tuning uses the measured voltage and current (if available) to estimate the remaining battery capacity.
This is important because it allows PX4 to take action when the vehicle is close to running out of power and crashing (and also to prevent battery damage due to deep-discharge).

PX4는 여러가지 효과적인 용량 추정 방법을 제공합니다.

1. [Basic Battery Settings](#basic_settings) (default): raw measured voltage is compared to the range between "empty" and "full" voltages.
   측정 전압 (및 해당 용량)은 부하시 변동으로 인하여 대략적인 추정치입니다.
2. [Voltage-based Estimation with Load Compensation](#load_compensation): Counteracts the effects of loading on the capacity calculation.
3. [Voltage-based Estimation with Current Integration](#current_integration): Fuses the load-compensated voltage-based estimate for the available capacity with a current-based estimate of the charge that has been consumed.
   그 결과 스마트 배터리와 비슷한 용량 추정치를 얻을 수 있습니다.

기타 다른 방법들은 이러한 방법들을 응용한 것입니다.
사용하는 접근 방식은 차량의 전원 모듈이 전류를 측정 가능 여부에 따라 다릅니다.

:::info
The instructions below refer to battery 1 calibration parameters: `BAT1_*`.
Other batteries use the `BATx_*` parameters, where `x` is the battery number.
All battery calibration parameters [are listed here](../advanced_config/parameter_reference.md#battery-calibration).
:::

:::tip
In addition to PX4 configuration discussed here, you should ensure that the ESC's low voltage cutoff is either disabled or set below the expected minimum voltage.
이렇게 하면 PX4에서 배터리 오류 안전 동작을 관리하고 배터리가 충전되어있는 동안 ESC가 차단되지 않도록합니다 (선택한 "빈 배터리"설정에 따라).
:::

:::tip
[Battery Chemistry Overview](../power_systems/battery_chemistry.md) explains the difference between the main battery types, and how that impacts the battery settings.
:::

## Basic Battery Settings (default) {#basic_settings}

기본 배터리 설정은 용량 추정 기본 방법을 사용하도록 PX4를 설정합니다.
이 방법은 측정된 원시 배터리 전압을 "빈"셀과 "충전"셀 (셀 수에 따라 조정 됨)에 대한 셀 전압 범위와 비교합니다.

:::info
This approach results in relatively coarse estimations due to fluctuations in the estimated charge as the measured voltage changes under load.
:::

배터리 1의 기본 설정 방법

1. Start _QGroundControl_ and connect the vehicle.
2. Select **"Q" icon > Vehicle Setup > Power** (sidebar) to open _Power Setup_.

배터리 특성을 나타내는 기본 설정이 제공됩니다.
아래 섹션에서는 각 필드에 대해 설정할 값들을 설명합니다.

![QGC Power Setup](../../assets/qgc/setup/power/qgc_setup_power_px4.png)

:::info
At time of writing _QGroundControl_ only allows you to set values for battery 1 in this view.
For vehicles with multiple batteries you'll need to directly [set the parameters](../advanced_config/parameters.md) for battery 2 (`BAT2_*`), as described in the following sections.
:::

### 셀의 갯수(직렬 연결)

이것은 배터리에 직렬로 연결된 셀 수를 설정합니다.
일반적으로 배터리에 "S"앞에 숫자로 표시합니다 (예 : "3S", "5S").

:::info
The voltage across a single galvanic battery cell is dependent on the [chemical properties of the battery type](../power_systems/battery_chemistry.md).
Lithium-Polymer (LiPo) batteries and Lithium-Ion batteries both have the same _nominal_ cell voltage of 3.7V.
In order to achieve higher voltages (which will more efficiently power a vehicle), multiple cells are connected in _series_.
터미널의 배터리 전압은 셀 전압의 배수입니다.
:::

셀 개수가 제공되지 않은 경우 배터리 전압을 단일 셀의 공칭 전압으로 나누어 계산할 수 있습니다.
아래의 표는 배터리의 전압-셀 관계를 나타냅니다.

| 셀  | LiPo (V) | LiIon (V) |
| -- | --------------------------- | ---------------------------- |
| 1S | 3.7         | 3.7          |
| 2S | 7.4         | 7.4          |
| 3S | 11.1        | 11.1         |
| 4S | 14.8        | 14.8         |
| 5S | 18.5        | 18.5         |
| 6S | 22.2        | 22.2         |

:::info
This setting corresponds to [parameters](../advanced_config/parameters.md): [BAT1_N_CELLS](../advanced_config/parameter_reference.md#BAT1_N_CELLS) and [BAT2_N_CELLS](../advanced_config/parameter_reference.md#BAT2_N_CELLS).
:::

### 충전 완료 전압 (셀당)

This sets the _nominal_ maximum voltage of each cell (the lowest voltage at which the cell will be considered "full").

이 값은 배터리의 공칭 최대 셀 전압보다 약간 낮게 설정하여야하지만, 몇 분간의 비행후에도 예상 용량이 100%가 될 정도로 낮지 않아야 합니다.

사용할 적절한 값은 다음과 같습니다.

- **LiPo:** 4.05V (default in _QGroundControl_)
- **LiIon:** 4.05V

:::info
The voltage of a full battery may drop a small amount over time after charging.
최대 값보다 약간 낮게 설정하여 이 하락값을 보정합니다.
:::

:::info
This setting corresponds to [parameters](../advanced_config/parameters.md): [BAT1_V_CHARGED](../advanced_config/parameter_reference.md#BAT1_V_CHARGED) and [BAT2_V_CHARGED](../advanced_config/parameter_reference.md#BAT2_V_CHARGED).
:::

### 방전 전압 (셀 당)

이는 각 셀의 공칭 최소 안전 전압을 설정합니다. 이 전압 미만을 사용하면 배터리가 손상될 수 있습니다.

:::info
There is no single value at which a battery is said to be empty.
너무 낮은 값을 선택하면 과방전으로 인해 배터리가 손상될 수 있습니다 (그리고, 기체 충돌이 발생할 수 있습니다).
너무 높은 값을 선택하면 비행 시간이 줄어듭니다.
:::

최소 셀당 전압에 대한 경험 규칙

| 단계                                                                  | LiPo (V) | LiIon (V) |
| ------------------------------------------------------------------- | --------------------------- | ---------------------------- |
| 보수적 (무부하 전압)                                     | 3.7         | 3                            |
| "Real" minimum (voltage under load/while flying) | 3.5         | 2.7          |
| 배터리 손상 (부하 전압)                                   | 3.0         | 2.5          |

:::tip
Below the conservative range, the sooner you recharge the battery the better - it will last longer and lose capacity slower.
:::

:::info
This setting corresponds to [parameter](../advanced_config/parameters.md): [BAT1_V_EMPTY](../advanced_config/parameter_reference.md#BAT1_V_EMPTY) and [BAT2_V_EMPTY](../advanced_config/parameter_reference.md#BAT2_V_EMPTY).
:::

### 전압 분배기

If you have a vehicle that measures voltage through a power module and the ADC of the flight controller then you should calibrate the measurements once per power module.
To calibrate, the actual voltage from the battery is measured (using a multimeter) and compared to the value provided by the power module.
This is used to calculate a "voltage divider" value, which can subsequently be used to scale the power module measurement to the correct value.

The easiest way to perform this calibration is by using _QGroundControl_ and following the step-by-step guide on [Setup > Power Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/power.html) (QGroundControl User Guide).

:::info
This setting corresponds to parameters: [BAT1_V_DIV](../advanced_config/parameter_reference.md#BAT1_V_DIV) and [BAT2_V_DIV](../advanced_config/parameter_reference.md#BAT2_V_DIV).
:::

### Amps per volt {#current_divider}

:::tip
This calibration is not needed if your power module does not provide current measurements.
:::

Current measurements are used (by default) for [Load Compensation](#load_compensation) and [Current Integration](#current_integration) if provided by the power module.
The amps per volt divider must be calibrated to ensure an accurate current measurement.

The easiest way to calibrate the dividers is by using _QGroundControl_ and following the step-by-step guide on [Setup > Power Setup](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/power.html) (QGroundControl User Guide).

:::info
This setting corresponds to parameter(s): [BAT1_A_PER_V](../advanced_config/parameter_reference.md#BAT1_A_PER_V) and [BAT2_A_PER_V](../advanced_config/parameter_reference.md#BAT2_A_PER_V).
:::

## Voltage-based Estimation with Load Compensation {#load_compensation}

When a current flows through a battery, the internal resistance causes a voltage drop, reducing the measured output voltage of the battery compared to its open-circuit (no-load) voltage.
When using the [basic configuration](#basic_settings), the measured output voltage is what is used to estimate the available capacity, which means that the battery level will appear to fluctuate when you fly up and down, or otherwise change the load on the battery.

_Load compensation_ uses a measured or estimated value for the internal resistance to correct for changes under load, resulting in far less variation in the estimated capacity when flying.
This is enabled by default when using a power module that provides current measurements.

To use the load compensation first set the [basic configuration](#basic_settings).
The _Empty Voltage_ ([BATn_V_EMPTY](../advanced_config/parameter_reference.md#BAT1_V_EMPTY), where `n` is the battery number) should be set higher (than without compensation) because the compensated voltage gets used for the estimation (typically set a bit below the expected rest cell voltage when empty after use).

You will then need to calibrate the [Amps per volt divider](#current_divider) in the basic settings screen (in order to get reliable current measurements).

PX4 uses current-based load compensation based on a _real-time estimate_ of the internal resistance of the battery by default (real time estimates are enabled if [BAT1_R_INTERNAL=-1](../advanced_config/parameter_reference.md#BAT1_R_INTERNAL)).
Using a real time estimate allows the compensation to adapt to changes in the internal resistance of the battery due to temperature changes during flight, as well as over time as the battery degrades.

The internal resistance can also be measured and [set manually](../advanced_config/parameters.md) in [BAT1_R_INTERNAL](../advanced_config/parameter_reference.md#BAT1_R_INTERNAL).
A positive value in this parameter will be used for the internal resistance instead of the estimated value (`0` disables load compensation altogether).

:::info
There are LiPo chargers that can measure the internal resistance of your battery.
A typical value for LiPo batteries is 5mΩ per cell but this can vary with discharge current rating, age and health of the cells.
:::

## Voltage-based Estimation Fused with Current Integration {#current_integration}

This method is the most accurate way to measure relative battery consumption.
부팅시마다 새로 충전한 배터리를 정확하게 설정하면, 추정 품질이 스마트 배터리의 품질과 비슷해질 것입니다 (이론적으로 정확한 잔여 비행 시간 추정이 가능합니다).

The method evaluates the remaining battery capacity by _fusing_ the voltage-based estimate for the available capacity with a current-based estimate of the charge that has been consumed.
전류를 정확하게 측정할 수있는 장치가 필요합니다.

이 기능을 활성화하려면:

1. First set up accurate voltage estimation using [load compensation](#load_compensation).

   :::tip
   Including calibrating the [Amps per volt divider](#current_divider) setting.

:::

2. Set the parameter [BAT1_CAPACITY](../advanced_config/parameter_reference.md#BAT1_CAPACITY) to around 90% of the advertised battery capacity (usually printed on the battery label).

   ::: info
   Do not set this value too high as this may result in a poor estimation or sudden drops in estimated capacity.

:::

---

**Additional information**

시간 경과에 따라 소비된 전하의 추정치는 측정된 전류를 수학적으로 통합하여 생성됩니다 (이 접근법은 매우 정확한 소비 에너지의 추정치를 제공합니다).

시스템 시작시 PX4는 먼저 전압 기반 추정치를 사용하여 초기 배터리 충전량을 결정합니다. 그런 다음이 추정치를 현재 통합의 값과 융합하여 더 나은 추정치를 제공합니다.
융합된 각 추정치에 배치된 상대값은 배터리 상태에 따라 달라집니다.
배터리가 비워 질수록 전압 기반 추정치가 더 많이 융합됩니다. 이는 과방전을 방지합니다 (예 : 잘못된 용량으로 구성되었거나, 초기치가 잘못 되었기 때문).

항상 정상적인 전체 배터리로 시작하는 경우이 방법은 스마트 배터리에서 사용하는 방법과 유사합니다.

:::info
Current integration cannot be used on its own (without voltage-based estimation) because it has no way to determine the _initial_ capacity.
전압 추정을 사용하면 초기 용량을 추정하고 가능한 오류에 대한 지속적인 피드백을 제공할 수 있습니다 (예 : 배터리에 결함이 있거나 다른 방법을 사용하여 계산된 용량간에 불일치가 있는 경우).
:::
