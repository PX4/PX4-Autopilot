# Advanced TECS Tuning (Weight and Altitude)

This topic shows how you can compensate for changes to the [weight of the vehicle](#vehicle-weight-compensation) and the [air density](#air-density-compensation), along with information about the [algorithms](#weight-and-density-compensation-algorithms) that are used.

:::warning
This topic requires that you have already performed [basic TECS tuning](../config_fw/position_tuning_guide_fixedwing.md#tecs-tuning-altitude-and-airspeed).
:::

[Basic TECS tuning](../config_fw/position_tuning_guide_fixedwing.md#tecs-tuning-altitude-and-airspeed) established the key performance limitations of the vehicle that are required for the altitude and airspeed controller to function properly.

While those limitations are specified using constant parameters, in reality vehicle performance is not constant and is affected by various factors.
If changes in weight and air density are not taken into account, altitude and airspeed tracking will likely deteriorate in the case where the configuration (air density and weight) deviate significantly from the configuration at which the vehicle was tuned.

## Vehicle Weight Compensation

Set (both) the following parameters to scale the maximum climb rate, minimum sink rate, and adjust airspeed limits for weight:

- [WEIGHT_BASE](../advanced_config/parameter_reference.md#WEIGHT_BASE) — the weight of the vehicle at which the [Basic TECS tuning](../config_fw/position_tuning_guide_fixedwing.md#tecs-tuning-altitude-and-airspeed) was performed.
- [WEIGHT_GROSS](../advanced_config/parameter_reference.md#WEIGHT_BASE) — the actual weight of the vehicle at any given time, for example when using a larger battery, or with a payload that was not present during tuning.

You can determine the values by measuring the weight of the vehicle using a scale in the tuning configuration and when flying with a payload.

Scaling is performed when _both_ `WEIGHT_BASE` and `WEIGHT_GROSS` are greater than `0`, and will have no effect if the values are the same.
See the [algorithms](#weight-and-density-compensation-algorithms) section below for more information.

## Air Density Compensation

### Specify a Service Ceiling

In PX4 the service ceiling [FW_SERVICE_CEIL](../advanced_config/parameter_reference.md#FW_SERVICE_CEIL) specifies the altitude in standard atmospheric conditions at which the vehicle is still able to achieve a maximum climb rate of 0.5 m/s at maximum throttle and weight equal to [WEIGHT_BASE](../advanced_config/parameter_reference.md#WEIGHT_BASE).
By default this parameter is disabled and no compensation will take place.

This parameter needs to be determined experimentally.
It is always better to set a conservative value (lower value) than an optimistic value.

### Apply Density Correction to Minimum Sink Rate

The minimum sink rate is set in [FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN).

If the [Basic TECS tuning](../config_fw/position_tuning_guide_fixedwing.md#tecs-tuning-altitude-and-airspeed) was not done in standard sea level conditions then the [FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN) parameter must be modified by multiplying with correction factor $P$ (where $\rho$ is the air density during tuning):

$$P = \sqrt{\rho\over{\rho_{sealevel}}}$$

For more information see [Effect of Density on minimum sink rate](#effect-of-density-on-minimum-sink-rate).

### Apply Density Correction to Trim Throttle

The trim throttle is set using [FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM).

If basic tuning was not done in standard sealevel conditions then the value for [FW_THR_TRIM](../advanced_config/parameter_reference.md#FW_THR_TRIM) must be modified by multiplying with correction factor $P$:

$$P = \sqrt{\rho\over{\rho_{sealevel}}}$$

For more information see [Effect of Density on Trim Throttle](#effect-of-density-on-trim-throttle)

## Weight and Density Compensation Algorithms

This section contains information about the scaling operations performed by PX4.
This is provided for interest only, and may be of interest to developers who want to modify the scaling code.

### 표기법

In the following sections we will use the notation $\hat X$ to specify that this value is a calibrated value of the variable $X$.
By calibrated we mean the value of that variable measured at sea level in standard atmospheric conditions, and when vehicle weight was equal to [WEIGHT_BASE](../advanced_config/parameter_reference.md#WEIGHT_BASE).

예: by $\hat{\dot{h}}_{max}$ we specify the maximum climb rate the vehicle can achieve at [WEIGHT_BASE](../advanced_config/parameter_reference.md#WEIGHT_BASE) at sea level in standard atmospheric conditions.

### Effect of Weight on Maximum Climb Rate

The maximum climb rate ([FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX)) is scaled as a function of the weight ratio.

From the steady state equations of motions of an airplane we find that the maximum climb rate can be written as:

$$\dot{h}_{max} = { V * ( Thrust - Drag ) \over{m*g}}$$

where `V` is the true airspeed and `m` is the vehicle mass.
From this equation we see that the maximum climb rates scales with vehicle mass.

### Effect of Weight on Minimum Sink Rate

The minimum sink rate ([FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN)) is scaled as a function of weight ratio

The minimum sink rate can be written as:

$$\dot{h}_{min} = \sqrt{2mg\over{\rho S}} f(C_L, C_D)$$

where $\rho$ is the air density, S is the wing surface reference area and $f(C_L, C_D)$ is a function of the polars, lift and drag.

From this equation we see that the minimum sink rate scales with the square root of the weight ratio.

### Effect of Weight on Airspeed Limits

The minimum airspeed ([FW_AIRSPD_MIN](../advanced_config/parameter_reference.md#FW_AIRSPD_MIN)), the stall airspeed ([FW_AIRSPD_STALL](../advanced_config/parameter_reference.md#FW_AIRSPD_STALL)) and trim airspeed ([FW_AIRSPD_TRIM](../advanced_config/parameter_reference.md#FW_AIRSPD_TRIM)) are adjusted based on the weight ratio specified by [WEIGHT_BASE](../advanced_config/parameter_reference.md#WEIGHT_BASE) and [WEIGHT_GROSS](../advanced_config/parameter_reference.md#WEIGHT_GROSS).

In steady state flight we can demand that lift should equal weight of the vehicle:

$$Lift = mg = {1\over{2}} \rho V^2 S C_L$$

rearranging this equation for airspeed gives:

$$V = \\sqrt{\\frac{2mg}{\\rho S C_D }}$$

From this equation we see that if we assume a constant angle of attack (which we generally desire), the vehicle weight affects airspeed with a square root relation.
Therefore, the airspeed limits mentioned above are all scaled using the square root of the weight ratio.

### Effect of Density on Maximum Climb Rate

The maximum climb rate is set using [FW_T_CLMB_MAX](../advanced_config/parameter_reference.md#FW_T_CLMB_MAX).

As we have seen previously, the maximum climb rate can be formulated as:

$$\dot{h}_{max} = { V * ( Thrust - Drag ) \over{m*g}}$$

The air density affects the airspeed, the thrust and the drag and modelling this effects is not straight forward.
However, we can refer to literature and experience, which suggest that for a propeller airplane the maximum climb rate reduces approximately linear with the air density.
Therefore, we can write the maximum climb rate as:

$$\dot{h}_{max} = \hat{\dot{h}} * {\rho_{sealevel} \over{\rho}} K$$

where $\rho_{sealevel}$ is the air density at sea level in the standard atmosphere and K is a scaling factor which determines the slope of the function.
Rather than trying to identify this constants, the usual practice in aviation is to specify a service ceiling altitude at which the vehicle is still able to achieve a minimum specified climb rate.

### Effect of Density on Minimum Sink Rate

The minimum sink rate is set using [FW_T_SINK_MIN](../advanced_config/parameter_reference.md#FW_T_SINK_MIN).

In previous sections we have seen the formula for the minimum sink rate:

$$\dot{h}_{min} = \sqrt{2mg\over{\rho S}} f(C_L, C_D)$$

This shows that the minimum sink rate scales with the square root of the inverse air density.

### Effect of Density on Trim Throttle

TODO: Add derivation here.
