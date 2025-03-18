# VTOL Weather Vane Feature

The _weather vane_ feature automatically turns a VTOL vehicle to face its nose into the relative wind during hover flight.
This improves stability (reducing the chance that wind from the side will pick-up the wind-facing wing and flip the vehicle).

The feature is [enabled by default](#configuration) on VTOL hybrid vehicles flying in multicopter mode.

::: info
Weather vane functionality is not supported on pure multirotors.
:::

## Manual Mode Behaviour

The weather vane feature will only take effect in [Position mode](../flight_modes_mc/position.md) (not other manual MC modes).

The user can still use the yaw stick to demand a yaw rate even while the weather vane controller is trying to turn the nose of the vehicle into the wind.
The target yaw rate is the sum of weather vane yaw rate and user commanded yaw rate.

## Mission Mode Behaviour

In [Mission mode](../flight_modes_vtol/mission.md) the weather vane feature will always be active when the parameter is enabled.
Any yaw angle commanded in a mission will be ignored.

<a id="configuration"></a>

## Configuration

This functionality is configured using the [WV\_\* parameters](../advanced_config/parameter_reference.md#WV_EN).

| Parameter                                                              | Description                                                                  |
| ---------------------------------------------------------------------- | ---------------------------------------------------------------------------- |
| [WV_EN](../advanced_config/parameter_reference.md#WV_EN)               | Enable weather vane.                                                         |
| [WV_ROLL_MIN](../advanced_config/parameter_reference.md#WV_ROLL_MIN)   | Minimum roll angle setpoint for weathervane controller to demand a yaw-rate. |
| [WV_YRATE_MAX](../advanced_config/parameter_reference.md#WV_YRATE_MAX) | Maximum yawrate the weathervane controller is allowed to demand.             |

## How Does it Work?

During hover flight the vehicle needs to overcome the drag exerted on it by the wind in order to hold its position.
The only way for it to achieve this is by tilting its thrust vector into the relative wind (it literally 'leans' against the wind).
By keeping track of the thrust vector one can estimate the wind direction.
A weathervane controller is used to command a yawrate that turns the vehicle nose into the estimated wind direction.
