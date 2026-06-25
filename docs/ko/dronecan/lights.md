# DroneCAN Lights

PX4 can control external LEDs on a connected DroneCAN peripheral using the standard DroneCAN [LightsCommand](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#lightscommand) message.

Up to 2 lights are supported.
These can show [system status colours](../getting_started/led_meanings.md#ui-led), a fixed colour (used for indicating aircraft orientation), or switch between both depending on arm state.

## 지원되는 RTK 장치

Any DroneCAN peripheral implementing the standard `LightsCommand` message type should work.

The following have been tested:

- **Vertiq ESC LED add-ons**: Each ESC exposes two light IDs — one RGB (for status) and one white.
  The `light_id` for each is calculated as `esc_index × 3 + BASE_ID`, where `BASE_ID` is 1 for RGB and 2 for white.
  See [Vertiq](../peripherals/vertiq.md) for other ESC setup details.

## PX4 설정

1. Set up DroneCAN as described in [DroneCAN](index.md) (`UAVCAN_ENABLE` ≥ 2).
2. Set [UAVCAN_LGT_NUM](../advanced_config/parameter_reference.md#UAVCAN_LGT_NUM) to the number of lights (1 or 2).
   Then reboot and reopen the ground station so that parameters for the new instances become visible.
3. Set the `light_id` and [light functions](#light_functions) of each light:
   - [UAVCAN_LGT_ID0](../advanced_config/parameter_reference.md#UAVCAN_LGT_ID0) / [UAVCAN_LGT_ID1](../advanced_config/parameter_reference.md#UAVCAN_LGT_ID1): Set to a `light_id` value (as defined by the specific product).
   - [UAVCAN_LGT_FN0](../advanced_config/parameter_reference.md#UAVCAN_LGT_FN0) / [UAVCAN_LGT_FN1](../advanced_config/parameter_reference.md#UAVCAN_LGT_FN1): Choose the desired [light function](#light_functions).
4. Set [UAVCAN_LGT_MODE](#UAVCAN_LGT_MODE) to control when fixed "orientation" colours activate.
5. Reboot for changes to take effect.

### Light Functions {#light_functions}

The functions of enabled lights are configured using [UAVCAN_LGT_FN0](../advanced_config/parameter_reference.md#UAVCAN_LGT_FN0) and [UAVCAN_LGT_FN1](../advanced_config/parameter_reference.md#UAVCAN_LGT_FN1), respectively.
Each function is represented by a value that defines two behaviours: one when the activation mode is **inactive** and one when it is **active**.

| Value | 명칭            | When mode inactive   | When mode active     |
| ----- | ------------- | -------------------- | -------------------- |
| 0     | Status/Status | System status colour | System status colour |
| 1     | Off/White     | Off                  | 흰색                   |
| 2     | Off/Red       | Off                  | 빨강                   |
| 3     | Off/Green     | Off                  | 녹색                   |
| 4     | Status/White  | System status colour | 흰색                   |
| 5     | Status/Red    | System status colour | 빨강                   |
| 6     | Status/Green  | System status colour | 녹색                   |
| 7     | Status/Off    | System status colour | Off                  |

Notes:

- The [system status colours](../getting_started/led_meanings.md#ui-led) is the same LED pattern used by the flight controller's onboard status LED (e.g. red when disarmed, green when armed and ready).
- A fixed colour, commonly used to indicate aircraft orientation. For example it is a common convention to have a red light on the port side, green on starboard, or white to the rear.
  These colours do not change with flight controller state.
- For _hybrid_ functions, such as `Status/Red`, the light shows the Status colour while the activation mode is inactive, then switches to the "fixed" light colour once the mode becomes active.

### Activation Mode (`UAVCAN_LGT_MODE`) {#UAVCAN_LGT_MODE}

The activation mode parameter ([UAVCAN_LGT_MODE](#UAVCAN_LGT_MODE)) controls when each light switches from its _inactive_ to its _active_ behaviour (configured with the [Light function](#light_functions)):

| Value | 설명                                                                          |
| ----- | --------------------------------------------------------------------------- |
| 0     | Always inactive (lights always show the inactive column) |
| 1     | Active when armed (default)                              |
| 2     | Active when prearmed or armed                                               |
| 3     | Always active (lights always show the active column)     |
