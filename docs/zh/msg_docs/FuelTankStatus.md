---
pageClass: is-wide-page
---

# FuelTankStatus (UORB message)

**TOPICS:** fuel_tankstatus

## Fields

| 参数名                                                             | 类型        | Unit [Frame] | Range/Enum | 描述                                                                                                                                                                                                                                               |
| --------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| timestamp                                                       | `uint64`  |                                                                  |            | time since system start (microseconds)                                                                                                                                                                                        |
| maximum_fuel_capacity | `float32` |                                                                  |            | maximum fuel capacity. Must always be provided, either from the driver or a parameter                                                                                                                                            |
| consumed_fuel                              | `float32` |                                                                  |            | consumed fuel, NaN if not measured. Should not be inferred from the max fuel capacity                                                                                                                                            |
| fuel_consumption_rate | `float32` |                                                                  |            | fuel consumption rate, NaN if not measured                                                                                                                                                                                                       |
| percent_remaining                          | `uint8`   |                                                                  |            | percentage of remaining fuel, UINT8_MAX if not provided                                                                                                                                                                     |
| remaining_fuel                             | `float32` |                                                                  |            | remaining fuel, NaN if not measured. Should not be inferred from the max fuel capacity                                                                                                                                           |
| fuel_tank_id          | `uint8`   |                                                                  |            | identifier for the fuel tank. Must match ID of other messages for same fuel system. 0 by default when only a single tank exists                                                                                  |
| fuel_type                                  | `uint32`  |                                                                  |            | type of fuel based on MAV_FUEL_TYPE enum. Set to MAV_FUEL_TYPE_UNKNOWN if unknown or it does not fit the provided types |
| temperature                                                     | `float32` |                                                                  |            | fuel temperature in Kelvin, NaN if not measured                                                                                                                                                                                                  |

## Constants

| 参数名                                                                                                                        | 类型      | 值 | 描述                                                                                                                                                                                                                                |
| -------------------------------------------------------------------------------------------------------------------------- | ------- | - | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a href="#MAV_FUEL_TYPE_UNKNOWN"></a> MAV_FUEL_TYPE_UNKNOWN | `uint8` | 0 | fuel type not specified. Fuel levels are normalized (i.e., maximum is 1, and other levels are relative to 1).                                  |
| <a href="#MAV_FUEL_TYPE_LIQUID"></a> MAV_FUEL_TYPE_LIQUID   | `uint8` | 1 | represents generic liquid fuels, such as gasoline or diesel. Fuel levels are measured in millilitres (ml), and flow rates in millilitres per second (ml/s). |
| <a href="#MAV_FUEL_TYPE_GAS"></a> MAV_FUEL_TYPE_GAS         | `uint8` | 2 | represents a gas fuel, such as hydrogen, methane, or propane. Fuel levels are in kilo-Pascal (kPa), and flow rates are in milliliters per second (ml/s).    |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/FuelTankStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp                        # time since system start (microseconds)

float32 maximum_fuel_capacity       	# maximum fuel capacity. Must always be provided, either from the driver or a parameter
float32 consumed_fuel       		# consumed fuel, NaN if not measured. Should not be inferred from the max fuel capacity
float32 fuel_consumption_rate     	# fuel consumption rate, NaN if not measured

uint8 percent_remaining                 # percentage of remaining fuel, UINT8_MAX if not provided
float32 remaining_fuel      		# remaining fuel, NaN if not measured. Should not be inferred from the max fuel capacity

uint8 fuel_tank_id                      # identifier for the fuel tank. Must match ID of other messages for same fuel system. 0 by default when only a single tank exists

uint32 fuel_type                        # type of fuel based on MAV_FUEL_TYPE enum. Set to MAV_FUEL_TYPE_UNKNOWN if unknown or it does not fit the provided types
uint8 MAV_FUEL_TYPE_UNKNOWN = 0		# fuel type not specified. Fuel levels are normalized (i.e., maximum is 1, and other levels are relative to 1).
uint8 MAV_FUEL_TYPE_LIQUID = 1		# represents generic liquid fuels, such as gasoline or diesel. Fuel levels are measured in millilitres (ml), and flow rates in millilitres per second (ml/s).
uint8 MAV_FUEL_TYPE_GAS = 2		# represents a gas fuel, such as hydrogen, methane, or propane. Fuel levels are in kilo-Pascal (kPa), and flow rates are in milliliters per second (ml/s).

float32 temperature                     # fuel temperature in Kelvin, NaN if not measured
```

:::
