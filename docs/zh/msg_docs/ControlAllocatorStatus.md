---
pageClass: is-wide-page
---

# ControlAllocatorStatus (UORB message)

**TOPICS:** control_allocatorstatus

## Fields

| 参数名                                                                                       | 类型           | Unit [Frame] | Range/Enum | 描述                                                                                                                                                                  |
| ----------------------------------------------------------------------------------------- | ------------ | ---------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                                                                 | `uint64`     |                                                                  |            | time since system start (microseconds)                                                                                                           |
| torque_setpoint_achieved                        | `bool`       |                                                                  |            | Boolean indicating whether the 3D torque setpoint was correctly allocated to actuators. 0 if not achieved, 1 if achieved.           |
| unallocated_torque                                                   | `float32[3]` |                                                                  |            | Unallocated torque. Equal to 0 if the setpoint was achieved.                                                                        |
| thrust_setpoint_achieved                        | `bool`       |                                                                  |            | Boolean indicating whether the 3D thrust setpoint was correctly allocated to actuators. 0 if not achieved, 1 if achieved.           |
| unallocated_thrust                                                   | `float32[3]` |                                                                  |            | Unallocated thrust. Equal to 0 if the setpoint was achieved.                                                                        |
| actuator_saturation                                                  | `int8[16]`   |                                                                  |            | Indicates actuator saturation status.                                                                                                               |
| handled_motor_failure_mask | `uint16`     |                                                                  |            | Bitmask of failed motors that were removed from the allocation / effectiveness matrix. Not necessarily identical to the report from FailureDetector |
| motor_stop_mask                                 | `uint16`     |                                                                  |            | Bitmaks of motors stopped by failure injection                                                                                                                      |

## Constants

| 参数名                                                                                                                                        | 类型     | 值  | 描述                                                                                                                                                    |
| ------------------------------------------------------------------------------------------------------------------------------------------ | ------ | -- | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a href="#ACTUATOR_SATURATION_OK"></a> ACTUATOR_SATURATION_OK                                    | `int8` | 0  | The actuator is not saturated                                                                                                                         |
| <a href="#ACTUATOR_SATURATION_UPPER_DYN"></a> ACTUATOR_SATURATION_UPPER_DYN | `int8` | 1  | The actuator is saturated (with a value <= the desired value) because it cannot increase its value faster |
| <a href="#ACTUATOR_SATURATION_UPPER"></a> ACTUATOR_SATURATION_UPPER                              | `int8` | 2  | The actuator is saturated (with a value <= the desired value) because it has reached its maximum value    |
| <a href="#ACTUATOR_SATURATION_LOWER_DYN"></a> ACTUATOR_SATURATION_LOWER_DYN | `int8` | -1 | The actuator is saturated (with a value >= the desired value) because it cannot decrease its value faster                          |
| <a href="#ACTUATOR_SATURATION_LOWER"></a> ACTUATOR_SATURATION_LOWER                              | `int8` | -2 | The actuator is saturated (with a value >= the desired value) because it has reached its minimum value                             |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/ControlAllocatorStatus.msg)

:::details
Click here to see original file

```c
uint64 timestamp                        # time since system start (microseconds)

bool torque_setpoint_achieved           # Boolean indicating whether the 3D torque setpoint was correctly allocated to actuators. 0 if not achieved, 1 if achieved.
float32[3] unallocated_torque           # Unallocated torque. Equal to 0 if the setpoint was achieved.
                                        # Computed as: unallocated_torque = torque_setpoint - allocated_torque

bool thrust_setpoint_achieved           # Boolean indicating whether the 3D thrust setpoint was correctly allocated to actuators. 0 if not achieved, 1 if achieved.
float32[3] unallocated_thrust           # Unallocated thrust. Equal to 0 if the setpoint was achieved.
                                        # Computed as: unallocated_thrust = thrust_setpoint - allocated_thrust

int8 ACTUATOR_SATURATION_OK        =  0 # The actuator is not saturated
int8 ACTUATOR_SATURATION_UPPER_DYN =  1 # The actuator is saturated (with a value <= the desired value) because it cannot increase its value faster
int8 ACTUATOR_SATURATION_UPPER     =  2 # The actuator is saturated (with a value <= the desired value) because it has reached its maximum value
int8 ACTUATOR_SATURATION_LOWER_DYN = -1 # The actuator is saturated (with a value >= the desired value) because it cannot decrease its value faster
int8 ACTUATOR_SATURATION_LOWER     = -2 # The actuator is saturated (with a value >= the desired value) because it has reached its minimum value

int8[16] actuator_saturation            # Indicates actuator saturation status.
                                        # Note 1: actuator saturation does not necessarily imply that the thrust setpoint or the torque setpoint were not achieved.
                                        # Note 2: an actuator with limited dynamics can be indicated as upper-saturated even if it as not reached its maximum value.

uint16 handled_motor_failure_mask        # Bitmask of failed motors that were removed from the allocation / effectiveness matrix. Not necessarily identical to the report from FailureDetector
uint16 motor_stop_mask                   # Bitmaks of motors stopped by failure injection
```

:::
