# Control Allocation (Mixing)

:::info
Control allocation replaces the legacy mixing approach used in PX4 v1.13 and earlier.
For PX4 v1.13 documentation see: [Mixing & Actuators](https://docs.px4.io/v1.13/en/concept/mixing.html), [Geometry Files](https://docs.px4.io/v1.13/en/concept/geometry_files.html) and [Adding a New Airframe Configuration](https://docs.px4.io/v1.13/en/dev_airframes/adding_a_new_frame.html).
:::

PX4 takes desired torque and thrust commands from the core controllers and translates them to actuator commands which control motors or servos.

The translation depends on the physical geometry of the airframe.
For example, given a torque command to "turn right" (say):

- A plane with one servo per aileron will command one of servo high and the other low.
- A multicopter will yaw right by changing the speed of all motors.

PX4 separates this translation logic, which is referred to as "mixing" from the attitude/rate controller.
This ensures that the core controllers do not require special handling for each airframe geometry, and greatly improves reusability.

In addition, PX4 abstracts the mapping of output functions to specific hardware outputs.
This means that any motor or servo can be assigned to almost any physical output.

<!-- https://docs.google.com/drawings/d/1Li9YhTLc3yX6mGX0iSOfItHXvaUhevO2DRZwuxPQ1PI/edit -->

![Mixing Overview](../../assets/diagrams/mixing_overview.png)

## Actuator Control Pipeline

Overview of the mixing pipeline in terms of modules and uORB topics (press to show full-screen):

<!-- https://drive.google.com/file/d/1L2IoxsyB4GAWE-s82R_x42mVXW_IDlHP/view?usp=sharing -->

![Pipeline Overview](../../assets/concepts/control_allocation_pipeline.png)

참고:

- The rate controller outputs torque and thrust setpoints
- the `control_allocator` module:
  - handles different geometries based on configuration parameters
  - does the mixing
  - handles motor failures
  - publishes the motor and servo control signals
  - publishes the servo trims separately so they can be added as an offset when [testing actuators](../config/actuators.md#actuator-testing) (using the test sliders).
- the output drivers:
  - handle the hardware initialization and update
  - use a shared library [src/libs/mixer_module](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/mixer_module/).
    The driver defines a parameter prefix, e.g. `PWM_MAIN` that the library then uses for configuration.
    Its main task is to select from the input topics and assign the right data to the outputs based on the user set `<param_prefix>_FUNCx` parameter values.
    For example if `PWM_MAIN_FUNC3` is set to **Motor 2**, the 3rd output is set to the 2nd motor from `actuator_motors`.
  - output functions are defined under [src/lib/mixer_module/output_functions.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/mixer_module/output_functions.yaml).
- if you want to control an output from MAVLink, set the relevant output function to **Offboard Actuator Set x**, and then send the [MAV_CMD_DO_SET_ACTUATOR](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ACTUATOR) MAVLink command.

## Adding a new Geometry or Output Function

See [this commit](https://github.com/PX4/PX4-Autopilot/commit/5cdb6fbd8e1352dcb94bd58918da405f8ff930d7) for how to add a new geometry.
The QGC UI will then automatically show the right configuration UI when [CA_AIRFRAME](../advanced_config/parameter_reference.md#CA_AIRFRAME) is set to the new geometry.

[This commit](https://github.com/PX4/PX4-Autopilot/commit/a65533b46986e32254b64b7c92469afb8178e370) shows how to add a new output function.
Any uORB topic can be subscribed and assigned to a function.

Note that parameters for control allocation are defined in [src/modules/control_allocator/module.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/control_allocator/module.yaml)
The schema for this file is [here](https://github.com/PX4/PX4-Autopilot/blob/main/validation/module_schema.yaml#L440=) (in particular, search for the key `mixer:`

## Setting the Default Frame Geometry

When [adding a new frame configuration](../dev_airframes/adding_a_new_frame.md), set the appropriate [CA_AIRFRAME](../advanced_config/parameter_reference.md#CA_AIRFRAME) and other default mixer values for the geometry.

You can see this, for example, in the airframe configuration file [13200_generic_vtol_tailsitter](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d/airframes/13200_generic_vtol_tailsitter)

```sh
...
param set-default CA_AIRFRAME 4
param set-default CA_ROTOR_COUNT 2
param set-default CA_ROTOR0_KM -0.05
param set-default CA_ROTOR0_PY 0.2
...
```

## Setting up Geometry and Outputs

The broad geometry and default parameters for a vehicle are set (from the frame configuration file) when selecting the airframe in QGroundControl: [Basic Configuration > Airframe](../config/airframe.md).

The geometry parameters and output mapping for the specific frame and flight controller hardware are then configured using the QGroundControl **Actuators** setup screen: [Basic Configuration > Actuator Configuration and Testing](../config/actuators.md).
